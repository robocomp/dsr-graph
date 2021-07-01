/*
 *    Copyright (C)2018 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"
#include <iostream>
#include <QtWidgets/QFileDialog>
#include "tests/CRDT_insert_remove_node.h"
#include "tests/CRDT_change_attribute.h"
#include "tests/CRDT_insert_remove_edge.h"
#include "tests/CRDT_conflict_resolution.h"
#include "tests/CRDT_concurrent_operations.h"


/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
    this->startup_check_flag = startup_check;
    //connect(&autokill_timer, SIGNAL(timeout()), this, SLOT(autokill()));
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker() {
    std::cout << "Destroying SpecificWorker" << std::endl;
    G.reset();

}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params) {
    agent_name = params["agent_name"].value;
    read_file = params["read_file"].value == "true";
    write_string = params["write_string"].value == "true";
    agent_id = stoi(params["agent_id"].value);

    dsr_output_file = params["dsr_output_file"].value;
    dsr_input_file = params["dsr_input_file"].value;
    test_output_file = params["test_output_file"].value;
    dsr_input_file = params["dsr_input_file"].value;
    try {
        delay = std::stoi(params["dsr_test_delay"].value);
    } catch (std::invalid_argument& e)
    {
        qWarning() <<"Invalid value [" << params["dsr_test_delay"].value.c_str() << "] on config argument dsr_test_delay. Setting value to default [10]";
        delay = 10;
    } catch (std::out_of_range & e) {
        qWarning() <<"Out of range value [" << params["dsr_test_delay"].value.c_str() << "] on config argument dsr_test_delay. Setting value to default [10]";
        delay = 10;
    }
    test_name = params["test_name"].value;
    return true;
}

void SpecificWorker::initialize(int period) {
    std::cout << "Initialize worker" << std::endl;

    // create graph
    G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id); // Init nodes
    //G->print();

    // Graph viewer
    using opts = DSR::DSRViewer::view;

    dsr_viewer = std::make_unique<DSR::DSRViewer>(this, G,/*opts::scene|*/opts::graph/*|opts::tree|opts::osg*/, opts::graph);
    setWindowTitle(QString::fromStdString(agent_name));
    connect(actionSave, &QAction::triggered,  [this]()
    {
        auto file_name = QFileDialog::getSaveFileName(this, tr("Save file"), "/home/robocomp/robocomp/components/dsr-graph/etc",
                                                      tr("JSON Files (*.json)"), nullptr,
                                                      QFileDialog::Option::DontUseNativeDialog);
        G->write_to_json_file(file_name.toStdString());
        qDebug() << __FUNCTION__ << "Written";
    });

}

void SpecificWorker::compute()
{
    qDebug()<<"COMPUTE";

    constexpr std::array<std::string_view, 7> tests = { "insert_remove_node", "insert_remove_edge", "change_attribute", "conflict_resolution", "concurrent_operations", "delayed_start", "noname"};
    auto iter = std::find(tests.begin(), tests.end(), test_name);
    [[maybe_unused]]bool exit_ = true;
    switch(std::distance(tests.begin(), iter)){
        case 0: {
            qDebug() << "INSERT AND REMOVE NODES TEST:";
            CRDT_insert_remove_node concurrent_test ( G, dsr_output_file, test_output_file, 2500, 10);
            concurrent_test.run_test();
            std::this_thread::sleep_for(std::chrono::seconds (15));
            concurrent_test.save_json_result();
            break;
        }
        case 1: {
            qDebug() << "INSERT AND REMOVE EDGES TEST:";
            CRDT_insert_remove_edge concurrent_test (G, dsr_output_file, test_output_file, 2500, 10);
            concurrent_test.run_test();
            std::this_thread::sleep_for(std::chrono::seconds (15));
            concurrent_test.save_json_result();
            break;
        }
        case 2: {
            qDebug() << "CHANGE ATTRIBUTES TEST:";
            CRDT_change_attribute concurrent_test ( G, dsr_output_file,  test_output_file,5000, agent_id, 10);
            concurrent_test.run_test();
            std::this_thread::sleep_for(std::chrono::seconds (15));
            concurrent_test.save_json_result();
            break;
        }
        case 3: {
            qDebug() << "CONFLICT RESOLUTION TEST:";
            CRDT_conflict_resolution concurrent_test (G, dsr_output_file, test_output_file, 10000, agent_id, 10);
            concurrent_test.run_test();
            std::this_thread::sleep_for(std::chrono::seconds (15));
            concurrent_test.save_json_result();
            break;
        }
        case 4: {
            qDebug() << "CONCURRENT OPERATIONS TEST:";
            CRDT_concurrent_operations concurrent_test (G, dsr_output_file,  test_output_file, 1000, 20, agent_id, 10);
            concurrent_test.run_test();
            std::this_thread::sleep_for(std::chrono::seconds (15));
            concurrent_test.save_json_result();
            break;
        }
        /*case 5: {
            qDebug() << "DELAYED START TEST:";
            CRDT_delayed_start concurrent_test (G, dsr_output_file,  test_output_file,1200, agent_id, 10);
            concurrent_test.run_test();
            std::this_thread::sleep_for(std::chrono::seconds (20));
            concurrent_test.save_json_result();
            break;
        }*/
        case 6: {
            qDebug() << "DUMMY:";
            exit(0);
        }
        default: {
            qDebug() << "TEST NOT FOUND";
            std::terminate();
        }
    }

    //if (exit_)
    //    exit(0);

}


