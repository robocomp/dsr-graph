/*
 *    Copyright (C)2020 by YOUR NAME HERE
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
#include <QFileDialog>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
    this->startup_check_flag = startup_check;
    QLoggingCategory::setFilterRules("*.debug=false\n");
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	qDebug() << "Destroying SpecificWorker" ;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	try
	{
		agent_name = params["agent_name"].value;
    	agent_id = stoi(params["agent_id"].value);
    	dsr_input_file = params["dsr_input_file"].value;
   	 	dsr_output_path = params["dsr_output_path"].value;
		dsr_write_to_file = params["dsr_write_to_file"].value == "true";
    	this->Period = stoi(params["period"].value);
		tree_view = (params["tree_view"].value == "true") ? DSR::DSRViewer::view::tree : 0;
		graph_view = (params["graph_view"].value == "true") ? DSR::DSRViewer::view::graph : 0;
		qscene_2d_view = (params["2d_view"].value == "true") ? DSR::DSRViewer::view::scene : 0;
		osg_3d_view = (params["3d_view"].value == "true") ? DSR::DSRViewer::view::osg : 0;
	}
	catch(const std::exception &e) { qFatal("Error reading config params"); }
	return true;
}

void SpecificWorker::initialize(int period)
{
    std::cout << "Initialize worker" << std::endl;
    this->Period = period;
    if(this->startup_check_flag)
    {
        this->startup_check();
    }
    else
    {
        // create graph
        G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, dsr_input_file);
        std::cout << __FUNCTION__ << "Graph loaded" << std::endl;

        // Graph viewer
        using opts = DSR::DSRViewer::view;
		int current_opts = tree_view | graph_view | qscene_2d_view | osg_3d_view;
        opts main = opts::none;
        if (graph_view)
		{
        	main = opts::graph;
		}
		dsr_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts);
		setWindowTitle(QString::fromStdString(agent_name + "-" + dsr_input_file));
        connect(actionSaveToFile, &QAction::triggered, [this]() {
            auto file_name = QFileDialog::getSaveFileName(this, tr("Save file"),
                                                          "/home/robocomp/robocomp/components/dsr-graph/etc",
                                                          tr("JSON Files (*.json)"), nullptr,
                                                          QFileDialog::Option::DontUseNativeDialog);
            G->write_to_json_file(file_name.toStdString());
            qDebug() << __FUNCTION__ << "Written";
        });
        this->Period = 100;

        // Compute max Id in G
        get_max_id_from_G();
        if (dsr_write_to_file)
            timer.start(Period);
    }
}

void SpecificWorker::compute()
{
	G->write_to_json_file(dsr_output_path + agent_name + "_" + std::to_string(output_file_count) + ".json");
    output_file_count++;
}

void SpecificWorker::get_max_id_from_G()
{
	for (const auto &[key, node] : G->getCopy())
        if (node.id() > node_id)
            node_id = node.id();
    qDebug() << "MAX ID from file:" << node_id;
}

//////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}


////////////////////////////////////////////////////////
//// IMPLEMENTS SECTION
///////////////////////////////////////////////////////

int SpecificWorker::DSRGetID_getID()
{
	QMutexLocker locker(mutex);
	node_id++;
	return node_id;
}


