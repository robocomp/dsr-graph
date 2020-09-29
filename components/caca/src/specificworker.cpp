/*
 *    Copyright (C) 2020 by YOUR NAME HERE
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

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
    this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
    std::cout << "Destroying SpecificWorker" << std::endl;
    G->write_to_json_file("./"+agent_name+".json");
    G.reset();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }





    agent_name = params["agent_name"].value;
    agent_id = stoi(params["agent_id"].value);

    tree_view = params["tree_view"].value == "true";
    graph_view = params["graph_view"].value == "true";
    qscene_2d_view = params["2d_view"].value == "true";
    osg_3d_view = params["3d_view"].value == "true";

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
        timer.start(Period);
        // create graph
        G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, "", dsrgetid_proxy); // Init nodes
        std::cout<< __FUNCTION__ << "Graph loaded" << std::endl;

        // Graph viewer
        using opts = DSR::DSRViewer::view;
        int current_opts = 0;
        opts main = opts::none;
        if(tree_view)
        {
            current_opts = current_opts | opts::tree;
        }
        if(graph_view)
        {
            current_opts = current_opts | opts::graph;
            main = opts::graph;
        }
        if(qscene_2d_view)
        {
            current_opts = current_opts | opts::scene;
        }
        if(osg_3d_view)
        {
            current_opts = current_opts | opts::osg;
        }
        graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
        setWindowTitle(QString::fromStdString(agent_name + "-") + QString::number(agent_id));

        innermodel = G->get_inner_api();
        inner_eigen = G->get_inner_eigen_api();

        this->Period = period;
        timer.setSingleShot(true);
        timer.start(Period);
    }

}

void SpecificWorker::compute()
{
    Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");
    Eigen::IOFormat CleanFmt(Eigen::StreamPrecision, 0, ", ", "\n", "[", "]");

    auto keys = G->getKeys();
    random_selector<> selector{};
    auto t1 = std::chrono::high_resolution_clock::now();
    for (auto i : iter::range(100000))
    {
        auto origen = selector(keys);
        auto dest = selector(keys);
        auto orig_n = G->get_node(origen);
        auto dest_n = G->get_node(dest);
        if (not orig_n.has_value() or not dest_n.has_value())
        {
            std::cout << "nodes " << origen << ", " << dest << std::endl;
            continue;
        };
       // qInfo() << origen << G->get_parent_id(orig_n.value()).value() << dest
       //         << G->get_parent_id(dest_n.value()).value();

        auto orig_name = orig_n.value().name();
        auto dest_name = dest_n.value().name();
        QVec qr = innermodel->transform(dest_name, orig_name).value();
        //qr.print("qmat:");
        //innermodel->get_transformation_matrix(dest_name, orig_name).value().print("trans mat: ");
        //  qInfo() << "-----------------------------";
    }
    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
    std::cout << duration << std::endl;

    t1 = std::chrono::high_resolution_clock::now();
    for (auto i : iter::range(100000))
    {
        auto origen = selector(keys);
        auto dest = selector(keys);
        auto orig_n = G->get_node(origen);
        auto dest_n = G->get_node(dest);
        if (not orig_n.has_value() or not dest_n.has_value())
        {
            std::cout << "nodes " << origen << ", " << dest << std::endl;
            continue;
        };

        auto orig_name = orig_n.value().name();
        auto dest_name = dest_n.value().name();

        auto r = inner_eigen->transform(dest_name, orig_name);
    }
    t2 = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
    std::cout << duration << std::endl;

//        auto r = inner_eigen->transform(dest_name, Eigen::Vector3d(100, -50, 10), orig_name);
//        std::cout << r.value().format(CommaInitFmt) << std::endl;
//        auto tm = inner_eigen->get_transformation_matrix(dest_name, orig_name);
//        std::cout << tm.value().matrix().format(CleanFmt) << std::endl;
//
//        Eigen::Vector3d rs;
//        rs << qr(0), qr(1), qr(2);
//        if( auto error = (r.value() - rs).squaredNorm() ; error > 0.001)
//            std::terminate();
//        else
//            std::cout << "\n" << error << std::endl;
//
//        qInfo() << "-----------------------------";
//        qInfo() << "-----------------------------";

}

int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}




/**************************************/
// From the RoboCompDSRGetID you can call this methods:
// this->dsrgetid_proxy->getID(...)

