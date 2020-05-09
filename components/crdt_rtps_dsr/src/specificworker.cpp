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

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx) : GenericWorker(tprx) {
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker() {
    std::cout << "Destroying SpecificWorker" << std::endl;
    //G.reset();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params) 
{

    agent_id = stoi(params["agent_id"].value);
    dsr_output_file = params["dsr_output_file"].value;
    //dsr_input_file = params["dsr_input_file"].value;
    test_output_file = params["test_output_file"].value;
    dsr_input_file = params["dsr_input_file"].value;    
    return true;
}

void SpecificWorker::initialize(int period) 
{
    std::cout << "Initialize worker" << std::endl;

    // create graph
    G = std::make_shared<CRDT::CRDTGraph>(0, agent_name, agent_id, ""); // Init nodes
    G->print();
    // G->start_subscription_thread(true);     // regular subscription to deltas
    // G->start_fullgraph_request_thread();    // for agents that want to request the graph for other agent
    
    // Graph viewer
	graph_viewer = std::make_unique<DSR::GraphViewer>(G);
	mainLayout.addWidget(graph_viewer.get());
	window.setLayout(&mainLayout);
	setCentralWidget(&window);
    setWindowTitle(QString::fromStdString(agent_name));
    
    // Random initialization
    // mt = std::mt19937(rd());
    // unif_float = std::uniform_real_distribution((float)-40.0, (float)40.0);
    // unif_int = std::uniform_int_distribution((int)100, (int)140.0);
    timer.setSingleShot(true);
    timer.start(100);
}

void SpecificWorker::compute()
{
    auto node_op = G->get_node("world");
    if (node_op.has_value()) 
    {
        auto node = node_op.value();
        G->print_node(node);
        
        qDebug() << "::::::::::::Get attrib by name :::::::::::::::::::::::";
        auto t = G->get_attrib_by_name<std::string>(node, "imType");
            std::cout << t.value_or("error") << std::endl;
            
        qDebug() << ":::::::::::::::::::::::::::::::::::";
        auto edge_op = G->get_edge(node.id(), 131, "RT");
        if(edge_op.has_value()) 
        {
            auto edge = edge_op.value();
            G->print_edge(edge);
            
            qDebug() << ":::::::::::::::::::::::::::::::::::";
            auto edges = G->get_edges(node.id());
            //for (auto e: edges)
            //    e.print();    
       
            qDebug() << ":::::::::::::::::::::::::::::::::::";
            auto v = G->get_attrib_by_name<QVec>(edge, "translation");
            if (v.has_value())
                v->print("QVec");
            
            qDebug() << ":::::::::::::::::::::::::::::::::::";
            auto r = G->get_attrib_by_name<QMat>(edge, "rotation_euler_xyz");
                if (r.has_value())
                    r->print("QMat");

        }

        qDebug() << "::::::::::: Get edge from node using to as key ::::::::::::::::::::::::";
        auto m = G->get_edge_RT(node, 131);
        m.value().print("RT"); 

        qDebug() << ":::::::::::::  (Non-existent) return type for a given attribute ::::::::::::::::::::";
        try
        {
            auto c = G->get_attrib_by_name<std::string>(node, "color");
            std::cout  << c.value() << '\n';
        }
        catch(const std::exception &e)
        { 
            std::cout << e.what() << '\n'; 
            QApplication::quit();
        }
    }
   
    auto no = G->get_node("world");
    if(no.has_value())
    {
        qDebug() << "::::::::::::Add attribute to node by name :::::::::::::::::";
        try
        { 
            G->insert_or_assign_attrib_by_name(no.value(), "caca1", std::vector<float>{2,2,2}); 
        }
        catch(const std::exception &e)
        {  std::cout << e.what() << '\n';}

        qDebug() << "::::::::::::Add node RT to node by name :::::::::::::::::";
        try
        { 
            G->get_edge_RT(no.value(), 131).value().print("antes");
            G->insert_or_assign_edge_RT(no.value(), 131, std::vector<float>{6,6,6}, std::vector<float>{2,2,2}); 
            G->get_edge_RT(no.value(), 131).value().print("despues");
        }
        catch(const std::exception &e)
        {  std::cout << e.what() << '\n';}
    }

    qDebug() << ":::::::::: Inner API transform from base to world :::::::::::::::::::::::::";
    auto innermodel = G->get_inner_api();
    auto r = innermodel->transform("world", "base");
    if(r.has_value())
        r.value().print("r");
}


