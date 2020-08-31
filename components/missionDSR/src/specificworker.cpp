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

        // custom_widget
		graph_viewer->add_custom_widget_to_dock("Mission", &custom_widget);
        
        connect(custom_widget.set_pb,SIGNAL(clicked()),this, SLOT(set_mission()));
        connect(custom_widget.delete_pb,SIGNAL(clicked()),this, SLOT(del_mission()));
        
		this->Period = period;
		timer.start(Period);
	}

}

void SpecificWorker::compute()
{
	//computeCODE
	//QMutexLocker locker(mutex);
	//try
	//{
	//  camera_proxy->getYImage(0,img, cState, bState);
	//  memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
	//  searchTags(image_gray);
	//}
	//catch(const Ice::Exception &e)
	//{
	//  std::cout << "Error reading from Camera" << e << std::endl;
	//}
	
	
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

//remove data from node intent
void SpecificWorker::del_mission()
{
    
}

//add data from node intent
void SpecificWorker::set_mission()
{
    auto node = this->get_intent_node();
    
    
}

//get node intent
Node SpecificWorker::get_intent_node()
{
    //TODO: robot name must be obtained from config file
    std::string robot_name = "omnirobot";

    //get intent node
    auto intent_nodes = G->get_nodes_by_type("intent");
    for (auto node : intent_nodes)
    {
        auto parent = G->get_parent_node(node);
        if(not parent.has_value())
            std::cout << __FILE__ << __FUNCTION__ << " Intent node without parent" << std::endl;
        
        if(parent.value().name() == robot_name)
            return node;
    }

    //intent node not found => creation
    if(auto robot = G->get_node(robot_name); robot.has_value())
    {
        Node node;
        node.type("intent");
        G->add_or_modify_attrib_local<parent_att>(node,  robot.value().id());
        G->add_or_modify_attrib_local<level_att>(node, G->get_node_level(robot.value()).value() + 1);
            
        try
        {     
            std::optional<int> new_id = G->insert_node(node);
            if(new_id.has_value())
            {
                Edge edge;
                edge.type("has");
                //get two ids
                edge.from(robot.value().id());
                edge.to(new_id.value());
                if(not G->insert_or_assign_edge(edge))
                {
                    std::cout<<"Error inserting new edge: "<<robot.value().id()<<"->"<<new_id.value()<<" type: has"<<std::endl;
                    std::terminate();
                }
                return node;
            }
            else 
            {
                qDebug() << __FUNCTION__ << "insert_node returned no value for" << QString::fromStdString(node.name());
                return {};
            }
        }
        catch(const std::exception& e)
        {
            std::cout << __FUNCTION__ <<  e.what() << std::endl;
            std::terminate();
        }
    }
    else
    {
        std::cout << __FILE__ << __FUNCTION__ << " No node robot found";
        std::terminate();
    }
    
}





