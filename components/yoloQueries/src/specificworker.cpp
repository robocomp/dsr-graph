/*
 *    Copyright (C) 2022 by YOUR NAME HERE
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
		G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, ""); // Init nodes
		std::cout<< __FUNCTION__ << "Graph loaded" << std::endl;  

		//dsr update signals
		connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::modify_node_slot);
		connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::modify_edge_slot);
		connect(G.get(), &DSR::DSRGraph::update_node_attr_signal, this, &SpecificWorker::modify_attrs_slot);
		connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
		connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);

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
	
	get_object_in_container("cup" , "table1");
		
	// auto table3 = G->get_node("table3");
	// set_focus(table3.value());
	//delete_on_focus_edge();
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	//QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

void SpecificWorker::set_focus(DSR::Node &node)
{
	//static bool already_executed = false;

	if(	auto mind = G->get_node("mind") ; mind.has_value())
	{
		// if(!already_executed)
		// {
		//If on_focus edge already exists change the to variable
		if( auto edge_on_focus = G->get_edges_by_type("on_focus"); edge_on_focus.size() > 0)
			edge_on_focus[0].to(node.id());
		else//If does not exists create the edge
		{
			auto edge = DSR::Edge::create<on_focus_edge_type>(mind.value().id(), node.id());
			G->insert_or_assign_edge(edge);
		}
		// 	already_executed = true;
		// }
	}
	else
		qWarning() << "Mind node has no value";
}

bool SpecificWorker::get_object_in_container(string object_type, string container)
{
	
	if(auto objects = G->get_nodes_by_type(object_type) ; objects.size() > 0)
	{
		for( auto object : objects)
		{
			if( auto parent = G->get_parent_node(object); parent.has_value())
			{
				if(parent.value().name() == container)
				{
					std::cout << "####################################EXISTE##################################" << std::endl;
					set_focus(object);
					return true; //First object that fullfill the condition, use a return
				}
			}
			else
			{
				qWarning() << "Object's parent does not exist";
			}
		}
	}
	else
		qWarning() << "Cups size lower than zero";

	return false;
}

bool SpecificWorker::delete_on_focus_edge()
{
	if( auto on_focus_edge = G->get_edges_by_type("on_focus"); on_focus_edge.size() > 0)
		return G->delete_edge(on_focus_edge[0].from(), on_focus_edge[0].to(), "on_focus");
	else
		qWarning() << "on_focus_edge does not exist";
		return false;
}


