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

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx) : GenericWorker(tprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	G->write_to_json_file("/home/robocomp/robocomp/components/dsr-graph/etc/"+agent_name+".json");
	G.reset();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	agent_name = params["agent_name"].value;
    agent_id = stoi(params["agent_id"].value);
	read_dsr = params["read_dsr"].value == "true";
    dsr_input_file = params["dsr_input_file"].value;

	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;

	// create graph
    G = std::make_shared<CRDT::CRDTGraph>(0, agent_name, agent_id); // Init nodes
	// read graph content from file
    if(read_dsr)
    {
        G->read_from_json_file(dsr_input_file);
        G->start_fullgraph_server_thread();     // to receive requests form othe starting agents
        G->start_subscription_thread(true);     // regular subscription to deltas
    }
    else
    {
        G->start_subscription_thread(true);     // regular subscription to deltas
        G->start_fullgraph_request_thread();    // for agents that want to request the graph for other agent
    }
	std::cout<< __FUNCTION__ << "Graph loaded" << std::endl;  

/*************** UNCOMMENT IF NEEDED **********/
	// GraphViewer creation
    graph_viewer = std::make_unique<DSR::GraphViewer>(std::shared_ptr<SpecificWorker>(this));
    setWindowTitle( agent_name.c_str() );

	this->Period = period;
	timer.start(Period);

}

void SpecificWorker::compute()
{
//computeCODE
	std::cout<< "Compute" << std::endl;
//QMutexLocker locker(mutex);
	updateBState();

}

void SpecificWorker::updateBState()
{
	RoboCompGenericBase::TBaseState bState;
    try
	{
		omnirobot_proxy->getBaseState(bState);
		qDebug() << bState.x << bState.z;

	}
	catch(const Ice::Exception &e)
	{
		std::cout << "Error reading bState from omnirobot " << e << std::endl;
	}

	// update bstate in DSR
	auto node = G->get_node("base");
	//auto node = G->get_node(131);
	if (node.id() == -1)
		return;
	
	// print value to check if it is changing
	std::cout<<"Actual bState "<< node.attrs()["x"] << " " << node.attrs()["z"] << " " << node.attrs()["alpha"] <<std::endl;
	G->add_attrib(node.attrs(), "x", std::to_string(bState.x));
	G->add_attrib(node.attrs(), "z", std::to_string(bState.z));
	G->add_attrib(node.attrs(), "alpha", std::to_string(bState.alpha));
	G->add_attrib(node.attrs(), "pos_x", std::to_string(100));
	G->add_attrib(node.attrs(), "pos_y", std::to_string(100));
	
	//qDebug() << "POSX " << QString::fromStdString(node.attrs()["pos_x"].value());
	
	auto r = G->insert_or_assign_node(node);
	if (r)
		std::cout << "Update node robot: "<<node.id()<<std::endl;
}
	// else  //node has to be created
	// {
	// 	try
	// 	{
	// 		int new_id = dsrgetid_proxy->getID();
    //         node.type("robot");
	// 		node.id(new_id);
	// 		node.agent_id(agent_id);
	// 		node.name("robot");
	// 		std::map<string, AttribValue> attrs;
	// 		G->add_attrib(attrs, "x", std::to_string(bState.x));
	// 		G->add_attrib(attrs, "z", std::to_string(bState.z));
	// 		G->add_attrib(attrs, "alpha", std::to_string(bState.alpha));
	// 		node.attrs(attrs);
	// 		auto r = G->insert_or_assign_node(node);
	// 		if (r)
	// 			std::cout << "New node robot: "<<node.id()<<std::endl;
	// 	}
	// 	catch(const std::exception& e)
	// 	{
	// 		std::cerr << "Error creating new node robot " << e.what() << '\n';
	// 	}
	// }






