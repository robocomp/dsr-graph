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
#include <limits>

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
	updateBState();
<<<<<<< HEAD
	checkNewCommand();
=======
	float adv = 0;
	float rot = 0;
	if (readSpeedCommand(adv, rot))
	{
		setBaseSpeed(adv, rot);
	}
>>>>>>> 39981cecc1ea26e026a85b63f13607fd2fb7a25f
}

void SpecificWorker::updateBState()
{
    try
	{
		omnirobot_proxy->getBaseState(bState);
	}
	catch(const Ice::Exception &e)
	{ std::cout << "Error reading bState from omnirobot " << e << std::endl; }

	// // New API proposal
	// auto [node, success] = G->getNode("base");
	
	// node->addOrAssignAttrib("id", val);
	// auto att = node->getAttrib("id");
	// auto atts = node->getAttribsByType("type");
	// auto edge = node->getEdge(key(to, "type"))
	// node->removeAttrib("id")
	// node->removeEdge(key(to, "type"))
	// RTMat node->getEdgeRTAttrib(to)

	// auto eatt = edge->getAttrib("name");
	// edge->addOrAssignAttrib("name", val);
	// auto eatt = edge->removeAttrib("name");
	// auto eatts = edge->getAttribsByType("type");
	// RTMat edge->getRTAttrib();
	
	// G->insertOrAssignNode(node);
	// G->removeNode(node);

	// update bstate in DSR
	auto base = G->get_node("base");
	if (base.id() == -1)
		return;
	auto &n_at = base.attrs();
	// print value to check if it is changing
	G->add_attrib(n_at, "x", std::to_string(bState.x));
	G->add_attrib(n_at, "z", std::to_string(bState.z));
	G->add_attrib(n_at, "alpha", std::to_string(bState.alpha));
	G->add_attrib(n_at, "pos_x", std::to_string(100));
	G->add_attrib(n_at, "pos_y", std::to_string(100));
		
	G->insert_or_assign_node(base);
}

// Check if rotation_speed or advance_speed have changed and move the robot consequently
void SpecificWorker::checkNewCommand()
{
	auto base = G->get_node("base");
	if(base.id() == -1)
		return;

	const float desired_z_speed = G->get_node_attrib_by_name<float>(base, "advance_speed");
	const float desired_rot_speed = G->get_node_attrib_by_name<float>(base, "rotation_speed");

	// Check de values are within robot's accepted range. Read them from config
	const float lowerA = -1000, upperA = 1000, lowerR = -1, upperR = 1;
	if( !(lowerA < desired_z_speed and desired_z_speed < upperA and lowerR < desired_rot_speed and desired_rot_speed < upperR))
	{
		qDebug() << __FUNCTION__ << "Desired speed values out of bounds:" << desired_z_speed << desired_rot_speed << "where bounds are:" << lowerA << upperA << lowerR << upperR;
		return;
	}

	if( areDifferent(bState.advVz, desired_z_speed, FLT_EPSILON) or areDifferent(bState.rotV, desired_rot_speed, FLT_EPSILON))
	{
		qDebug() << __FUNCTION__ << "Diff detected" << desired_z_speed << bState.advVz << desired_rot_speed << bState.rotV;
		try
		{
			omnirobot_proxy->setSpeedBase(0, desired_z_speed, desired_rot_speed);
		}
		catch(const RoboCompGenericBase::HardwareFailedException &re)
		{ std::cout << re << '\n';}
		catch(const Ice::Exception &e)
		{ std::cout << e.what() << '\n';}
	}
}

bool SpecificWorker::areDifferent(float a, float b, float epsilon)
{ 
	return !((fabs(a - b) <= epsilon * std::max(fabs(a), fabs(b)))); 
};

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
<<<<<<< HEAD
=======


bool SpecificWorker::readSpeedCommand(float &adv, float &rot)
{
	auto node = G->get_node("base");
	//auto node = G->get_node(131);
	if (node.id() != -1)
	{
		try{
			adv = G->get_node_attrib_by_name<float>(node, "advSpeed");
			rot = G->get_node_attrib_by_name<float>(node, "rotVel");
			qDebug()<<"Speed read from DSR: "<<adv << rot;
			return (last_adv != adv or last_rot != rot);
		}catch(...) {}
	}
	return false;
}

void SpecificWorker::setBaseSpeed(float adv, float rot)
{
	if (adv != last_adv or rot != last_rot)
	{
		try
		{
qDebug()<<"set Base speed"<<adv << rot;
			omnirobot_proxy->setSpeedBase(0, adv, rot);
			last_adv = adv;
			last_rot = rot;
		}
		catch(const Ice::Exception &e)
		{
			std::cout << "Error setting speed to omnirobot " << e << std::endl;
		}
	}
}




>>>>>>> 39981cecc1ea26e026a85b63f13607fd2fb7a25f
