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
//    graph_viewer = std::make_unique<DSR::GraphViewer>(std::shared_ptr<SpecificWorker>(this));
//    setWindowTitle( agent_name.c_str() );

	this->Period = period;
	timer.start(Period);

}

void SpecificWorker::compute()
{
//computeCODE
//QMutexLocker locker(mutex);
	auto node = G->get_node("hokuyo_1");
	if (node.id() == -1)
		return;

	const vector<float> lAngles = G->get_node_attrib_by_name<vector<float>>(node, "laser_data_angles");
	const vector<float> lDists = G->get_node_attrib_by_name<vector<float>>(node, "laser_data_dists");

	// check if there is any obstacle in fron of robot
	int pos = 0;
	bool rotate = false;
	for (const float &value: lAngles)
	{
		if (fabs(value) < CHECK_ANGLE)
		{
			if (lDists[pos] < MIN_DISTANCE)
			{
				rotate = true;
				break;
			}
		}
		pos++;
	}
	if (rotate) //obstacle in front of robot => rotate
	{
		setBaseSpeed(0, 0.6);
	}
	else //robot keep straight
	{
		setBaseSpeed(250, 0);
	}

}



void SpecificWorker::setBaseSpeed(float adv, float rot)
{
	// update bstate in DSR
	auto node = G->get_node("base");
	if (node.id() == -1)
		return;
	G->add_attrib(node.attrs(), "advSpeed", adv);
	G->add_attrib(node.attrs(), "rotVel", rot);
	
	auto r = G->insert_or_assign_node(node);
	if (r)
		std::cout << "Update node robot: "<<node.id()<< " with speed(adv,rot): "<<adv << "," << rot << std::endl;
}


