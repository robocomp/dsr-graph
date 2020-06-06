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
	std::cout<< __FUNCTION__ << "Graph loaded" << std::endl;  

	// Graph viewer
	graph_viewer = std::make_unique<DSR::GraphViewer>(G);
	mainLayout.addWidget(graph_viewer.get());
	window.setLayout(&mainLayout);
	setCentralWidget(&window);
    setWindowTitle(QString::fromStdString(agent_name));
	
	
	this->Period = period;
	timer.start(Period);

}

void SpecificWorker::compute()
{
    updateLaser();
}


void SpecificWorker::updateLaser()
{
	RoboCompLaser::TLaserData ldata;
    try
	{
		ldata = laser_proxy->getLaserData();
	}
	catch(const Ice::Exception &e)
	{ std::cout << " Compute: Error reading laser " << e << std::endl;	}
   
    // Transform laserData into two std::vector<float>
    std::vector<float> dists;
    std::transform(ldata.begin(), ldata.end(), std::back_inserter(dists), [](const auto &l) { return l.dist; });
    std::vector<float> angles;
    std::transform(ldata.begin(), ldata.end(), std::back_inserter(angles), [](const auto &l) { return l.angle; });

	// update laser in DSR
	auto node = G->get_node("hokuyo_base");
	if (node.has_value())
	{
		G->insert_or_assign_attrib_by_name(node.value(), "dists", dists);
		G->insert_or_assign_attrib_by_name(node.value(), "angles", angles);
	}
}


