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
    const float threshold = 200; // millimeters
    float rot = 0.6;  // rads per second
    
    auto laser_o = G->get_node("hokuyo_base");
    auto base_o = G->get_node("base");
    if(not laser_o.has_value() or not base_o.has_value()) return;
    auto laser = laser_o.value();
    auto base = base_o.value();

    auto l_angles_o = G->get_attrib_by_name<std::vector<float>>(laser, "laser_data_angles");
    auto l_dists_o = G->get_attrib_by_name<std::vector<float>>(laser, "laser_data_dists");
    if(not l_angles_o.has_value() and not l_dists_o.has_value()) return;
    auto l_angles = l_angles_o.value();
    auto l_dists = l_dists_o.value();

    std::sort( l_dists.begin(), l_dists.end(), [](const float a, const float b){ return  a < b; });      
	if( l_dists.front() < threshold)
	{
		std::cout << l_dists.front() << std::endl;
        G->insert_or_assign_attrib_by_name(base, "advance_speed", 5.0);
        G->insert_or_assign_attrib_by_name(base, "rotation_speed", rot);
		usleep(rand()%(1500000-100000 + 1) + 100000);  // random wait between 1.5s and 0.1sec
	}
	else
	{
        G->insert_or_assign_attrib_by_name(base, "advance_speed", 200.0);
        G->insert_or_assign_attrib_by_name(base, "rotation_speed", 0.0);
  	}
}





