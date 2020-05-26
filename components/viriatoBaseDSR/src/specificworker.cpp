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
{}

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
    G = std::make_shared<CRDT::CRDTGraph>(0, agent_name, agent_id, ""); // Init nodes
	std::cout<< __FUNCTION__ << "Graph loaded" << std::endl;  

	// Graph viewer
	using g_opts = std::list<DSR::GraphViewer::View>;
	graph_viewer = std::make_unique<DSR::GraphViewer>(G, g_opts{DSR::GraphViewer::View::Scene, DSR::GraphViewer::View::OSG});
	mainLayout.addWidget(graph_viewer.get());
	window.setLayout(&mainLayout);
	setCentralWidget(&window);
    setWindowTitle(QString::fromStdString(agent_name));

	this->Period = period;
	timer.start(Period);
}

void SpecificWorker::compute()
{
	updateBState();
	checkNewCommand();
}

void SpecificWorker::updateBState()
{
	static RoboCompGenericBase::TBaseState last_state;
    try
	{
		omnirobot_proxy->getBaseState(bState);
	}
	catch(const Ice::Exception &e)
	{ std::cout << "Error reading bState from omnirobot " << e << std::endl; }

	// update bstate in DSR
	auto floor = G->get_node(101);
	if (not floor.has_value()) return;
	if( areDifferent(bState.x, last_state.x, FLT_EPSILON) or areDifferent(bState.z, last_state.z, FLT_EPSILON) or areDifferent(bState.alpha, last_state.alpha, FLT_EPSILON))
	{
		G->insert_or_assign_edge_RT(floor.value(), 111, std::vector<float>{bState.x, 0., bState.z}, std::vector<float>{0., bState.alpha, 0.});
		qDebug() << "DONE--------";
		last_state = bState;
	}
}

// Check if rotation_speed or advance_speed have changed and move the robot consequently
void SpecificWorker::checkNewCommand()
{
	auto base = G->get_node("base");
	if(not base.has_value())return;

	auto desired_z_speed = G->get_attrib_by_name<float>(base.value(), "advance_speed");
	auto desired_rot_speed = G->get_attrib_by_name<float>(base.value(), "rotation_speed");
	if(not desired_z_speed.has_value() or not desired_rot_speed.has_value()) return;

	// Check de values are within robot's accepted range. Read them from config
	const float lowerA = -1000, upperA = 1000, lowerR = -1, upperR = 1;
	if( !(lowerA < desired_z_speed.value() and desired_z_speed.value() < upperA and lowerR < desired_rot_speed.value() and desired_rot_speed.value() < upperR))
	{
		qDebug() << __FUNCTION__ << "Desired speed values out of bounds:" << desired_z_speed.value() << desired_rot_speed.value() << "where bounds are:" << lowerA << upperA << lowerR << upperR;
		return;
	}

	if( areDifferent(bState.advVz, desired_z_speed.value(), FLT_EPSILON) or areDifferent(bState.rotV, desired_rot_speed.value(), FLT_EPSILON))
	{
		qDebug() << __FUNCTION__ << "Diff detected" << desired_z_speed.value() << bState.advVz << desired_rot_speed.value() << bState.rotV;
		try
		{
			omnirobot_proxy->setSpeedBase(0, desired_z_speed.value(), desired_rot_speed.value());
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

