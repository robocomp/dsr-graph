/*
 *    Copyright (C) 2010 by RoboLab - University of Extremadura
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
#include "specificmonitor.h"
/**
* \brief Default constructor
*/
SpecificMonitor::SpecificMonitor(GenericWorker *_worker,Ice::CommunicatorPtr _communicator):GenericMonitor(_worker, _communicator)
{
	ready = false;
}
/**
* \brief Default destructor
*/
SpecificMonitor::~SpecificMonitor()
{
	std::cout << "Destroying SpecificMonitor" << std::endl;
}

void SpecificMonitor::run()
{
	initialize();
	ready = true;
	forever
	{
		//rDebug("specific monitor run");
		this->sleep(period);
	}
}

/**
 * \brief Reads components parameters and checks set integrity before signaling the Worker thread to start running
 *   (1) Ice parameters
 *   (2) Local component parameters read at start
 *
 */
void SpecificMonitor::initialize()
{
	rInfo("Starting monitor ...");
	initialTime=QTime::currentTime();
	RoboCompCommonBehavior::ParameterList params;
	readPConfParams(params);
	readConfig(params);
	if(!sendParamsToWorker(params))
	{
		rError("Error reading config parameters. Exiting");
		killYourSelf();
	}
	state = RoboCompCommonBehavior::State::Running;
	emit initializeWorker(period);
}

bool SpecificMonitor::sendParamsToWorker(RoboCompCommonBehavior::ParameterList params)
{
	if(checkParams(params))
	{
		//Set params to worker
		if(worker->setParams(params)) 
			return true;
	}
	else
	{
		rError("Incorrect parameters");
	}
	return false;

}

///Local Component parameters read at start
///Reading parameters from config file or passed in command line, with Ice machinery
///We need to supply a list of accepted values to each call
void SpecificMonitor::readConfig(RoboCompCommonBehavior::ParameterList &params )
{
	RoboCompCommonBehavior::Parameter aux;
	aux.editable = false;
	configGetString( "","agent_name", aux.value,"");
	params["agent_name"] = aux;
	configGetString( "","agent_id", aux.value,"false");
	params["agent_id"] = aux;
	configGetString( "","tree_view", aux.value, "none");
	params["tree_view"] = aux;
	configGetString( "","graph_view", aux.value, "none");
	params["graph_view"] = aux;
	configGetString( "","2d_view", aux.value, "none");
	params["2d_view"] = aux;
	configGetString( "","3d_view", aux.value, "none");
	params["3d_view"] = aux;

    configGetString("", "tile_size", aux.value,"250");
    params["tile_size"] = aux;
	configGetString("", "robot_name", aux.value,"robot");
	params["robot_name"] = aux;
	configGetString("","robot_width", aux.value,"500");
	params["robot_width"] = aux;
	configGetString("","robot_length", aux.value,"500");
	params["robot_length"] = aux;
	configGetString("","robot_radius", aux.value,"300");
	params["robot_radius"] = aux;
	configGetString("","excluded_objects_in_collision_check", aux.value,"floor");
	params["excluded_objects_in_collision_check"] = aux;
    configGetString("", "number_of_not_visible_points", aux.value, "0");
    params["number_of_not_visible_points"] = aux;
    configGetString("", "external_forces_gain", aux.value,"30");
    params["external_forces_gain"] = aux;
    configGetString("", "internal_forces_gain", aux.value,"10");
    params["internal_forces_gain"] = aux;
    configGetString("", "social_forces_gain", aux.value,"40");
    params["social_forces_gain"] = aux;
    configGetString("", "delta_derivation_step", aux.value,"50");
    params["delta_derivation_step"] = aux;
    configGetString("", "max_laser_range", aux.value,"4000");
    params["max_laser_range"] = aux;
    configGetString("", "max_free_energy_iterations", aux.value,"3");
    params["max_free_energy_iterations"] = aux;
    configGetString("", "max_total_energy_ratio", aux.value,"10");
    params["max_total_energy_ratio"] = aux;
}

//Check parameters and transform them to worker structure
bool SpecificMonitor::checkParams(RoboCompCommonBehavior::ParameterList l)
{
	bool correct = true;
	return correct;
}

