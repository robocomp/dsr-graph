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
//	RoboCompCommonBehavior::Parameter aux;
//	aux.editable = true;
//	configGetString( "","InnerModelPath", aux.value, "nofile");
//	params["InnerModelPath"] = aux;
	RoboCompCommonBehavior::Parameter aux;
	aux.editable = false;
	configGetString( "","agent_name", aux.value,"noname");
	params["agent_name"] = aux;
	configGetString( "","agent_id", aux.value, "none");
	params["agent_id"] = aux;
	configGetString( "","tree_view", aux.value, "none");
	params["tree_view"] = aux;
	configGetString( "","graph_view", aux.value, "none");
	params["graph_view"] = aux;
	configGetString( "","2d_view", aux.value, "none");
	params["2d_view"] = aux;
	configGetString( "","3d_view", aux.value, "none");
	params["3d_view"] = aux;

    configGetString("","robot_width", aux.value,"400");
    params["robot_width"] = aux;
    configGetString("","robot_length", aux.value,"500");
    params["robot_length"] = aux;
    configGetString("","robot_radius", aux.value,"300");
    params["robot_radius"] = aux;
	configGetString( "","max_advance_speed", aux.value,"500");
	params["max_advance_speed"] = aux;
	configGetString( "","max_side_speed", aux.value,"400");
	params["max_side_speed"] = aux;
	configGetString( "","max_rotation_speed", aux.value,"1");
	params["max_rotation_speed"] = aux;
	configGetString( "","min_controller_period", aux.value,"100");
	params["min_controller_period"] = aux;
    configGetString( "","lateral_correction_gain", aux.value,"0.2");
    params["lateral_correction_gain"] = aux;
    configGetString( "","lateral_correction_for_side_velocity", aux.value,"500");
    params["lateral_correction_for_side_velocity"] = aux;
    configGetString( "","rotation_gain", aux.value,"0.9");
    params["rotation_gain"] = aux;
    configGetString( "","times_final_distance_to_target_before_zero_rotation", aux.value,"3");
    params["times_final_distance_to_target_before_zero_rotation"] = aux;
    configGetString( "","advance_gaussian_cut_x", aux.value,"0.7");
    params["advance_gaussian_out_x"] = aux;
    configGetString( "","advance_gaussian_cut_y", aux.value,"0.2");
    params["advance_gaussian_out_y"] = aux;
    configGetString( "","final_distance_to_target", aux.value,"200");
    params["final_distance_to_target"] = aux;

}

//Check parameters and transform them to worker structure
bool SpecificMonitor::checkParams(RoboCompCommonBehavior::ParameterList l)
{
	bool correct = true;
	return correct;
}

