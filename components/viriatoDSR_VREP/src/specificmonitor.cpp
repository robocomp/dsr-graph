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
//	aux.editable = true;
//	configGetString( "","InnerModelPath", aux.value, "nofile");
//	params["InnerModelPath"] = aux;
	std::cout << __FUNCTION__ << " Reading parameters from config:" << std::endl;

	configGetString( "","agent_name", aux.value,"");
	params["agent_name"] = aux;
	
	configGetString( "","read_file", aux.value,"true");
	params["read_file"] = aux;

    configGetString( "","agent_id", aux.value,"true");
    params["agent_id"] = aux;

	configGetString( "","dsr_input_file", aux.value, "");
    params["dsr_input_file"] = aux;

	configGetString( "","dsr_output_file", aux.value, "");
    params["dsr_output_file"] = aux;
	
	configGetString( "","test_output_file", aux.value, "");
    params["test_output_file"] = aux;

	configGetString( "","ShowImage", aux.value, "false");
	params["ShowImage"] = aux;

	configGetString( "","CameraName", aux.value, "camera1");
	params["CameraName"] = aux;

	configGetString( "","Publish", aux.value, "false");
	params["Publish"] = aux;
	
	configGetString( "","Depth", aux.value, "false");
	params["Depth"] = aux;

	configGetString( "","Laser", aux.value, "false");
	params["Laser"] = aux;

	configGetString( "","LaserName", aux.value, "hokuyo_base");
	params["LaserName"] = aux;

	configGetString( "","Image", aux.value, "true");
	params["Image"] = aux;

	configGetString( "","RobotName", aux.value, "Viriato");
	params["RobotName"] = aux;

}

//Check parameters and transform them to worker structure
bool SpecificMonitor::checkParams(RoboCompCommonBehavior::ParameterList l)
{
	bool correct = true;
	return correct;
}

