/*
 *    Copyright (C) 2021 by Fernando Martín Ramos-Catalina
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
 
 /*

 */
#include "specificworker.h"
#include<unistd.h>
/*
#pragma push_macro("slots")
#undef slots
#include <Python.h>
#pragma pop_macro("slots")
#include<unistd.h>
*/


/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
	QLoggingCategory::setFilterRules("*.debug=false\n");
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
	agent_name = params["agent_name"].value;
	agent_id = stoi(params["agent_id"].value);
        dsr_output_path = params["dsr_output_path"].value;
	tree_view = params["tree_view"].value == "true";
	graph_view = params["graph_view"].value == "true";
	qscene_2d_view = params["2d_view"].value == "true";
	osg_3d_view = params["3d_view"].value == "true";
	//Getting arguments from the config file. PENDING WORK, NOT WORKING ACTUALLY.
	aggl_file = params["aggl_file"].value;
	init_file = params["init_file"].value;
	aggt_goal = params["aggt_goal"].value;
	result_plan = params["result_plan"].value;

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
		connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::add_or_assign_node_slot);
		connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::add_or_assign_edge_slot);
		//connect(G.get(), &DSR::DSRGraph::update_attrs_signal, this, &SpecificWorker::add_or_assign_attrs_slot);
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

  //We write the dsr JSON
  printf("writing JSON\n");
  G->write_to_json_file(dsr_output_path + agent_name + "_" + std::to_string(output_file_count) + ".json");
  output_file_count++;
  

  //We get the arguments from the config file. PENDING WORK
  //Setting arguments to call the interface of the python agent.
  
  //std::string aggl_file_param = aggl_file;
  
  //This string is getting the dsr JSON file that we write before as init_file.
  //std::string init_file_param = dsr_output_path + agent_name + "_" + std::to_string(output_file_count) + ".json";
  //std::string aggt_goal_param = aggt_goal;
  //std::string result_plan_param = result_plan;
  
  //printf("agent_name:%s, aggl_file:%s , init_file:%s , aggt_goal:%s , result_plan:%s\n",agent_name.c_str(), aggl_file.c_str(),
  //init_file.c_str(), aggt_goal.c_str(), result_plan.c_str());
  
  
  
   //Get the parameters from config is not working right now, so I have to set them here. 
  
  RoboCompAGGLPlanner::Parameters ParameterList;
  ParameterList.pythonarg = "python3 ";
  ParameterList.agglplanarg = "../../../../AGM/AGGLPlanner/agglplan.py "; 
  ParameterList.agglfile = "examples/logistics/domain.aggl "; //Ruleset
  ParameterList.initfile = "examples/logistics/init0.xml "; //init world xml
  ParameterList.aggtgoal = "examples/logistics/prueba0.aggt "; // goal file
  ParameterList.resultplan = "examples/logistics/resultado.plan "; //plan result file
  
  this->agglplanner_proxy->AGGlplannerexecution(ParameterList); //Calling the interface implemented in AGMPlannerPython component.
  printf("done\n");
  
  //PENDING WORK: 
  //It is necessary to add a signal that comes from another component so that instead of executing the scheduler every X seconds,
  //it only executes when the signal is received
  unsigned int microsecond = 1000000;
  usleep(15 * microsecond);//sleeps for 15 second
        
}



int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}


/**************************************/
// From the RoboCompAGGLPlanner you can call this methods:
// this->agglplanner_proxy->AGGlplannerexecution(...)

/**************************************/
// From the RoboCompAGGLPlanner you can use this types:
// RoboCompAGGLPlanner::Parameters


