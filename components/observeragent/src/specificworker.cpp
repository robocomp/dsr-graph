/*
 *    Copyright (C)2018 by YOUR NAME HERE
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
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//       THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = new InnerModel(innermodel_path);
//	}
//	catch(std::exception e) { qFatal("Error reading config params"); }
	return true;
}

void SpecificWorker::initialize(int period)
{
	qRegisterMetaType<std::int32_t>("std::int32_t");
	qRegisterMetaType<std::string>("std::string");
	qRegisterMetaType<DSR::Attribs>("DSR::Attribs");

	std::cout << "Initialize Agent1" << std::endl;
	// Graph creation
	graph = std::make_shared<DSR::Graph>();
	
	// CRDT creation and connection to graph
	gcrdt = std::make_unique<DSR::GraphCRDT>(graph, "agent1");
	setWindowTitle( "Agent 1" );
	//connect(graph.get(), &DSR::Graph::NodeAttrsChangedSIGNAL, gcrdt.get(), &DSR::GraphCRDT::NodeAttrsChangedSLOT); 
	
	// GraphViewer creation
	std::cout << __FILE__ << __FUNCTION__ << " -- Initializing graphic graph" << std::endl;
	graph_viewer = std::make_unique<DSR::GraphViewer>(std::shared_ptr<SpecificWorker>(this));
	connect(graph.get(), &DSR::Graph::addNodeSIGNAL, graph_viewer.get(), &DSR::GraphViewer::addNodeSLOT);
	connect(graph.get(), &DSR::Graph::addEdgeSIGNAL, graph_viewer.get(), &DSR::GraphViewer::addEdgeSLOT);
	connect(graph.get(), &DSR::Graph::NodeAttrsChangedSIGNAL, graph_viewer.get(), &DSR::GraphViewer::NodeAttrsChangedSLOT); 
	
	std::cout << __FILE__ << __FUNCTION__ << " -- graphics initialized OK" << std::endl;	

	this->Period = 100;
	timer.start(Period);
}

void SpecificWorker::compute()
{

}



