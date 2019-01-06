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
	graph_viewer.reset();
	gcrdt.reset();
	graph.reset();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	try
	{
		AGENT_NAME = params.at("AgentName").value;
	}
	catch(std::exception e) { qFatal("Error reading config params"); }
	return true;
}

void SpecificWorker::initialize(int period)
{
	qRegisterMetaType<std::int32_t>("std::int32_t");
	qRegisterMetaType<std::string>("std::string");
	qRegisterMetaType<DSR::Attribs>("DSR::Attribs");

	std::cout << __FUNCTION__ << "Initialize " << AGENT_NAME << std::endl;
	
	// Graph creation
	graph = std::make_shared<DSR::Graph>();
	
	// CRDT creation and connection to graph
	gcrdt = std::make_unique<DSR::GraphCRDT>(graph, AGENT_NAME);
	//connect(graph.get(), &DSR::Graph::NodeAttrsChangedSIGNAL, gcrdt.get(), &DSR::GraphCRDT::NodeAttrsChangedSLOT); 
	gcrdt->newGraphRequestAndWait();
	//graph->print();
	//gcrdt->startSubscriptionThread();
	//gcrdt->printIceGraph();
	//graph->print();
	
	// GraphViewer creation
	std::cout << __FILE__ << __FUNCTION__ << " -- Initializing graphic graph" << std::endl;
	graph_viewer = std::make_unique<DSR::GraphViewer>(std::shared_ptr<SpecificWorker>(this));   //NO PASAR THIS, SOLO EL WIDGET
	setWindowTitle( AGENT_NAME.c_str() );
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



