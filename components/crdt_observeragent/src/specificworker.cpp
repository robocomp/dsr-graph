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
	gcrdt.reset();
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
	std::cout << __FUNCTION__ << "Initialize " << AGENT_NAME << std::endl;

	gcrdt = std::make_shared<CRDT::CRDTGraph>(0, AGENT_NAME); // Init nodes

	gcrdt->start_fullgraph_request_thread();
	sleep(TIMEOUT);
	gcrdt->start_subscription_thread(false);
//	gcrdt->print();

	// GraphViewer creation
//	std::cout << __FILE__ << __FUNCTION__ << " -- Initializing graphic graph" << std::endl;
	graph_viewer = std::make_unique<DSR::GraphViewer>(std::shared_ptr<SpecificWorker>(this));
//	setWindowTitle( AGENT_NAME.c_str() );

	connect(gcrdt.get(), &CRDT::CRDTGraph::updateNodeSIGNAL, graph_viewer.get(), &DSR::GraphViewer::addNodeSLOT);
//	connect(graph.get(), &DSR::Graph::addEdgeSIGNAL, graph_viewer.get(), &DSR::GraphViewer::addEdgeSLOT);
//	connect(graph.get(), &DSR::Graph::NodeAttrsChangedSIGNAL, graph_viewer.get(), &DSR::GraphViewer::NodeAttrsChangedSLOT);

	//connect(graph.get(), &DSR::Graph::EdgeAttrsChangedSIGNAL, graph_viewer.get(), &DSR::GraphViewer::EdgeAttrsChangedSLOT); 
	
	std::cout << __FILE__ << __FUNCTION__ << " -- graphics initialized OK" << std::endl;

	this->Period = 100;
	timer.start(Period);
}

void SpecificWorker::compute()
{
//	gcrdt->print();
}



