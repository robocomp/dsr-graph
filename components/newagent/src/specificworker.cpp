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
#include <QDesktopWidget>
#include <QXmlSimpleReader>
#include <QXmlInputSource>
#include <QXmlDefaultHandler>
#include <QGLViewer/qglviewer.h>
#include <QGraphicsEllipseItem>
#include <cppitertools/range.hpp>
#include <libxml2/libxml/parser.h>
#include <libxml2/libxml/tree.h>


/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx mprx) : GenericWorker(mprx)
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
	//THE FOLLOWING IS JUST AN EXAMPLE
	//To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		std::string innermodel_path = par.value;
		innerModel = new InnerModel(innermodel_path);
	}
	catch(std::exception e) { qFatal("Error reading config params"); }

	return true;
}

void SpecificWorker::initialize(int period)
{
	// Graph creation
	graph = std::make_shared<DSR::Graph>();
	//graph->readFromFile("/home/robocomp/robocomp/files/innermodel/simpleworld-hybrid.xml");
	graph->readFromFile("caca.xml");
	graph->print();

	// CRDT creation and connection to graph
	gcrdt = std::make_unique<DSR::GraphCRDT>(graph, "agent0");
	connect(graph.get(), &DSR::Graph::NodeAttrsChangedSIGNAL, gcrdt.get(), &DSR::GraphCRDT::NodeAttrsChangedSLOT); 
	connect(this, &SpecificWorker::addNodeSIGNAL, gcrdt.get(), &DSR::GraphCRDT::addNodeSLOT);
	connect(this, &SpecificWorker::addEdgeSIGNAL, gcrdt.get(), &DSR::GraphCRDT::addEdgeSLOT);
					
	std::cout << __FILE__ << __FUNCTION__ << " -- Initializing graphic graph" << std::endl;
	graph_viewer->setWidget(this);
	connect(this, &SpecificWorker::addNodeSIGNAL, graph_viewer.get(), &DSR::GraphViewer::addNodeSLOT);
	connect(this, &SpecificWorker::addEdgeSIGNAL, graph_viewer.get(), &DSR::GraphViewer::addEdgeSLOT);
	connect(graph_viewer.get(), &DSR::GraphViewer::saveGraphSIGNAL, this, &SpecificWorker::saveGraphSLOT);
					
	drawGraph();
	std::cout << __FILE__ << __FUNCTION__ << " -- Graph set OK" << std::endl;
	
	///// TESTING to be moved to a unit tests file
	// std::cout << __FILE__ << __FUNCTION__ << " -- Iniializing  innerAPI" << std::endl;
	// innerapi.setGraph(graph);
	
	// //std::cout << __FILE__ << __FUNCTION__ << " -- TEST treewalk" << std::endl;
	// //innerapi.innerModelTreeWalk("world");

	// std::cout << __FILE__ << __FUNCTION__ << " -- TESTS transform" << std::endl;
	// auto r = innerapi.transform("base", QVec::zeros(3), "laser");
	// r.print("Target coordinates from 0, 0, 0");
	
	// std::cout << __FILE__ << __FUNCTION__ << " -- TESTS updatetransformvalues " << std::endl;
	// innerapi.updateTransformValues("base", 20, 30, 40, 50, 60, 70);

	this->Period = 100;
	timer.start(Period);  
	
	std::cout<< __FILE__ << __FUNCTION__ << ":  Initialize OK" << std::endl;
}

void SpecificWorker::compute()
{
	try
	{
		auto lData = laser_proxy->getLaserData();
		RoboCompGenericBase::TBaseState bState;
		differentialrobot_proxy->getBaseState(bState);
		//innerapi.updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
		//auto r = innerapi.transform("world", "base");
		//r.print("transform");
		//std::cout << bState.x << " " << bState.z << std::endl;
		std::vector<float> dists, angles;
		for(const auto &l : lData)
		{
			dists.push_back(l.dist);
			angles.push_back(l.angle);
		}
		auto node_id = graph->getNodeByInnerModelName("laser");
		graph->addNodeAttribs(node_id, DSR::Attribs{ std::pair("laser_data_dists", dists)});
		graph->addNodeAttribs(node_id, DSR::Attribs{ std::pair("laser_data_angles", angles)});  // does not change but needed first time
		
	}
	catch(const Ice::Exception &e)
	{
		std::cout << "Error reading from Laser" << e << std::endl;
	}
}

void SpecificWorker::drawGraph()
{
	std::cout << __FUNCTION__ << "-- Entering drawGraph" << std::endl;
	for(const auto &par : *graph)
	{
		const auto &node_id = par.first;
		float node_posx = graph->getNodeAttribByName<float>(node_id, "pos_x");
		float node_posy = graph->getNodeAttribByName<float>(node_id, "pos_y");
		std::string color_name = graph->getNodeDrawAttribByName<std::string>(node_id, "color");
		std::string qname = graph->getNodeDrawAttribByName<std::string>(node_id, "name");
		std::string type = graph->getNodeType(node_id);
		emit addNodeSIGNAL(node_id, qname, type, node_posx, node_posy, color_name);
	}		

	// add edges after all nodes have been created
	for(const auto &par : *graph)
	{
		auto &node_fanout = graph->fanout(par.first);
		for( auto &[node_adj, edge_atts] : node_fanout)
		{
			auto edge_tag = graph->attr<std::string>(edge_atts.draw_attrs.at("name"));
			emit addEdgeSIGNAL(par.first, node_adj, edge_tag);
		}
	}
}
