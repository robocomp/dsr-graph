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
	try
	{
		std::string innermodel_path = params.at("InnerModelPath").value;
		innerModel = new InnerModel(innermodel_path);
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
	
	// DSR Graph creation
	graph = std::make_shared<DSR::Graph>();
	//graph->readFromFile("/home/robocomp/robocomp/files/innermodel/simpleworld-hybrid.xml");
	graph->readFromFile("caca.xml");
	// graph->print();

	// CRDT creation and connection to graph
	gcrdt = std::make_unique<DSR::GraphCRDT>(graph, AGENT_NAME);
	connect(graph.get(), &DSR::Graph::NodeAttrsChangedSIGNAL, gcrdt.get(), &DSR::GraphCRDT::NodeAttrsChangedSLOT); 
	connect(graph.get(), &DSR::Graph::EdgeAttrsChangedSIGNAL, gcrdt.get(), &DSR::GraphCRDT::EdgeAttrsChangedSLOT); 
	gcrdt->startGraphRequestThread();
	std::cout << __FILE__ << __FUNCTION__ << " -- GraphCRDT created OK" << std::endl;
					
	// GraphViewer creation
	std::cout << __FILE__ << __FUNCTION__ << " -- Initializing graphic graph" << std::endl;
	graph_viewer = std::make_unique<DSR::GraphViewer>(std::shared_ptr<SpecificWorker>(this));
	setWindowTitle( AGENT_NAME.c_str() );
	connect(graph_viewer.get(), &DSR::GraphViewer::saveGraphSIGNAL, this, &SpecificWorker::saveGraphSLOT);
	std::cout << __FILE__ << __FUNCTION__ << " -- graphics initialized OK" << std::endl;	

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

	// add laser angle data once to the graph
	auto ldata = laser_proxy->getLaserData();
	std::vector<float> angles;
	std::transform(ldata.begin(), ldata.end(), std::back_inserter(angles), [](const auto &l){ return l.angle;});
	auto node_id = graph->getNodeByInnerModelName("laser");
	graph->addNodeAttribs(node_id, DSR::Attribs{ std::pair("laser_data_angles", angles)});  // does not change but needed first time
	
	this->Period = 100;
	timer.start(Period);  
	
	std::cout<< __FILE__ << __FUNCTION__ << ":  Initialize finshed OK" << std::endl;
}

void SpecificWorker::compute()
{
	//static bool first_time = true;
	try
	{
		// robot update
		RoboCompGenericBase::TBaseState bState;
		differentialrobot_proxy->getBaseState(bState);
		auto base_id = graph->getNodeByInnerModelName("base");
		auto world_id = graph->getNodeByInnerModelName("world");  //OJO: THERE CAN BE SEVERAL EDGES BETWEEN TWO NODES WITH DIFFERENT LABELS
		RMat::RTMat rt;
    	rt.setTr( QVec::vec3(bState.x, 0, bState.z));
		rt.setRX(0.f);rt.setRY(bState.alpha);rt.setRZ(0.f);
		
		//graph->addEdgeAttribs(world_id, base_id, DSR::Attribs{std::make_pair("RT", rt)} );


		//innerapi.updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
		//auto r = innerapi.transform("world", "base");
		//r.print("transform");

		// laser update
		auto ldata = laser_proxy->getLaserData();
		std::vector<float> dists;
		std::transform(ldata.begin(), ldata.end(), std::back_inserter(dists), [](const auto &l){ return l.dist;});
		auto node_id = graph->getNodeByInnerModelName("laser");
		graph->addNodeAttribs(node_id, DSR::Attribs{ std::pair("laser_data_dists", dists)});
	}
	catch(const Ice::Exception &e)
	{
		std::cout << "Error reading from Laser" << e << std::endl;
	}
}

// void SpecificWorker::drawGraph()
// {
// 	std::cout << __FUNCTION__ << "-- Entering drawGraph" << std::endl;
// 	for(const auto &par : *graph)
// 	{
// 		const auto &node_id = par.first;
// 		float node_posx = graph->getNodeAttribByName<float>(node_id, "pos_x");
// 		float node_posy = graph->getNodeAttribByName<float>(node_id, "pos_y");
// 		std::string color_name = graph->getNodeDrawAttribByName<std::string>(node_id, "color");
// 		std::string qname = graph->getNodeDrawAttribByName<std::string>(node_id, "name");
// 		std::string type = graph->getNodeType(node_id);
// 		emit addNodeSIGNAL(node_id, qname, type, node_posx, node_posy, color_name);
// 	}		

// 	// add edges after all nodes have been created
// 	for(const auto &par : *graph)
// 	{
// 		auto &node_fanout = graph->fanout(par.first);
// 		for( auto &[node_adj, edge_atts] : node_fanout)
// 		{
// 			auto edge_tag = graph->attr<std::string>(edge_atts.draw_attrs.at("name"));
// 			emit addEdgeSIGNAL(par.first, node_adj, edge_tag);
// 		}
// 	}
// }
