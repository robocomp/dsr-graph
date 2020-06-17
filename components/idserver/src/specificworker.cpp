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
#include <QFileDialog>

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
	try
	{
		agent_name = params["agent_name"].value;
    	agent_id = stoi(params["agent_id"].value);
    	dsr_input_file = params["dsr_input_file"].value;
   	 	dsr_output_path = params["dsr_output_path"].value;
		dsr_write_to_file = params["dsr_write_to_file"].value == "true";
    	this->Period = stoi(params["period"].value);
	}
	catch(const std::exception &e) { qFatal("Error reading config params"); }
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;

	// create graph
    G = std::make_shared<CRDT::CRDTGraph>(0, agent_name, agent_id, dsr_input_file); // Init nodes
	G->print();
	G->print_RT(100);

	// Graph viewer
	using opts = DSR::GraphViewer::view;
	graph_viewer = std::make_unique<DSR::GraphViewer>(this, G, opts::scene|opts::osg|opts::graph|opts::tree);
//	mainLayout.addWidget(graph_viewer.get());
//	window.setLayout(&mainLayout);
//	setCentralWidget(&window);
	setWindowTitle(QString::fromStdString(agent_name + "-" + dsr_input_file));
	connect(actionSaveToFile, &QAction::triggered,  [this]()
		{ 
			auto file_name = QFileDialog::getSaveFileName(this, tr("Save file"), "/home/robocomp/robocomp/components/dsr-graph/etc", 
															   tr("JSON Files (*.json)"));
			G->write_to_json_file(file_name.toStdString()); 
			qDebug() << __FUNCTION__ << "Written";
		});
	//connect(actionSimulate, &QAction::triggered, graph_viewer.get(), &DSR::GraphViewer::toggleSimulationSLOT);

	// Compute max Id in G
    get_max_id_from_G();
	std::cout<< __FUNCTION__ << ": Graph loaded" << std::endl;  
	if(dsr_write_to_file)
		timer.start(Period);
	
}

void SpecificWorker::compute()
{
	G->write_to_json_file(dsr_output_path + agent_name + "_" + std::to_string(output_file_count) + ".json");
    output_file_count++;
}

void SpecificWorker::get_max_id_from_G()
{
	//auto g = G->getCopy();
	//auto node_id = std::max_element(g.begin(), g.end(), [](const auto &[k1,v1], const auto n1 &[k2,v2]){ return v1.id() > v2.id(); }
	for (const auto &[key, node] : G->getCopy())
        if (node.id() > node_id)
            node_id = node.id();
    qDebug() << "MAX ID from file:" << node_id;
}

////////////////////////////////////////////////////////
//// IMPLEMENTS SECTION
///////////////////////////////////////////////////////

int SpecificWorker::DSRGetID_getID()
{
	QMutexLocker locker(mutex);
	node_id++;
	//Qdebug << "NEW ID:" << node_id;
	return node_id;
}


