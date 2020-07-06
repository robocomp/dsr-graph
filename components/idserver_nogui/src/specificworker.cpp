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
#include <iostream>

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
        return true;
	}
	catch(const std::exception &e) { qFatal("Error reading config params"); }
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;

	// create graph
    G = std::make_shared<CRDT::CRDTGraph>(0, agent_name, agent_id, /*"/home/juancarlos/robocomp/components/dsr-graph/components/crdt_rtps_dsr_tests/src/tests/testfiles/empty_file.json"*/dsr_input_file); // Init nodes

    /*
    //INSERTAR NODO
    CRDT::Node node;
    node.type("n");
    node.id( 7777 );
    node.agent_id(0);
    node.name("plane" + std::to_string(7777));
    G->add_attrib(node, "name", std::string("fucking_plane"));
    G->add_attrib(node, "color", std::string("SteelBlue"));
    G->add_attrib(node, "pos_x", 10.0);
    G->add_attrib(node, "pos_y", 5.0);
    G->add_attrib(node, "parent", 100);

    auto res = G->insert_node(node);
    std::cout << std::boolalpha << res.has_value() << std::endl;
    G->print();

    //ACTUALIZAR NODO

    std::optional<CRDT::Node> n_u = G->get_node(7777);
    if (!n_u.has_value())
    {
        throw std::runtime_error("ERROR OBTENIENDO EL NODO");
    }

    std::string str = std::to_string(agent_id) + "-" + std::to_string(8) + "_" + std::to_string(0);
    CRDT::Attribute ab;
    ab.val().str(str);
    n_u.value().attrs()["testattrib"].write(ab);
    G->add_or_modify(n_u.value(), "pos_x", 7.0);
    G->add_or_modify(n_u.value(), "pos_y", 7.0);

    bool r = G->update_node(n_u.value());
    std::cout << std::boolalpha << r << std::endl;


    G->print();
	//G->print_RT(100);

    //INSERTAR NODO 2

    CRDT::Node node2;
    node2.type("n");
    node2.id( 7778 );
    node2.agent_id(0);
    node2.name("plane" + std::to_string(7778));
    G->add_attrib(node2, "name", std::string("fucking_plane"));
    G->add_attrib(node2, "color", std::string("SteelBlue"));
    G->add_attrib(node2, "pos_x", 5.0);
    G->add_attrib(node2, "pos_y", 10.0);
    G->add_attrib(node2, "parent", 100);

    res = G->insert_node(node);
    std::cout << std::boolalpha << res.has_value() << std::endl;
    G->print();

    //INSERTAR EDGE




    */

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


