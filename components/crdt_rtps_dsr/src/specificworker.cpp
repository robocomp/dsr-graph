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
SpecificWorker::SpecificWorker(TuplePrx tprx) : GenericWorker(tprx) {
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker() {
    std::cout << "Destroying SpecificWorker" << std::endl;
    G.reset();

}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params) 
{
    agent_name = params["agent_name"].value;
    read_file = params["read_file"].value == "true";
    agent_id = stoi(params["agent_id"].value);
    return true;
}

void SpecificWorker::initialize(int period) {
    std::cout << "Initialize worker" << std::endl;

    // create graph
    G = std::make_shared<CRDT::CRDTGraph>(0, agent_name, agent_id); // Init nodes

    // read graph content from file
    if(read_file)
    {
        //G->read_from_file("grafo.xml");
        G->read_from_json_file("../../etc/grafo.json");
        G->start_fullgraph_server_thread();
        G->start_subscription_thread(true);
        G->print();
    }
    else
    {
        //De momento no funciona bien
        G->start_fullgraph_request_thread();

    }

    sleep(TIMEOUT);
    //G->start_subscription_thread(false);
    //G->print();
    std::cout << "Starting compute" << std::endl;

    // GraphViewer creation
    graph_viewer = std::make_unique<DSR::GraphViewer>(std::shared_ptr<SpecificWorker>(this));
    setWindowTitle( agent_name.c_str() );

    // Random initialization
    mt = std::mt19937(rd());
    dist = std::uniform_real_distribution((float)-40.0, (float)40.0);
    randomNode = std::uniform_int_distribution((int)100, (int)140.0);
    timer.start(20);
}

void SpecificWorker::compute()
{

}


