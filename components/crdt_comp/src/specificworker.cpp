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
    gcrdt.reset();
    graph.reset();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params) {
    agent_name = params["agent_name"].value;
    return true;
}

void SpecificWorker::initialize(int period) {
    std::cout << "Initialize worker" << std::endl;

    // DSR Graph creation
    graph = std::make_shared<DSR::Graph>();
//    graph->readFromFile("caca.xml");

    gcrdt = std::make_shared<CRDT::CRDTGraph>(0, agent_name, graph); // Init nodes
    gcrdt->read_from_file("caca.xml");

    gcrdt->start_fullgraph_server_thread();
    gcrdt->start_fullgraph_request_thread();
    sleep(TIMEOUT);
    gcrdt->start_subscription_thread(false);
//    gcrdt->print();
    std::cout << "Starting compute" << std::endl;
    timer.start(30);
}


void SpecificWorker::tester() {
    try {
        static int cont = 0, laps = 1;
        if (laps < LAPS) {
            try {
                cont++;
                auto test = RoboCompDSR::Node{
                        "foo_id:" + std::to_string(cont) + "_laps:" + std::to_string(laps) + "_" + agent_name, cont};
//            std::cout <<" New node: "<< test << std::endl;
                gcrdt->insert_or_assign(cont, test);
            }
            catch (const std::exception &ex) { cerr << __FUNCTION__ << " -> " << ex.what() << std::endl; }

            if (cont == NODES) {
                cont = 0;
                laps++;
            }
        } else if (laps == LAPS) {
//            gcrdt->print();
            laps++;
        } else
            sleep(5);
    } catch(const std::exception &e){ std::cout <<__FILE__ << " " << __FUNCTION__ << " "<< e.what() << std::endl;};
}

void SpecificWorker::compute()
{

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
        gcrdt->add_edge_attribs(world_id, base_id, RoboCompDSR::Attribs{std::make_pair("RT", RoboCompDSR::AttribValue{"RTMat",rt.serializeAsString(),1} )});

        // laser update
        auto ldata = laser_proxy->getLaserData();
        std::vector<float> dists;
        std::transform(ldata.begin(), ldata.end(), std::back_inserter(dists), [](const auto &l){ return l.dist;});
        int node_id = gcrdt->get_id_from_name("laser");
        std::string s;
        for (auto & x : dists)
            s += std::to_string(x) + ":";

        gcrdt->add_node_attribs(node_id, RoboCompDSR::Attribs{std::make_pair("laser_data_dists", RoboCompDSR::AttribValue{"vector<float>", s,(int)dists.size()} )});

//        gcrdt->print(node_id);
    }
    catch(const Ice::Exception &e)
    {
        std::cout << "Error reading from Laser" << e << std::endl;
    }

}