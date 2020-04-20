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
#include <iostream>
#include <boost/format.hpp>
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

}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params) {
    agent_name = params["agent_name"].value;
    read_file = params["read_file"].value == "true";
    write_string = params["write_string"].value == "true";

    return true;
}

void SpecificWorker::initialize(int period) {
    std::cout << "Initialize worker" << std::endl;

    // create graph
    gcrdt = std::make_shared<CRDT::CRDTGraph>(0, agent_name); // Init nodes

    // read graph content from file
    if(read_file)
    {
        gcrdt->read_from_file("grafo.xml");
        gcrdt->start_fullgraph_server_thread();
        gcrdt->start_subscription_thread(true);

    }
    else
    {
        //De momento no funciona bien
        gcrdt->start_fullgraph_request_thread();

        // wait until graph read
        sleep(TIMEOUT*2);
        //gcrdt->start_fullgraph_server_thread();
        gcrdt->start_subscription_thread(true);
    }

    sleep(TIMEOUT);
    //gcrdt->start_subscription_thread(false);
    //gcrdt->print();
    std::cout << "Starting compute" << std::endl;

    // GraphViewer creation
    graph_viewer = std::make_unique<DSR::GraphViewer>(std::shared_ptr<SpecificWorker>(this));
    setWindowTitle( agent_name.c_str() );

    // Random initialization
    mt = std::mt19937(rd());
    dist = std::uniform_real_distribution((float)-40.0, (float)40.0);
    randomNode = std::uniform_int_distribution((int)100, (int)140.0);
    timer.start(2000);
}

void SpecificWorker::compute()
{
    if (write_string)
        test_set_string();
   //   test_nodes_mov();
   // test_node_random();
}


void SpecificWorker::test_set_string()
{
    static int cont = 0;
        try
        {

            int node_id = gcrdt->get_id_from_name("Strings");
            // convert values to string

            if (node_id == -1) return;

            vector<AttribValue> at = gcrdt->get_node_attribs_crdt(node_id);
            auto val = std::find_if(at.begin(), at.end(), [](const auto element) { return element.key() == "String"; });
            //generate string;
            string str;
            if (agent_name == "agent0" )
                //str = std::format("{}-{}", "agente0", cont); C++20
                str = boost::str(boost::format("%s - %d") % agent_name % cont);
            else
                //str = std::format("{}-{}", "agente1", cont);
                str = boost::str(boost::format("%s - %d") % agent_name % cont);

            val->value(str);
            // add attributes to node
            gcrdt->add_node_attribs(node_id, at);
        }
        catch (const Ice::Exception &e) { std::cout << "Error reading from Laser" << e << std::endl; }

        std::cout<<"Working..." << cont << std::endl;
        cont++;

}

// void SpecificWorker::test_nodes_mov() {
//     static int cont = 0;
//     if (cont<LAPS) {
//         try {
//             for (auto x : gcrdt->get_list()) {
//                 for (auto &[k, v] : x.attrs) {
//                     if(k == "pos_x" || k == "pos_y") {
//                         std::string nValue = std::to_string(std::stoi(v.value) + dist(mt));
//                         cout << "Nodo: "<<x.id<<", antes: "<<v<<", ahora: "<<nValue<<endl;
//                         gcrdt->add_node_attrib(x.id, k, v.type, nValue, v.length);
//                     }
//                 }
//             }
//             std::cout<<"Working..."<<cont<<std::endl;
//             cont++;
// //            auto toDelete = randomNode(mt);
// //            std::cout<<"Deleting.... "<<toDelete<<std::endl;
// //            gcrdt->delete_node(toDelete);
//         }
//         catch (const Ice::Exception &e) {
//             std::cout << "Error reading from Laser" << e << std::endl;
//         }
//     } else if (cont == LAPS)
//     {
// //        auto to_delete = randomNode(mt);
// ////        int to_delete = 118;
// //        std::cout<<"Antes "<<to_delete<<std::endl;
// //        gcrdt->delete_node(to_delete);
// //        std::cout<<"Fin "<<std::endl;
//         cont++;
//     } else
//         std::cout<<"nada "<<std::endl;
// }

// void SpecificWorker::test_node_random()
// {   static int cont = 0;
//     if (cont<NODES) {
//         try {
//             int to_move = randomNode(mt);
//             if (gcrdt->in(to_move))
//             {
//                 std::cout << "[" << cont << "] to_move: " << to_move << std::endl;
//                 float p_x = gcrdt->get_node_attrib_by_name<float>(to_move, "pos_x");
//                 p_x += dist(mt);
//                 float p_y = gcrdt->get_node_attrib_by_name<float>(to_move, "pos_y");
//                 p_y += dist(mt);
//                 gcrdt->add_node_attrib(to_move, "pos_x", p_x);
//                 gcrdt->add_node_attrib(to_move, "pos_y", p_y);
//             }
//         } catch (const std::exception &e) {
//             std::cout << "EXCEPTION: " << __FILE__ << " " << __FUNCTION__ << ":" << __LINE__ << " " << e.what()
//                       << std::endl;
//         };
//         cont++;
//     }
// }

// void SpecificWorker::tester() {
//     try {
//         static int cont = 0, laps = 1;
//         if (laps < LAPS) {
//             try {
//                 cont++;
//                 auto test = Node{
//                         "foo_id:" + std::to_string(cont) + "_laps:" + std::to_string(laps) + "_" + agent_name, cont};
// //            std::cout <<" New node: "<< test << std::endl;
//                 gcrdt->insert_or_assign(cont, test);
//             }
//             catch (const std::exception &ex) { cerr << __FUNCTION__ << " -> " << ex.what() << std::endl; }

//             if (cont == NODES) {
//                 cont = 0;
//                 laps++;
//             }
//         } else if (laps == LAPS) {
// //            gcrdt->print();
//             laps++;
//         } else
//             sleep(5);
//     } catch(const std::exception &e){ std::cout <<__FILE__ << " " << __FUNCTION__ << " "<< e.what() << std::endl;};
// }
