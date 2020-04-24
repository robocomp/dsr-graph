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
    agent_id = stoi(params["agent_id"].value);

    return true;
}

void SpecificWorker::initialize(int period) {
    std::cout << "Initialize worker" << std::endl;

    // create graph
    gcrdt = std::make_shared<CRDT::CRDTGraph>(0, agent_name, agent_id); // Init nodes
    
    // read graph content from file
    if(read_file)
    {
        gcrdt->read_from_file("grafo.xml");
        gcrdt->start_fullgraph_server_thread();
        gcrdt->start_subscription_thread(true);

    }
    else
    {
        gcrdt->start_fullgraph_request_thread();
        //Si se piede el grafo no hace falta iniciar subscription thread, se inicia al sincronizar.
    }

    //sleep(TIMEOUT);
    qDebug() << __FUNCTION__ << "Graph loaded";       
    
    //gcrdt->start_subscription_thread(false);
    //gcrdt->print();
    
    // GraphViewer creation
    graph_viewer = std::make_unique<DSR::GraphViewer>(std::shared_ptr<SpecificWorker>(this));
    setWindowTitle( agent_name.c_str() );
    qDebug() << __FUNCTION__ << "Graph Viewer started";       
    
    // Random initialization
    mt = std::mt19937(rd());
    dist = std::uniform_real_distribution((float)-40.0, (float)40.0);
    randomNode = std::uniform_int_distribution((int)100, (int)140.0);
    random_selector = std::uniform_int_distribution(0,1);
    node_selector = std::uniform_int_distribution(5000,6000);
    //timer.start(300)

    // threads
    threads.resize(1);
    for(int i=0; auto &t : threads)
        t = std::move(std::thread(&SpecificWorker::test_create_or_remove_node, this, i++));
    qDebug() << __FUNCTION__ << "Threads initiated";       
    
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    for(auto &t : threads)
        t.join();
    qDebug() << __FUNCTION__ << "Threads finished";       

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[µs]" << std::endl;
}


void SpecificWorker::compute()
{
    if (write_string)
        test_set_string(0);
   //   test_nodes_mov();
   // test_node_random();
}

void SpecificWorker::test_create_or_remove_node(int i)
{
    static int cont = 0;
    qDebug() << __FUNCTION__ << "Enter thread" << i;
    while (cont < 100) 
    {
        // ramdomly select create or remove
        if(random_selector(mt) == 0)
        {
            qDebug() << __FUNCTION__ << "Create node";
            // create node
            Node node;
            node.type("new_type");
            node.id(5000+cont);
            node.agent_id(agent_id);
            // insert node
            qDebug() << gcrdt->insert_or_assign_node(node) << gcrdt->size();
        }
        // else
        // {
        //     qDebug() << __FUNCTION__ << "Remove node";
        //     int id = node_selector(mt);
        //     bool r = gcrdt->delete_node(id);
        //     if(r==false)
        //         qDebug() << __FUNCTION__ << "Node not found";
        // }
        cont++;
    }
}

void SpecificWorker::test_set_string(int i)
{
    static int cont = 0;
    qDebug() << __FUNCTION__ << "Enter thread" << i;
    while (cont < 1000) 
    {
        // request node
        Node node = gcrdt->get_node(135);
        if (node.id() == -1) 
            return;

        // check for attribute
        //auto val = gcrdt->get_node_attrib_by_name(node, "String");
        auto val = std::find_if(node.attrs().begin(), node.attrs().end(), [](const auto element) { return element.key() == "String"; });
        if (val == node.attrs().end()) 
            return;

        //std::string str = boost::str(boost::format("%s - %d") % agent_name % cont);
        std::string str = agent_name + "-" + std::to_string(i) + "_" + std::to_string(cont);
        val->value(str);

        // reinsert node
        gcrdt->insert_or_assign_node(node);
        qDebug() << __FUNCTION__ << "Strings:" << QString::fromStdString(str);       

        cont++;
    }
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
