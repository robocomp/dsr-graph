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
    G.reset();

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
    G = std::make_shared<CRDT::CRDTGraph>(0, agent_name, agent_id); // Init nodes
    
    // read graph content from file
    if(read_file)
    {
        G->read_from_file("grafo.xml");
        G->start_fullgraph_server_thread();     // to receive requests form othe starting agents
        G->start_subscription_thread(true);     // regular subscription to deltas

    }
    else
    {
        G->start_fullgraph_request_thread();    // for agents that want to request the graph for other agent
        //Si se piede el grafo no hace falta iniciar subscription thread, se inicia al sincronizar.
    }

    //sleep(TIMEOUT);
    qDebug() << __FUNCTION__ << "Graph loaded";       
    
    //G->start_subscription_thread(false);
    //G->print();
    
    // GraphViewer creation
    graph_viewer = std::make_unique<DSR::GraphViewer>(std::shared_ptr<SpecificWorker>(this));
    setWindowTitle( agent_name.c_str() );
    qDebug() << __FUNCTION__ << "Graph Viewer started";       
    
    // Random initialization
    mt = std::mt19937(rd());
    dist = std::uniform_real_distribution((float)-40.0, (float)40.0);
    randomNode = std::uniform_int_distribution((int)100, (int)140.0);
    random_selector = std::uniform_int_distribution(0,1);
    //timer.start(300)

    // threads for testing concurrent accesses inside one agent
    threads.resize(5);
    for(int i=0; auto &t : threads)
        t = std::move(std::thread(&SpecificWorker::test_create_or_remove_node, this, i++));
    qDebug() << __FUNCTION__ << "Threads initiated";       
    
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    for(auto &t : threads)
        t.join();
    qDebug() << __FUNCTION__ << "Threads finished";       

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
}


void SpecificWorker::compute()
{
    if (write_string)
        test_set_string(0);
   //   test_nodes_mov();
   // test_node_random();
}

// create and insert a new id in the list
int SpecificWorker::newID()
{
    std::lock_guard<std::mutex>  lock(mut);
    int l = created_nodos.back();
    created_nodos.push_back(++l);
    return l;
}
// pick a random id from the list of new ones
int SpecificWorker::removeID()    
{
    std::lock_guard<std::mutex>  lock(mut);
    if(created_nodos.size()==1)
        return -1;
    auto node_randomizer = std::uniform_int_distribution(0, (int)created_nodos.size()-1);
    int l = node_randomizer(mt);
    int val = created_nodos.at(l);
    created_nodos.erase(created_nodos.begin()+l);
    return val;
}

void SpecificWorker::test_create_or_remove_node(int i)
{
    static int it=0;
    qDebug() << __FUNCTION__ << "Enter thread" << i;
    while (it++ < 1000) 
    {
        // ramdomly select create or remove
        if(random_selector(mt) == 0)
        {
            qDebug() << __FUNCTION__ << "Create node";
            // create node
            Node node;
            node.type("new_type");
            auto id = newID();
            node.id( id );
            node.agent_id(agent_id);
            auto at = AttribValue(); 
            at.key("imName"); at.value("caca");
            node.attrs()["imName"] = at; 
            
            // insert node
            auto r = G->insert_or_assign_node(node);
            if (r)
                qDebug() << "Created node:" << id << " Total size:" << G->size();
        }
        else
        {
            qDebug() << __FUNCTION__ << "Remove node";
            int id = removeID();
            if(id>-1)
            {
                G->delete_node(id);
                qDebug() << "Deleted node:" << id << " Total size:" << G->size();
            }
        }
    }
}

void SpecificWorker::test_set_string(int i)
{
    static int cont = 0;
    qDebug() << __FUNCTION__ << "Enter thread" << i;
    while (cont < 10000) 
    {
        // request node
        Node node = G->get_node(135);
        if (node.id() == -1) 
            return;

        // check for attribute
         if ( node.attrs().find("String") == node.attrs().end() ) 
            return;
        else 
        {
            std::string str = agent_name + "-" + std::to_string(i) + "_" + std::to_string(cont);
            node.attrs()["String"].value() = str;
            // reinsert node
            G->insert_or_assign_node(node);
            qDebug() << __FUNCTION__ << "Strings:" << QString::fromStdString(str);       
        }

        cont++;
    }
}

// void SpecificWorker::test_nodes_mov() {
//     static int cont = 0;
//     if (cont<LAPS) {
//         try {
//             for (auto x : G->get_list()) {
//                 for (auto &[k, v] : x.attrs) {
//                     if(k == "pos_x" || k == "pos_y") {
//                         std::string nValue = std::to_string(std::stoi(v.value) + dist(mt));
//                         cout << "Nodo: "<<x.id<<", antes: "<<v<<", ahora: "<<nValue<<endl;
//                         G->add_node_attrib(x.id, k, v.type, nValue, v.length);
//                     }
//                 }
//             }
//             std::cout<<"Working..."<<cont<<std::endl;
//             cont++;
// //            auto toDelete = randomNode(mt);
// //            std::cout<<"Deleting.... "<<toDelete<<std::endl;
// //            G->delete_node(toDelete);
//         }
//         catch (const Ice::Exception &e) {
//             std::cout << "Error reading from Laser" << e << std::endl;
//         }
//     } else if (cont == LAPS)
//     {
// //        auto to_delete = randomNode(mt);
// ////        int to_delete = 118;
// //        std::cout<<"Antes "<<to_delete<<std::endl;
// //        G->delete_node(to_delete);
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
//             if (G->in(to_move))
//             {
//                 std::cout << "[" << cont << "] to_move: " << to_move << std::endl;
//                 float p_x = G->get_node_attrib_by_name<float>(to_move, "pos_x");
//                 p_x += dist(mt);
//                 float p_y = G->get_node_attrib_by_name<float>(to_move, "pos_y");
//                 p_y += dist(mt);
//                 G->add_node_attrib(to_move, "pos_x", p_x);
//                 G->add_node_attrib(to_move, "pos_y", p_y);
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
//                 G->insert_or_assign(cont, test);
//             }
//             catch (const std::exception &ex) { cerr << __FUNCTION__ << " -> " << ex.what() << std::endl; }

//             if (cont == NODES) {
//                 cont = 0;
//                 laps++;
//             }
//         } else if (laps == LAPS) {
// //            G->print();
//             laps++;
//         } else
//             sleep(5);
//     } catch(const std::exception &e){ std::cout <<__FILE__ << " " << __FUNCTION__ << " "<< e.what() << std::endl;};
// }
