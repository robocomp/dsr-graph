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
SpecificWorker::SpecificWorker(TuplePrx tprx) : GenericWorker(tprx)
{
    connect(&autokill_timer, SIGNAL(timeout()), this, SLOT(autokill()));
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

    dsr_output_file = params["dsr_output_file"].value;
    dsr_input_file = params["dsr_input_file"].value;
    test_output_file = params["test_output_file"].value;

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
        G->read_from_json_file(dsr_input_file);
        G->print();
        G->start_fullgraph_server_thread();     // to receive requests form othe starting agents
        G->start_subscription_thread(true);     // regular subscription to deltas

    }
    else
    {
        G->start_subscription_thread(true);     // regular subscription to deltas
        G->start_fullgraph_request_thread();    // for agents that want to request the graph for other agent
    }

    //sleep(5);
    qDebug() << __FUNCTION__ << "Graph loaded";       
    
    // for(auto kv: *G)
    // {
    //     Node node = kv.second.dots().ds.rbegin()->second;
    //     std::cout << node.id() << std::endl;
    // }

    //G->print();
    
    // GraphViewer creation
    graph_viewer = std::make_unique<DSR::GraphViewer>(std::shared_ptr<SpecificWorker>(this));
    setWindowTitle( agent_name.c_str() );

    // qDebug() << __FUNCTION__ << "Graph Viewer started";
    
    // Random initialization
    mt = std::mt19937(rd());
    dist = std::uniform_real_distribution((float)-40.0, (float)40.0);
    randomNode = std::uniform_int_distribution((int)100, (int)140.0);
    random_pos = std::uniform_int_distribution((int)-200, (int)200);
    random_selector = std::uniform_int_distribution(0,1);

    //timer.start(300);
    //autokill_timer.start(10000);
    compute();
}

void SpecificWorker::compute()
{
    qDebug()<<"COMPUTE";
    test_concurrent_access(1);
    //std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    //test_create_or_remove_node(100, 10000, 10);
    // if (write_string)
    //     test_set_string(0);
    //   test_nodes_mov();
    // test_node_random();
    /*int num_ops = 100000;
    test_set_string(0, num_ops, 0);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::string time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count());
    qDebug() << __FUNCTION__ << "Elapsed time:" << QString::fromStdString(time) << "ms for " << num_ops << " ops";*/
    sleep(25);
    G->write_to_json_file(dsr_output_file);
    //exit(0);
}


void SpecificWorker::autokill()
{
    G->write_to_json_file(dsr_output_file);
    exit(0);
}

// This has to be a RPC call to the idserver component
// create and insert a new id in the list
int SpecificWorker::newID()
{
    /*static int node_counter = 5000;
    std::lock_guard<std::mutex>  lock(mut);
    created_nodos.push_back(++node_counter);
    */
    int node_id;
    try{
        node_id = dsrgetid_proxy->getID();
        created_nodos.push_back(node_id);
        qDebug()<<"New nodeID: "<<node_id;
    }catch(...)
    {
        qDebug()<<"Error getting new nodeID from idserver, check connection";
    }
    return node_id;
}
// pick a random id from the list of new ones
int SpecificWorker::removeID()    
{
    std::lock_guard<std::mutex>  lock(mut);
    if(created_nodos.size()==0)
        return -1;
    auto node_randomizer = std::uniform_int_distribution(0, (int)created_nodos.size()-1);
    int l = node_randomizer(mt);
    int val = created_nodos.at(l);
    created_nodos.erase(created_nodos.begin()+l);
    return val;
}

// pick a random id from the list of new ones without removing
int SpecificWorker::getID()
{
    std::lock_guard<std::mutex>  lock(mut);
    if(created_nodos.size()==0)
        return -1;
    auto node_randomizer = std::uniform_int_distribution(0, (int)created_nodos.size()-1);
    int l = node_randomizer(mt);
    int val = created_nodos.at(l);
    return val;
}

pair<int, int> SpecificWorker::removeEdge(){
    std::lock_guard<std::mutex>  lock(mut);
    if(created_edges.size()==0)
        return { -1, -1 };
    auto edge_randomizer = std::uniform_int_distribution(0, (int)created_edges.size()-1);
    int l = edge_randomizer(mt);
    auto val = created_edges.at(l);
    created_edges.erase(created_edges.begin()+l);
    return val;
}


void SpecificWorker::test_concurrent_access(int num_threads)
{
   // threads for testing concurrent accesses inside one agent
    threads.resize(num_threads);
    for(int i=0; auto &t : threads)
        t = std::move(std::thread(&SpecificWorker::test_create_or_remove_node, this, i++, 2000, 10));
    qDebug() << __FUNCTION__ << "Threads initiated";

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    for(auto &t : threads)
        t.join();
    qDebug() << __FUNCTION__ << "Threads finished";

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::string time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count());
    std::string result = "test_concurrent_access: test_create_or_remove_node"+ MARKER + "OK"+ MARKER + time + MARKER + "Finished properly, num_threads " +std::to_string(num_threads);
    write_test_output(result);


    threads.resize(num_threads);
    for(int i=0; auto &t : threads)
    t = std::move(std::thread(&SpecificWorker::test_create_or_remove_edge, this, i++, 100, 10));
    qDebug() << __FUNCTION__ << "Threads initiated";

    begin = std::chrono::steady_clock::now();
    for(auto &t : threads)
        t.join();
    qDebug() << __FUNCTION__ << "Threads finished";

    end = std::chrono::steady_clock::now();
    time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count());
    result = "test_concurrent_access: test_create_or_remove_edge"+ MARKER + "OK"+ MARKER + time + MARKER + "Finished properly, num_threads " +std::to_string(num_threads);
    write_test_output(result);

}
void SpecificWorker::test_create_or_remove_node(int i, int iters, int delay)
{
    static int it=0;
    qDebug() << __FUNCTION__ << "Enter thread" << i;
    while (it++ < iters)
    {
        // ramdomly select create or remove
        if( random_selector(mt)== 0)
        {
            //qDebug() << __FUNCTION__ << "Create node";
            // create node
            Node node;
            node.type("plane");
            auto id = newID();
            node.id( id );
            node.agent_id(agent_id);
            node.name("plane" + std::to_string(id));
            std::map<string, Attribs> attrs;
            G->add_attrib(attrs, "name", std::string("fucking_plane"));
            G->add_attrib(attrs, "color", std::string("SteelBlue"));
            auto dis = std::uniform_real_distribution(-200.0, 200.0);
            G->add_attrib(attrs, "pos_x", (float)dis(mt));
            G->add_attrib(attrs, "pos_y", (float)dis(mt));

            node.attrs(attrs);
            
            // insert node
            auto r = G->insert_or_assign_node(node);
            if (r)
                qDebug() << "Created node:" << id << " Total size:" << G->size();
        }
        else
        {
            //qDebug() << __FUNCTION__ << "Remove node";
            int id = removeID();
            if(id>-1)
            {
                G->delete_node(id);
                qDebug() << "Deleted node:" << id << " Total size:" << G->size();
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(delay));
    }
}


void SpecificWorker::test_create_or_remove_edge(int i, int iter, int delay)
{
    static int it=0;
    qDebug() << __FUNCTION__ << "Enter thread" << i;
    while (it++ < iter)
    {
        // ramdomly select create or remove

        if(random_selector(mt) == 0)
        {
            //qDebug() << __FUNCTION__ << "Create node";
            // create edge

            Edge edge;
            edge.type("Edge");
            //get two ids
            edge.from(getID());
            edge.to(getID());
            std::map<string, Attribs> attrs;
            G->add_attrib(attrs, "name", std::string("fucking_plane"));
            G->add_attrib(attrs, "color", std::string("SteelBlue"));
            edge.attrs(attrs);

            //qDebug() << "[" <<edge.from() << " - " << edge.to()<< "]";
            // insert node
            auto r = G->insert_or_assign_edge(edge);
            if (r) {
                created_edges.emplace_back(make_pair(edge.from(), edge.to()));
                qDebug() << "Created edge:" << edge.from() << " - " << edge.to();
            }
        }
        else
        {
            //qDebug() << __FUNCTION__ << "Remove node";
            int id = removeID();
            if(id>-1)
            {
                //get two ids
                auto [from, to] = removeEdge();
                //qDebug() << " remove [" << from << " - " << to << "]";

                auto r = G->delete_edge(from, to, "Edge");
                if (r)
                    qDebug() << "Deleted edge :"  << from << " - " << to ;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(delay));
    }
}

void SpecificWorker::test_set_string(int i, int iter, int delay)
{
    std::string result;
    static int it = 0;
    qDebug() << __FUNCTION__ << "Enter thread" << i;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    bool fail=false;
    //auto map = G->getCopy();  // provides a deep copy of the graph. Changes in it won't have effect on G
    auto keys = G->getKeys();
    auto node_randomizer = std::uniform_int_distribution(0, (int)keys.size()-1);

    while (it++ < iter)
    {
        // request node
        auto nid = keys.at(node_randomizer(mt));
        //qDebug() << __FUNCTION__ << nid;
        if(nid<0) continue;
        Node node = G->get_node(nid);
        if (node.id() == -1) 
        {
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            std::string time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count());
            result = "test_set_string" + MARKER +"FAIL" + MARKER + time + MARKER +" error getting node->line:" + std::to_string(__LINE__);
            fail = true;
            break;
        }
        // check for attribute
        if ( auto iter = node.attrs().find("imName"); iter != node.attrs().end() )
        {
            //std::string str = agent_name + "-" + std::to_string(i) + "_" + std::to_string(cont);
            auto name = iter->second.value();
            //it->second.value() = std::to_string(random_pos(mt));
            iter->second.value() = name;
            //node.attrs()["String"].value() = str;
            // reinsert node
            G->insert_or_assign_node(node);
            //qDebug() << __FUNCTION__ << "Strings:" << QString::fromStdString(str);
            //qDebug() << __FUNCTION__ << "Strings:" << QString::fromStdString(std::to_string(random_pos(mt)));
        }
        else
         {
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            std::string time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count());
            result = "test_set_string" + MARKER + "FAIL" + MARKER + time + MARKER + " error getting attibutte->line:" + std::to_string(__LINE__);
            fail = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(delay));
    }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::string time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count());
    if (fail==false)
        result = "test_set_string"+ MARKER + "OK"+ MARKER + time + MARKER + "Finished properly";
    write_test_output(result);
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


void SpecificWorker::write_test_output(std::string result)
{
    qDebug()<<"write results"<<QString::fromStdString(test_output_file)<<QString::fromStdString(result);
    std::ofstream out;
    out.open(test_output_file, std::ofstream::out | std::ofstream::app);
    out << result << "\n";
    out.close();
}