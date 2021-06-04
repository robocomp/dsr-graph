//
// Created by juancarlos on 7/5/20.
//

#include <QtCore/qlogging.h>
#include <QtCore/qdebug.h>
#include "CRDT_concurrent_operations.h"
#include <thread>
#include <fstream>


#include <type_traits>
REGISTER_TYPE(testattrib, std::reference_wrapper<const std::string>, false)

void CRDT_concurrent_operations::concurrent_ops(int i, int no , const std::shared_ptr<DSR::DSRGraph>& G)
{
    int it=0;
    qDebug() << __FUNCTION__ << "Enter thread" << i;
    std::uniform_int_distribution<int> rnd = std::uniform_int_distribution(0, 2);
    std::vector<int> times_th (num_ops);
    while (it++ < no )
    {
        bool r = false;
        std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

        // ramdomly select create or remove
        int x = rnd(mt);
        if( x == 0) {
            if ( rnd_selector())
            {
                // create node
                DSR::Node node;
                node.type("n"); //node.id(id);
                node.agent_id(agent_id);
                //node.name("plane" + std::to_string(id));
                G->add_attrib_local<name_att>(node, std::string("fucking_plane"));
                G->add_attrib_local<color_att>(node, std::string("SteelBlue"));
                G->add_attrib_local<pos_x_att>(node,  rnd_float());
                G->add_attrib_local<pos_y_att>(node,  rnd_float());
                G->add_attrib_local<parent_att>(node,  static_cast<uint64_t>(100));

                auto res = G->insert_node(node);
                if (res.has_value()) {
                    created_nodes.push_back(res.value());
                    qDebug() << "Created node:" << res.value();
                    r = true;
                }
                else
                    qDebug() << "Error inserting node";

            }
            else {
                int id = removeID();
                if (id > -1) {
                    r = G->delete_node(id);
                    if (r)
                        qDebug() << "Deleted node:" << id;
                    else
                        qDebug() << "Error deleting node";
                }
            }
        }
        else if (x == 1)
        {
            if(rnd_selector() == 0)
            {
                DSR::Edge edge;
                edge.type("Edge");
                //get two ids
                edge.from(getID());
                edge.to(getID());
                G->add_attrib_local<name_att>(edge,  std::string("fucking_plane"));
                G->add_attrib_local<color_att>(edge,  std::string("SteelBlue"));

                r = G->insert_or_assign_edge(edge);
                if (r) {
                    addEdgeIDs(edge.from(), edge.to());
                    qDebug() << "Created edge:" << edge.from() << " - " << edge.to();
                }
            }
            else
            {
                //get two ids
                auto [from, to] = removeEdgeIDs();
                r = G->delete_edge(from, to, "Edge");
                if (r)
                    qDebug() << "Deleted edge :"  << from << " - " << to ;

            }
        }

        else if (x == 2)
        {
            std::vector<uint64_t> keys;
            for (auto &x : G->getCopy()){
                keys.emplace_back(x.first);
            }
            std::uniform_int_distribution<int> rnd = std::uniform_int_distribution(0, static_cast<int>(keys.size()-1));
            // request node
            auto nid = keys.at(rnd(mt));
            //qDebug() << __FUNCTION__ << nid;
            if(nid<0) continue;
            std::optional<DSR::Node> node = G->get_node(nid);
            if (!node.has_value())
            {
                qDebug() << "ERROR OBTENIENDO EL NODO";
                //std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
                //times.push_back(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
                std::this_thread::sleep_for(std::chrono::microseconds(delay));
                continue;
            }

            std::string str = std::to_string(agent_id) + "_" + std::to_string(i) + "_" + std::to_string(it);
            if (rnd_selector()) {
                G->add_or_modify_attrib_local<testattrib_att>(node.value(), str) ;
            } else {
                G->remove_attrib_local(node.value(), "testattrib") ;
            }


            G->modify_attrib_local<pos_x_att>(node.value(),  rnd_float());
            G->modify_attrib_local<pos_y_att>(node.value(),  rnd_float());
            node->agent_id(agent_id);
            G->print_node(node->id());
            r = G->update_node(node.value());

            if (!r) {
                qDebug() << "ERROR INSERTANDO EL NODO";
                //std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
                //times.push_back(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
                std::this_thread::sleep_for(std::chrono::microseconds(delay));
                continue;

            }
        }
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        if (r)
            times_th.emplace_back(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
        std::this_thread::sleep_for(std::chrono::microseconds(delay));
    }
    {
        std::unique_lock<std::mutex> lck (mtx);
        times.insert(times.end(), times_th.begin(), times_th.end());
    }
}


void CRDT_concurrent_operations::run_test()
{
    try {
        threads.resize(num_threads);

        for (int i = 0; std::thread &t: threads) {
            t = std::thread(&CRDT_concurrent_operations::concurrent_ops,this, i++, num_ops, G);
        }
        //create_or_remove_nodes(0, G);
        start_global = std::chrono::steady_clock::now();


        for (auto &t: threads) if (t.joinable()) t.join();//out = false;


        qDebug() << __FUNCTION__ << "Test finished";

        end_global = std::chrono::steady_clock::now();
        double time = std::chrono::duration_cast<std::chrono::milliseconds>(end_global - start_global).count();
        result = "CONCURRENT ACCESS: concurrent_operations:"+ MARKER + "OK"+ MARKER + std::to_string(time) + MARKER + "Finished properly ";
        qDebug()<< QString::fromStdString(result);
        mean = static_cast<double>(std::accumulate(times.begin(), times.end(), 0))/(num_ops*num_threads);
        ops_second = num_ops*num_threads*1000/static_cast<double>(std::accumulate(times.begin(), times.end(), 0));
    } catch (std::exception& e) {
        std::cerr << "API TEST: TEST FAILED WITH EXCEPTION "<<  e.what() << " " << std::to_string(__LINE__) + " error file:" + __FILE__ << std::endl;
    }
}

void CRDT_concurrent_operations::save_json_result() {
    G->write_to_json_file(output);

    qDebug()<<"write results"<<QString::fromStdString(output_result);
    std::ofstream out;
    out.open(output_result, std::ofstream::out | std::ofstream::trunc);
    out << result << "\n";
    out << "ops/second"<<MARKER<<ops_second<< "\n";
    out << "mean(ms)"<<MARKER<<mean<< "\n";
    out.close();
}
