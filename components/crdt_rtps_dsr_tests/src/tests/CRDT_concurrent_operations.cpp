//
// Created by juancarlos on 7/5/20.
//

#include <QtCore/qlogging.h>
#include <QtCore/qdebug.h>
#include "CRDT_concurrent_operations.h"
#include "../../../../graph-related-classes/topics/DSRGraph.h"
#include <thread>

void CRDT_concurrent_operations::concurrent_ops(int i, int no , const shared_ptr<CRDT::CRDTGraph>& G)
{
    int it=0;
    qDebug() << __FUNCTION__ << "Enter thread" << i;
    std::uniform_int_distribution<int> rnd = std::uniform_int_distribution(0, 2);

    while (it++ < no )
    {
        // ramdomly select create or remove
        int x = rnd(mt);
        if( x == 0) {
            if ( rnd_selector())
            {
                //qDebug() << __FUNCTION__ << "Create node";
                // create node
                auto id = newID();
                Node node; node.type("n"); node.id(id);
                node.agent_id(agent_id);
                node.name("plane" + std::to_string(id));
                G->add_attrib(node, "name", std::string("fucking_plane"));
                G->add_attrib(node, "color", std::string("SteelBlue"));
                G->add_attrib(node, "pos_x", rnd_float());
                G->add_attrib(node, "pos_y", rnd_float());
                G->add_attrib(node, "parent", 100);

                auto res = G->insert_node(node);
                if (res.has_value())
                    qDebug() << "Created node:" << id;// << " Total size:" << G->size();
                else
                    qDebug() << "Error inserting node";

            }
            else {
                //qDebug() << __FUNCTION__ << "Remove node";
                int id = removeID();
                if (id > -1) {
                    bool r = G->delete_node(id);
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
                Edge edge;
                edge.type("Edge");
                //get two ids
                edge.from(getID());
                edge.to(getID());
                bool r =  G->add_attrib(edge, "name", std::string("fucking_plane"));
                r =  G->add_attrib(edge, "color", std::string("SteelBlue"));

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
                auto r = G->delete_edge(from, to, "Edge");
                if (r)
                    qDebug() << "Deleted edge :"  << from << " - " << to ;

            }
        }

        else if (x == 2)
        {
            auto keys = G->getKeys();
            std::uniform_int_distribution<int> rnd = std::uniform_int_distribution(0, static_cast<int>(keys.size()-1));
            // request node
            auto nid = keys.at(rnd(mt));
            //qDebug() << __FUNCTION__ << nid;
            if(nid<0) continue;
            std::optional<Node> node = G->get_node(nid);
            if (!node.has_value())
            {
                end = std::chrono::steady_clock::now();
                std::string time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
                qDebug() << "ERROR OBTENIENDO EL NODO";
                break;
            }

            std::string str = std::to_string(agent_id) + "_" + std::to_string(i) + "_" + std::to_string(it);

            auto at = node.value().attrs().find("testattrib");
            if (at == node.value().attrs().end()) {
                Val v; v.str(str);
                Attrib ab; ab.value(v); ab.type(STRING);
                node.value().attrs()["testattrib"] = ab;
                G->add_attrib(node.value(), "pos_x", rnd_float());
                G->add_attrib(node.value(), "pos_y", rnd_float());
            }
            else {
                at->second.value().str(str);
                G->modify_attrib(node.value(), "pos_x", rnd_float());
                G->modify_attrib(node.value(), "pos_y", rnd_float());
            }

            node->agent_id(agent_id);
            bool r = G->update_node(node.value());

            if (!r) {
                std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
                std::string time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
                qDebug() << "ERROR INSERTANDO EL NODO";
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(delay));
    }
}


void CRDT_concurrent_operations::run_test()
{
    try {
        threads.resize(num_threads);

        for (int i; thread &t: threads) {
            t = std::thread(&CRDT_concurrent_operations::concurrent_ops,this, ++i, num_ops, G);
        }
        //create_or_remove_nodes(0, G);
        start = std::chrono::steady_clock::now();


        for (auto &t: threads) if (t.joinable()) t.join();//out = false;


        qDebug() << __FUNCTION__ << "Test finished";

        end = std::chrono::steady_clock::now();
        std::string time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
        std::string result = "CONCURRENT ACCESS: concurrent_operations:"+ MARKER + "OK"+ MARKER + time + MARKER + "Finished properly ";
        qDebug()<< QString::fromStdString(result);

    } catch (std::exception& e) {
        std::cerr << "API TEST: TEST FAILED WITH EXCEPTION "<<  e.what() << " " << std::to_string(__LINE__) + " error file:" + __FILE__ << std::endl;
    }
}

void CRDT_concurrent_operations::save_json_result() {
    //DSR_test::save_json_result();
    CRDT::Utilities u (G.get());
    u.write_to_json_file(output);
}
