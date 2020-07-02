//
// Created by juancarlos on 7/5/20.
//

#include <QtCore/qlogging.h>
#include <QtCore/qdebug.h>
#include "CRDT_insert_remove_edge.h"
#include <thread>
#include <fstream>


void CRDT_insert_remove_edge::create_or_remove_edges(int i, const shared_ptr<CRDT::CRDTGraph>& G)
{
    static int it=0;
    while (it++ < num_ops)
    {
        bool r = false;
        start = std::chrono::steady_clock::now();
        // ramdomly select create or remove
        if(rnd_selector() == 0)
        {
            Edge edge;
            edge.type("Edge");
            //get two ids
            edge.from(getID());
            edge.to(getID());
            G->add_attrib_local(edge, "name", std::string("fucking_plane"));
            G->add_attrib_local(edge, "color", std::string("SteelBlue"));

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
        end = std::chrono::steady_clock::now();
        if (r)
            times.emplace_back(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
        std::this_thread::sleep_for(std::chrono::microseconds(delay));
    }
}


void CRDT_insert_remove_edge::run_test()
{
    try {
        int i = 0;
        while (i++ < 20) {
            Node node;
            node.type("plane");
            auto id = newID();
            node.id( id );
            node.agent_id(0);
            node.name("plane" + std::to_string(id));
            G->add_attrib_local(node, "name", std::string("fucking_plane"));
            G->add_attrib_local(node, "color", std::string("SteelBlue"));
            G->add_attrib_local(node, "pos_x", rnd_float());
            G->add_attrib_local(node, "pos_y", rnd_float());
            G->add_attrib_local(node, "parent", 100);

            // insert node
            auto r = G->insert_node(node);
            if (r)
                qDebug() << "Created node:" << id << " Total size:" << G->size();
        }

        start_global = std::chrono::steady_clock::now();
        create_or_remove_edges(0, G);
        end_global = std::chrono::steady_clock::now();
        double time = std::chrono::duration_cast<std::chrono::milliseconds>(end_global - start_global).count();
        result = "CONCURRENT ACCESS: create_or_remove_edges"+ MARKER + "OK"+ MARKER + std::to_string(time) + MARKER + "Finished properly";
        //write_test_output(result);
        qDebug()<< QString::fromStdString(result);
        mean = static_cast<double>(std::accumulate(times.begin(), times.end(), 0))/num_ops;
        ops_second = num_ops*1000/static_cast<double>(std::accumulate(times.begin(), times.end(), 0));


    } catch (std::exception& e) {
        std::cerr << "API TEST: TEST FAILED WITH EXCEPTION "<<  e.what() << " " << std::to_string(__LINE__) + " error file:" + __FILE__ << std::endl;
    }
}

void CRDT_insert_remove_edge::save_json_result() {
    G->write_to_json_file(output);

    qDebug()<<"write results"<<QString::fromStdString(output_result);
    std::ofstream out;
    out.open(output_result, std::ofstream::out | std::ofstream::trunc);
    out << result << "\n";
    out << "ops/second"<<MARKER<<ops_second<< "\n";
    out << "mean(ms)"<<MARKER<<mean<< "\n";
    out.close();
}
