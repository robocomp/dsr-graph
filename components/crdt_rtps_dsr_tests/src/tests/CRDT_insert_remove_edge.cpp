//
// Created by juancarlos on 7/5/20.
//

#include <QtCore/qlogging.h>
#include <QtCore/qdebug.h>
#include "CRDT_insert_remove_edge.h"
#include "../../../../graph-related-classes/topics/DSRGraph.h"
#include <thread>

void CRDT_insert_remove_edge::create_or_remove_edges(int i, const shared_ptr<CRDT::CRDTGraph>& G)
{
    static int it=0;
    qDebug() << __FUNCTION__ << "Enter thread" << i;
    while (it++ < num_ops)
    {
        // ramdomly select create or remove

        if(rnd_selector() == 0)
        {
            Edge edge;
            edge.type("Edge");
            //get two ids
            edge.from(getID());
            edge.to(getID());
            G->add_attrib(edge, "name", std::string("fucking_plane"));
            G->add_attrib(edge, "color", std::string("SteelBlue"));

            auto r = G->insert_or_assign_edge(edge);
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
            G->add_attrib(node, "name", std::string("fucking_plane"));
            G->add_attrib(node, "color", std::string("SteelBlue"));
            G->add_attrib(node, "pos_x", rnd_float());
            G->add_attrib(node, "pos_y", rnd_float());


            // insert node
            auto r = G->insert_node(node);
            if (r)
                qDebug() << "Created node:" << id << " Total size:" << G->size();
        }

        start = std::chrono::steady_clock::now();
        create_or_remove_edges(0, G);
        end = std::chrono::steady_clock::now();
        std::string time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
        std::string result = "CONCURRENT ACCESS: create_or_remove_edges:"+ MARKER + "OK"+ MARKER + time + MARKER + "Finished properly, num_threads ";
        //write_test_output(result);
        qDebug()<< QString::fromStdString(result);



    } catch (std::exception& e) {
        std::cerr << "API TEST: TEST FAILED WITH EXCEPTION "<<  e.what() << " " << std::to_string(__LINE__) + " error file:" + __FILE__ << std::endl;
    }
}

void CRDT_insert_remove_edge::save_json_result() {
    //DSR_test::save_json_result();
    CRDT::Utilities u (G.get());
    u.write_to_json_file(output);
}
