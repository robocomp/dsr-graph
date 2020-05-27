//
// Created by juancarlos on 7/5/20.
//

#include <QtCore/qlogging.h>
#include <QtCore/qdebug.h>
#include "CRDT_insert_remove_node.h"
#include "../../../../graph-related-classes/topics/DSRGraph.h"
#include <thread>

void CRDT_insert_remove_node::create_or_remove_nodes(int i, const shared_ptr<CRDT::CRDTGraph>& G)
{
    static int it=0;
    qDebug() << __FUNCTION__ << "Enter thread" << i;
    while (it++ < num_ops)
    {
        // ramdomly select create or remove
        if( rnd_selector() == 0)
        {
            //qDebug() << __FUNCTION__ << "Create node";
            // create node
            Node node;
            node.type("n");
            auto id = newID();
            node.id( id );
            node.agent_id(0);
            node.name("plane" + std::to_string(id));
            G->add_attrib(node, "name", std::string("fucking_plane"));
            G->add_attrib(node, "color", std::string("SteelBlue"));
            G->add_attrib(node, "pos_x", rnd_float());
            G->add_attrib(node, "pos_y", rnd_float());
            G->add_attrib(node, "parent", 100);

            //node.attrs(attrs);

            // insert node
            auto r = G->insert_node(node);
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
        std::this_thread::sleep_for(std::chrono::microseconds(delay));
    }
}


void CRDT_insert_remove_node::run_test()
{
    try {

        create_or_remove_nodes(0, G);
        start = std::chrono::steady_clock::now();

        qDebug() << __FUNCTION__ << "Test finished";

        end = std::chrono::steady_clock::now();
        std::string time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
        std::string result = "CONCURRENT ACCESS: create_or_remove_nodes:"+ MARKER + "OK"+ MARKER + time + MARKER + "Finished properly ";
        qDebug()<< QString::fromStdString(result);

    } catch (std::exception& e) {
        std::cerr << "API TEST: TEST FAILED WITH EXCEPTION "<<  e.what() << " " << std::to_string(__LINE__) + " error file:" + __FILE__ << std::endl;
    }
}

void CRDT_insert_remove_node::save_json_result() {
    //DSR_test::save_json_result();
    CRDT::Utilities u (G.get());
    u.write_to_json_file(output);
}
