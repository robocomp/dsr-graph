//
// Created by juancarlos on 7/5/20.
//

#include <QtCore/qlogging.h>
#include <QtCore/qdebug.h>
#include "CRDT_concurrent_operations.h"
#include "../../../../graph-related-classes/topics/DSRGraph.h"
#include <thread>

void CRDT_concurrent_operations::create_or_remove_nodes(int i, const shared_ptr<CRDT::CRDTGraph>& G)
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
            node.type("plane");
            auto id = newID();
            node.id( id );
            node.agent_id(0);
            node.name("plane" + std::to_string(id));
            G->add_attrib(node.attrs(), "name", std::string("fucking_plane"));
            G->add_attrib(node.attrs(), "color", std::string("SteelBlue"));
            G->add_attrib(node.attrs(), "pos_x", rnd_float());
            G->add_attrib(node.attrs(), "pos_y", rnd_float());

            //node.attrs(attrs);
            qDebug() << "Ready to insert";
            // insert node
            auto r = G->insert_or_assign_node(node);
            if (r)
                qDebug() << "Created node:" << id;// << " Total size:" << G->size();
            else  qDebug() << "Error inserting node";

        }
        else
        {
            //qDebug() << __FUNCTION__ << "Remove node";
            int id = removeID();
            if(id>-1)
            {
                bool r = G->delete_node(id);
                if (r)
                    qDebug() << "Deleted node:" << id;
                else  qDebug() << "Error deleting node";
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(delay));
    }
}


void CRDT_concurrent_operations::run_test()
{
    try {
        threads.resize(num_threads);

        for (int i; thread &t: threads) {
            t = std::thread(&CRDT_concurrent_operations::create_or_remove_nodes,this, i, G);
        }
        //create_or_remove_nodes(0, G);
        start = std::chrono::steady_clock::now();


        for (auto &t: threads) if (t.joinable()) t.join();//out = false;


        qDebug() << __FUNCTION__ << "Test finished";

        end = std::chrono::steady_clock::now();
        std::string time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
        std::string result = "CONCURRENT ACCESS: create_or_remove_nodes:"+ MARKER + "OK"+ MARKER + time + MARKER + "Finished properly ";
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
