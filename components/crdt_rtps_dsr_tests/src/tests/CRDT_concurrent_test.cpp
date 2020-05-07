//
// Created by juancarlos on 7/5/20.
//

#include <QtCore/qlogging.h>
#include <QtCore/qdebug.h>
#include "CRDT_concurrent_test.h"
#include "../../../../graph-related-classes/topics/DSRGraph.h"
#include <thread>

void CRDT_concurrent_test::create_or_remove_nodes(int i, const shared_ptr<CRDT::CRDTGraph>& G)
{
    static int it=0;
    qDebug() << __FUNCTION__ << "Enter thread" << i;
    while (it++ < num_ops)
    {
        // ramdomly select create or remove
        if( testutils->rnd_selector() == 0)
        {
            //qDebug() << __FUNCTION__ << "Create node";
            // create node
            Node node;
            node.type("plane");
            auto id = testutils->newID();
            node.id( id );
            node.agent_id(0);
            node.name("plane" + std::to_string(id));
            std::map<std::string, Attrib> attrs;
            G->add_attrib(attrs, "name", std::string("fucking_plane"));
            G->add_attrib(attrs, "color", std::string("SteelBlue"));
            auto dis = std::uniform_real_distribution(-200.0, 200.0);
            G->add_attrib(attrs, "pos_x", testutils->rnd_float());
            G->add_attrib(attrs, "pos_y", testutils->rnd_float());

            node.attrs(attrs);

            // insert node
            auto r = G->insert_or_assign_node(node);
            if (r)
                qDebug() << "Created node:" << id << " Total size:" << G->size();
        }
        else
        {
            //qDebug() << __FUNCTION__ << "Remove node";
            int id = testutils->removeID();
            if(id>-1)
            {
                G->delete_node(id);
                qDebug() << "Deleted node:" << id << " Total size:" << G->size();
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(delay));
    }
}

void CRDT_concurrent_test::create_or_remove_edges(int i, const shared_ptr<CRDT::CRDTGraph>& G)
{
    static int it=0;
    qDebug() << __FUNCTION__ << "Enter thread" << i;
    while (it++ < num_ops)
    {
        // ramdomly select create or remove

        if(testutils->rnd_selector() == 0)
        {
            Edge edge;
            edge.type("Edge");
            //get two ids
            edge.from(testutils->getID());
            edge.to(testutils->getID());
            std::map<string, Attrib> attrs;
            G->add_attrib(attrs, "name", std::string("fucking_plane"));
            G->add_attrib(attrs, "color", std::string("SteelBlue"));
            edge.attrs(attrs);

            auto r = G->insert_or_assign_edge(edge);
            if (r) {
                testutils->addEdgeIDs(edge.from(), edge.to());
                qDebug() << "Created edge:" << edge.from() << " - " << edge.to();
            }
        }
        else
        {
            //get two ids
            auto [from, to] = testutils->removeEdgeIDs();
            auto r = G->delete_edge(from, to, "Edge");
            if (r)
                qDebug() << "Deleted edge :"  << from << " - " << to ;

        }
        std::this_thread::sleep_for(std::chrono::milliseconds(delay));
    }
}

void CRDT_concurrent_test::insert_or_assign_attributes(int i, const shared_ptr<CRDT::CRDTGraph>& G)
{
    std::string result;
    static int it = 0;
    qDebug() << __FUNCTION__ << "Enter thread" << i;
    start = std::chrono::steady_clock::now();
    //bool fail=false;
    //auto map = G->getCopy();  // provides a deep copy of the graph. Changes in it won't have effect on G
    auto keys = G->getKeys();
    auto node_randomizer = std::uniform_int_distribution(0, (int)keys.size()-1);




    while (it++ < num_ops)
    {
        // request node
        auto nid = keys.at(testutils->rnd_selector());
        //qDebug() << __FUNCTION__ << nid;
        if(nid<0) continue;
        std::optional<Node> node = G->get_node(nid);
        if (!node.has_value())
        {
            end = std::chrono::steady_clock::now();
            std::string time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
            //result = "test_set_string" + MARKER +"FAIL" + MARKER + time + MARKER +" error getting node->line:" + std::to_string(__LINE__);
            //fail = true;
            qDebug() << "ERROR OBTENIENDO EL NODO";

            break;
        }

        std::string str = 0 + "-" + std::to_string(i) + "_" + std::to_string(it);

        auto at = node.value().attrs().find("testattrib");
        if (at == node.value().attrs().end()) {
            Val v;
            v.str(str);
            Attrib ab;
            ab.value(v);
            ab.type(STRING);
            node.value().attrs()["testattrib"] = ab;
        }
        else {
            at->second.value().str(str);
        }
        bool r = G->insert_or_assign_node(node.value());

        if (!r) {
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            std::string time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
            //result = "test_set_string" + MARKER + "FAIL" + MARKER + time + MARKER + " error inserting node->line:" + std::to_string(__LINE__);
            //fail = true;
            qDebug() << "ERROR INSERTANDO EL NODO";
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(delay));
    }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::string time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());

    /*
    if (fail==false)
        result = "CONCURRENT ACCESS: insert_or_assign_attributes"+ MARKER + "OK"+ MARKER + time + MARKER + "Finished properly";
    //write_test_output(result);
    */
}


void CRDT_concurrent_test::test(const shared_ptr<CRDT::CRDTGraph>& G)
{

    threads.resize(num_threads);
    for(int i=0; auto &t : threads)
        t = std::move(std::thread(&CRDT_concurrent_test::create_or_remove_nodes, this, i++, G));
    qDebug() << __FUNCTION__ << "Threads initiated";

    start = std::chrono::steady_clock::now();
    for(auto &t : threads)
        t.join();
    qDebug() << __FUNCTION__ << "Threads finished";

    end = std::chrono::steady_clock::now();
    std::string time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
    std::string result = "CONCURRENT ACCESS: create_or_remove_nodes:"+ MARKER + "OK"+ MARKER + time + MARKER + "Finished properly, num_threads " +std::to_string(num_threads);
    //write_test_output(result);
    qDebug()<< QString::fromStdString(result);

    threads.resize(num_threads);
    for(int i=0; auto &t : threads)
    t = std::move(std::thread(&CRDT_concurrent_test::create_or_remove_edges, this, i++, G));
    qDebug() << __FUNCTION__ << "Threads initiated";

    start = std::chrono::steady_clock::now();
    for(auto &t : threads)
        t.join();
    qDebug() << __FUNCTION__ << "Threads finished";

    end = std::chrono::steady_clock::now();
    time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
    result = "CONCURRENT ACCESS: create_or_remove_edges:"+ MARKER + "OK"+ MARKER + time + MARKER + "Finished properly, num_threads " +std::to_string(num_threads);
    //write_test_output(result);
    qDebug()<< QString::fromStdString(result);

    threads.resize(num_threads);
    for(int i=0; auto &t : threads)
    t = std::move(std::thread(&CRDT_concurrent_test::insert_or_assign_attributes, this, i++, G));
    qDebug() << __FUNCTION__ << "Threads initiated";

    start = std::chrono::steady_clock::now();
    for(auto &t : threads)
        t.join();
    qDebug() << __FUNCTION__ << "Threads finished";

    end = std::chrono::steady_clock::now();
    time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
    result = "CONCURRENT ACCESS: insert_or_assign_attributes:"+ MARKER + "OK"+ MARKER + time + MARKER + "Finished properly, num_threads " +std::to_string(num_threads);
    qDebug()<< QString::fromStdString(result);
}