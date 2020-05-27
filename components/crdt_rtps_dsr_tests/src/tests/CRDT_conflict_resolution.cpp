//
// Created by juancarlos on 7/5/20.
//

#include <QtCore/qlogging.h>
#include <QtCore/qdebug.h>
#include "CRDT_conflict_resolution.h"
#include "../../../../graph-related-classes/topics/DSRGraph.h"
#include <thread>
#include <random>

void CRDT_conflict_resolution::insert_or_assign_attributes(int i, const shared_ptr<CRDT::CRDTGraph>& G)
{
    std::string result;
    static int it = 0;
    qDebug() << __FUNCTION__ << "Enter thread" << i;
    start = std::chrono::steady_clock::now();
    //bool fail=false;
    //auto map = G->getCopy();  // provides a deep copy of the graph. Changes in it won't have effect on G

    //auto keys = G->getKeys();
    //std::uniform_int_distribution<int> rnd = std::uniform_int_distribution(0, static_cast<int>(keys.size()-1));

    while (it++ < num_ops)
    {
        // request node
        std::optional<Node> node = G->get_node(100);
        if (!node.has_value())
        {
            end = std::chrono::steady_clock::now();
            std::string time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
            //result = "test_set_string" + MARKER +"FAIL" + MARKER + time + MARKER +" error getting node->line:" + std::to_string(__LINE__);
            //fail = true;
            qDebug() << "ERROR OBTENIENDO EL NODO";
            break;
        }

        std::string str = std::to_string(agent_id) + "-"+ std::to_string(it);

        auto at = node.value().attrs().find("testattrib");
        if (at == node.value().attrs().end()) {
            Val v;
            v.str(str);
            Attrib ab;
            ab.value(v);
            ab.type(STRING);
            node.value().attrs()["testattrib"] = ab;
            node->agent_id(agent_id);
            G->add_attrib(node.value(), "pos_x", rnd_float());
            G->add_attrib(node.value(), "pos_y", rnd_float());
        }
        else {
            at->second.value().str(str);
            G->modify_attrib(node.value(), "pos_x", rnd_float());
            G->modify_attrib(node.value(), "pos_y", rnd_float());
        }

        bool r = G->update_node(node.value());

        if (!r) {
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            std::string time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
            qDebug() << "ERROR INSERTANDO EL NODO";
            break;
        }
        std::this_thread::sleep_for(std::chrono::microseconds (delay));
    }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::string time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());



}


void CRDT_conflict_resolution::run_test()
{
    try {
        start = std::chrono::steady_clock::now();
        insert_or_assign_attributes(0, G);
        end = std::chrono::steady_clock::now();
        std::string time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
        std::string result = "CONCURRENT ACCESS: conflict_resolution:"+ MARKER + "OK"+ MARKER + time + MARKER + "Finished properly ";
        qDebug()<< QString::fromStdString(result);

    } catch (std::exception& e) {
        std::cerr << "API TEST: TEST FAILED WITH EXCEPTION "<<  e.what() << " " << std::to_string(__LINE__) + " error file:" + __FILE__ << std::endl;
    }
}

void CRDT_conflict_resolution::save_json_result() {
    //DSR_test::save_json_result();
    CRDT::Utilities u (G.get());
    u.write_to_json_file(output);
}
