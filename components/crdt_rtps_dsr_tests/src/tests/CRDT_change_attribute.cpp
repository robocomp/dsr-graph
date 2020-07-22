//
// Created by juancarlos on 7/5/20.
//

#include <QtCore/qlogging.h>
#include <QtCore/qdebug.h>
#include "CRDT_change_attribute.h"
#include <thread>
#include <random>
#include <fstream>

void DSR_change_attribute::insert_or_assign_attributes(int i, const shared_ptr<DSR::DSRGraph>& G)
{
    std::string result;
    static int it = 0;
    qDebug() << __FUNCTION__ << "Enter thread" << i;
    start = std::chrono::steady_clock::now();
    //bool fail=false;
    //auto map = G->getCopy();  // provides a deep copy of the graph. Changes in it won't have effect on G

    auto keys = G->getKeys();
    std::uniform_int_distribution<int> rnd = std::uniform_int_distribution(0, static_cast<int>(keys.size()-1));

    while (it++ < num_ops)
    {
        start = std::chrono::steady_clock::now();
        // request node
        auto nid = keys.at(rnd(mt));
        if(nid<0) continue;
        std::optional<Node> node = G->get_node(nid);
        if (!node.has_value())
        {
            throw std::runtime_error("ERROR OBTENIENDO EL NODO");
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
            node->agent_id(agent_id);
        }
        else {
            at->second.value().str(str);
        }
        bool r = G->update_node(node.value());

        if (!r) {
            throw std::runtime_error("ERROR INSERTANDO EL NODO");
        }
        end = std::chrono::steady_clock::now();
        times.emplace_back(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());

        std::this_thread::sleep_for(std::chrono::microseconds(delay));
    }



}


void DSR_change_attribute::run_test()
{
    try {
        start_global = std::chrono::steady_clock::now();
        insert_or_assign_attributes(0, G);
        end_global = std::chrono::steady_clock::now();
        double time = std::chrono::duration_cast<std::chrono::milliseconds>(end_global - start_global).count();
        result = "CONCURRENT ACCESS: insert_or_assign_attributes"+ MARKER + "OK"+ MARKER + std::to_string(time) + MARKER + "Finished properly ";
        qDebug()<< QString::fromStdString(result);

        mean = static_cast<double>(std::accumulate(times.begin(), times.end(), 0))/num_ops;
        ops_second = num_ops*1000/static_cast<double>(std::accumulate(times.begin(), times.end(), 0));

    } catch (std::exception& e) {
        end_global = std::chrono::steady_clock::now();
        std::string time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(end_global - start_global).count());
        qDebug()<< QString::fromStdString(result);
        result = "CONCURRENT ACCESS: insert_or_assign_attributes"+ MARKER + "ERROR"+ MARKER + time + MARKER + e.what();
        std::cerr << "API TEST: TEST FAILED WITH EXCEPTION "<<  e.what() << " " << std::to_string(__LINE__) + " error file:" + __FILE__ << std::endl;
    }
}

void DSR_change_attribute::save_json_result() {
    G->write_to_json_file(output);

    qDebug()<<"write results"<<QString::fromStdString(output_result);
    std::ofstream out;
    out.open(output_result, std::ofstream::out | std::ofstream::trunc);
    out << result << "\n";
    out << "ops/second"<<MARKER<<ops_second<< "\n";
    out << "mean(ms)"<<MARKER<<mean<< "\n";
    out.close();

}
