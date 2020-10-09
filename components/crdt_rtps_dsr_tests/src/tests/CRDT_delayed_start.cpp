//
// Created by juancarlos on 7/5/20.
//

#include <QtCore/qlogging.h>
#include <QtCore/qdebug.h>
#include "CRDT_delayed_start.h"
#include <thread>
#include <fstream>
#include <type_traits>
REGISTER_TYPE(testattrib, std::reference_wrapper<const string>, false)

void CRDT_delayed_start::create_or_remove_nodes(int i, const shared_ptr<DSR::DSRGraph>& G)
{
    static int it=0;
    while (it++ < num_ops)
    {
        bool r = false;
        start = std::chrono::steady_clock::now();
        // ramdomly select create or remove
        if( rnd_selector() == 0)
        {
            //qDebug() << __FUNCTION__ << "Create node";
            // create node
            DSR::Node node;
            node.type("plane");
            //auto id = newID();
            //node.id( id );
            node.agent_id(agent_id);
            //node.name("plane" + std::to_string(id));
            G->add_attrib_local<name_att>(node, std::string("fucking_plane"));
            G->add_attrib_local<color_att>(node, std::string("SteelBlue"));
            G->add_attrib_local<pos_x_att>(node,  rnd_float());
            G->add_attrib_local<pos_y_att>(node,  rnd_float());
            G->add_attrib_local<parent_att>(node,  100u);

            //node.attrs(attrs);

            // insert node
            auto res = G->insert_node(node);
            if (res.has_value()) {
                created_nodes.push_back(res.value());
                qDebug() << "Created node:" << res.value() << " Total size:" << G->size();
                r = true;
            }
        }
        else
        {
            //qDebug() << __FUNCTION__ << "Remove node";
            int id = removeID();
            if(id>-1)
            {
                r = G->delete_node(id);
                if (r)
                    qDebug() << "Deleted node:" << id << " Total size:" << G->size();
            }
        }
        end = std::chrono::steady_clock::now();
        if (r)
            times.emplace_back(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
        std::this_thread::sleep_for(100ms);
    }
}


void CRDT_delayed_start::run_test()
{
    try {
        start_global = std::chrono::steady_clock::now();
        create_or_remove_nodes(0, G);
        end_global = std::chrono::steady_clock::now();
        double time = std::chrono::duration_cast<std::chrono::milliseconds>(end_global - start_global).count();
        result = "CONCURRENT ACCESS: delayed_start"+ MARKER + "OK"+ MARKER + std::to_string(time) + MARKER + "Finished properly ";
        qDebug()<< QString::fromStdString(result);
        mean = static_cast<double>(std::accumulate(times.begin(), times.end(), 0))/num_ops;
        ops_second = num_ops*1000/static_cast<double>(std::accumulate(times.begin(), times.end(), 0));
    } catch (std::exception& e) {
        std::cerr << "API TEST: TEST FAILED WITH EXCEPTION "<<  e.what() << " " << std::to_string(__LINE__) + " error file:" + __FILE__ << std::endl;
    }
}

void CRDT_delayed_start::save_json_result() {
    G->write_to_json_file(output);

    qDebug()<<"write results"<<QString::fromStdString(output_result);
    std::ofstream out;
    out.open(output_result, std::ofstream::out | std::ofstream::trunc);
    out << result << "\n";
    out << "ops/second"<<MARKER<<ops_second<< "\n";
    out << "mean(ms)"<<MARKER<<mean<< "\n";
    out.close();
}
