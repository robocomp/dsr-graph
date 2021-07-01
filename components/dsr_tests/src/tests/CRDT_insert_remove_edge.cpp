//
// Created by juancarlos on 7/5/20.
//

#include <QtCore/qlogging.h>
#include <QtCore/qdebug.h>
#include "CRDT_insert_remove_edge.h"
#include <thread>
#include <fstream>

#include <type_traits>
REGISTER_TYPE(testattrib, std::reference_wrapper<const std::string>, false)

void CRDT_insert_remove_edge::create_or_remove_edges(const std::shared_ptr<DSR::DSRGraph>& G)
{
    static int it=0;
    while (it++ < num_ops)
    {
        bool r = false;
        start = std::chrono::steady_clock::now();
        // ramdomly select create or remove
        if(rnd_selector() == 0)
        {
            DSR::Edge edge;
            edge.type("testtype_e");
            //get two ids
            edge.from(getID());
            edge.to(getID());

            G->add_attrib_local<name_att>(edge,  std::string("fucking_plane"));
            G->add_attrib_local<color_att>(edge,  std::string("SteelBlue"));

            r = G->insert_or_assign_edge(edge);
            if (r) {
                addEdgeIDs(edge.from(), edge.to());
                qDebug() << "Created edge:" << edge.from() << " - " << edge.to();
                operations.emplace_back(Operation{0, get_unix_timestamp(), "INSERT_EDGE;(" + std::to_string(edge.from()) + "," + std::to_string(edge.to()) + ")", true});
            } else {
                operations.emplace_back(Operation{0, get_unix_timestamp(), "INSERT_EDGE;FAIL(" + std::to_string(edge.from()) + "," + std::to_string(edge.to()) + ")", true});
            }
        }
        else
        {
            //get two ids
            auto [from, to] = removeEdgeIDs();
            r = G->delete_edge(from, to, "testtype_e");
            if (r) {
                //operations.emplace_back(Operation{0, get_unix_timestamp(), "DELETE_EDGE;(" + std::to_string(from) + "," + std::to_string(to) + ")", true});
                qDebug() << "Deleted edge :" << from << " - " << to;
            } else {
                //operations.emplace_back(Operation{0, get_unix_timestamp(), "DELETE_EDGE;FAIL(" + std::to_string(from) + "," + std::to_string(to) + ")", true});
            }
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
            DSR::Node node;
            node.type("testtype");
            node.agent_id(0);
            G->add_attrib_local<name_att>(node, std::string("fucking_plane"));
            G->add_attrib_local<color_att>(node, std::string("SteelBlue"));
            G->add_attrib_local<pos_x_att>(node,  rnd_float());
            G->add_attrib_local<pos_y_att>(node,  rnd_float());
            G->add_attrib_local<parent_att>(node,  static_cast<uint64_t>(100));

            // insert node
            auto id = G->insert_node(node);
            if (id.has_value()) {
                qDebug() << "Created node:" << id.value() << " Total size:" << G->size();
                created_nodes.push_back(id.value());
                operations.emplace_back(Operation{0, get_unix_timestamp(), "INSERT_NODE;" + std::to_string(id.value()), true});
            }
        }

        start_global = std::chrono::steady_clock::now();
        create_or_remove_edges(G);
        end_global = std::chrono::steady_clock::now();
        uint64_t time = std::chrono::duration_cast<std::chrono::milliseconds>(end_global - start_global).count();
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
