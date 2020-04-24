//
// Created by juancarlos on 23/4/20.
//

#include <iostream>
#include <memory>
#include <thread>
#include <pthread.h>
#include <numeric>
#include <vector>

#include "../../graph-related-classes/CRDT.h"

#include <boost/format.hpp>

static int n_threads = 100;
static int n_iters = 100;
static vector<int> time_;

void tests_basicos(const shared_ptr<CRDT::CRDTGraph>& gcrdt) {

    //1. Obtener un nodo con el grafo vacío.

    auto n = gcrdt->get_node("noexiste");

    assert(n.id() == -1);
    std::cout << "Obtener un nodo con el grafo vacío. OK" << std::endl;

    //2. Obtener un edge con el grafo vacío.

    auto e = gcrdt->get_edge("noexiste", "otro");

    assert(e.label() == "error");
    std::cout << "Obtener un edge con el grafo vacío. OK" << std::endl;

    //3. Añadir nodo.

    AttribValue av;
    av.key("imName");
    av.value("existe");
    av.type("string");

    n = Node();
    n.id(1);
    n.agent_id(0);
    n.type("prueba");
    n.name("existe");
    n.attrs(std::map<string, AttribValue> { pair {"prueba", av} });

    bool r = gcrdt->insert_or_assign_node(n);

    assert(r && gcrdt->get_node("existe") == n);
    std::cout << "Añadir un nodo nuevo. OK" << std::endl;


    //4. Añadir un arco.

    av = AttribValue();
    av.key("imName");
    av.value("otroexiste");
    av.type("string");

    n = Node();
    n.id(2);
    n.agent_id(0);
    n.type("prueba");
    n.name("otroexiste");
    n.attrs(std::map<string, AttribValue> { pair {"prueba", av} });

    r = gcrdt->insert_or_assign_node(n);

    EdgeAttribs ea;
    ea.from(1);
    ea.to(2);
    ea.label("RT");

    r = gcrdt->insert_or_assign_edge(ea);

    assert(r && gcrdt->get_edge("existe", "otroexiste") == ea);
    std::cout << "Añadir una nuevo edge. OK" << std::endl;

    //5. Reemplazar un arco.
    ea = EdgeAttribs();
    ea.from(1);
    ea.to(2);
    ea.label("RT2");
    r = gcrdt->insert_or_assign_edge(ea);

    assert(r && gcrdt->get_edge("existe", "otroexiste") == ea && ea.label() == "RT2");
    std::cout << "Reemplazar un edge. OK" << std::endl;

    //6. Borrar un arco.

    r =  gcrdt->delete_edge("existe", "otroexiste");

    assert(r && gcrdt->get_edge("existe", "otroexiste").label() == "error");
    std::cout << " Borrar un arco. OK" << std::endl;

    //7. Borrar nodo que existe.
    r = gcrdt->insert_or_assign_edge(ea);

    r = gcrdt->delete_node("otroexiste");
    assert(r && gcrdt->get_node("otroexiste").id() == -1 && gcrdt->get_node("existe").fano().size() == 0);
    std::cout << " Borrar un nodo. OK" << std::endl;

    //8. Borrar nodo que no existe.
    r = gcrdt->delete_node("otroexiste");
    assert(!r && gcrdt->get_node("otroexiste").id() == -1);
    std::cout << " Borrar un nodo que no existe. OK" << std::endl;


    //9. Borrar un arco que no existe.

    r =  gcrdt->delete_edge("existe", "otroexiste");

    assert(!r && gcrdt->get_edge("existe", "otroexiste").label() == "error");
    std::cout << " Borrar un arco que no existe. OK" << std::endl;


    //10. Reemplazar un nodo.

    n = gcrdt->get_node("existe");
    n.type("otro tipo");

    r =  gcrdt->insert_or_assign_node(n);

    assert(r && gcrdt->get_node("existe") == n);
    std::cout << " Reemplazar un nodo existente. OK" << std::endl;

}

void work_nodes(int id, const shared_ptr<CRDT::CRDTGraph>& gcrdt) {
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    int x = 0;
    while (x < n_iters) {
        start = std::chrono::system_clock::now();

        auto n = gcrdt->get_node(135);
        //cout << boost::str(boost::format("Obtenido nodo: thread%d - %d \n")  % id % x);

        auto val = n.attrs().find("String");
        if (val == n.attrs().end()) return;
        val->second.value(boost::str(boost::format("thread%d - %d ")  % id % x));
        bool res = gcrdt->insert_or_assign_node(n);

        ++x;
        end = std::chrono::system_clock::now();
        int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>
                (end-start).count();
        time_.emplace_back(elapsed_ms);
    }

}


void work_edges(int id, const shared_ptr<CRDT::CRDTGraph>& gcrdt) {

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    int x = 0;
    while (x < n_iters) {
        start = std::chrono::system_clock::now();
        auto edge = gcrdt->get_edge("base", "rgbd");

        //cout << boost::str(boost::format("Obtenido edge: thread%d - %d \n")  % id % x);

        auto val = edge.attrs().find("prueba");

        if (val == edge.attrs().end()) {
            AttribValue av;
            av.key("prueba");
            av.value(boost::str(boost::format("thread%d - %d ") % id % x));
            edge.attrs().emplace(make_pair("prueba",av));
        } else {
            val->second.value(boost::str(boost::format("thread%d - %d ")  % id % x));
        }

        bool res = gcrdt->insert_or_assign_edge(edge);

        ++x;
        end = std::chrono::system_clock::now();
        int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>
                (end-start).count();
        time_.push_back(elapsed_ms);
    }

}



int main() {

    // create graph
    auto gcrdt = std::make_shared<CRDT::CRDTGraph>(0, "prueba", 0); // Init nodes
    tests_basicos(gcrdt);

    //Leer grafo desde un fichero.
    gcrdt->read_from_file("grafo.xml");
    assert(gcrdt->size() == 6);
    std::cout << "Cargar grafo desde un fichero. OK" << std::endl;

    time_.reserve(n_threads*n_iters);

    //int n_threads = 100;

    vector<std::thread> threads;

    for (int i = 0 ; i < n_threads;++i) {
        threads.emplace_back(std::thread(work_nodes, i, gcrdt));
    }


    for (auto &th : threads)
    {
        if (th.joinable())
            th.join();
    }

    std::cout << "Tiempo medio por iteracion (ms): "<< std::accumulate(time_.begin(), time_.end(), 0.0)/time_.size() << std::endl;
    time_.clear();

    threads.clear();


    for (int i = 0 ; i < n_threads;++i) {
        threads.emplace_back(std::thread(work_edges, i, gcrdt));
    }


    for (auto &th : threads)
    {
        if (th.joinable())
            th.join();
    }

    std::cout << "Tiempo medio por iteracion (ms): "<< std::accumulate(time_.begin(), time_.end(), 0.0)/time_.size() << std::endl;

    threads.clear();

    return 0;
}