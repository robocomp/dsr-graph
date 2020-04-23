//
// Created by juancarlos on 23/4/20.
//

#include <iostream>
#include <memory>
#include <thread>
#include "../../graph-related-classes/CRDT.h"

void tests_basicos(shared_ptr<CRDT::CRDTGraph> gcrdt) {

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
    n.attrs(vector<AttribValue> { av });

    bool r = gcrdt->insert_or_assign_node(n);

    assert(r == true && gcrdt->get_node("existe") == n);
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
    n.attrs(vector<AttribValue> { av });

    r = gcrdt->insert_or_assign_node(n);

    EdgeAttribs ea;
    ea.from(1);
    ea.to(2);
    ea.label("RT");

    r = gcrdt->insert_or_assign_edge(ea);

    assert(r == true && gcrdt->get_edge("existe", "otroexiste") == ea);
    std::cout << "Añadir una nuevo edge. OK" << std::endl;

    //5. Reemplazar un arco.
    ea = EdgeAttribs();
    ea.from(1);
    ea.to(2);
    ea.label("RT2");
    r = gcrdt->insert_or_assign_edge(ea);

    assert(r == true && gcrdt->get_edge("existe", "otroexiste") == ea && ea.label() == "RT2");
    std::cout << "Reemplazar un edge. OK" << std::endl;

    //6. Borrar un arco.

    r =  gcrdt->delete_edge("existe", "otroexiste");

    assert(r == true && gcrdt->get_edge("existe", "otroexiste").label() == "error");
    std::cout << " Borrar un arco. OK" << std::endl;

    //7. Borrar nodo que existe.
    r = gcrdt->insert_or_assign_edge(ea);

    r = gcrdt->delete_node("otroexiste");
    assert(r == true && gcrdt->get_node("otroexiste").id() == -1 && gcrdt->get_node("existe").fano().size() == 0);
    std::cout << " Borrar un nodo. OK" << std::endl;

    //8. Borrar nodo que no existe.
    r = gcrdt->delete_node("otroexiste");
    assert(r == false && gcrdt->get_node("otroexiste").id() == -1);
    std::cout << " Borrar un nodo que no existe. OK" << std::endl;


    //9. Borrar un arco que no existe.

    r =  gcrdt->delete_edge("existe", "otroexiste");

    assert(r == false && gcrdt->get_edge("existe", "otroexiste").label() == "error");
    std::cout << " Borrar un arco que no existe. OK" << std::endl;


    //10. Reemplazar un nodo.

    n = gcrdt->get_node("existe");
    n.type("otro tipo");

    r =  gcrdt->insert_or_assign_node(n);

    assert(r == true && gcrdt->get_node("existe") == n);
    std::cout << " Reemplazar un nodo existente. OK" << std::endl;

}

void work(int id) {
    auto gcrdt = std::make_shared<CRDT::CRDTGraph>(0, "prueba", id); // Init nodes

    while (1) {

    }

}

int main() {

    // create graph
    auto gcrdt = std::make_shared<CRDT::CRDTGraph>(0, "prueba", 0); // Init nodes
    tests_basicos(gcrdt);


    /*
    int n_threads = 10;
    vector<std::thread> threads;

    for (int i = 0 ; i < n_threads;++i) {
        threads.push_back(std::thread(work, i));
    }

    for (auto &th : threads)
    {
        if (th.joinable())
            th.join();
    }
    */
    return 0;
}