//
// Created by juancarlos on 7/5/20.
//

#include <iostream>
#include <optional>
#include "CRDT_G_api_test.h"
#include "../../../../graph-related-classes/vertex.h"

void CRDT_G_api_test::get_nonexistent_node(const shared_ptr<CRDT::CRDTGraph>& G) {
    G->reset();
    std::string result;
    start = std::chrono::steady_clock::now();

    std::optional<Node> n_id = G->get_node(666666);
    std::optional<Node> n_name = G->get_node("no existe");
    std::optional<shared_ptr<CRDT::Vertex>> v_id = G->get_vertex(666666);
    std::optional<shared_ptr<CRDT::Vertex>> v_name = G->get_vertex("no existe");

    end = std::chrono::steady_clock::now();
    std::string time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>
                                              (end - start).count());
    if (!n_id.has_value() and !n_name.has_value() and !v_id.has_value()and  !v_name.has_value())
        result = "API TEST: get_nonexistent_node" + MARKER + "OK" + MARKER + time + MARKER + "Finished properly";
    else
        result = "API TEST: get_nonexistent_node" + MARKER + "FAIL" + MARKER + time + MARKER +" error line:" + std::to_string(__LINE__) + " error file:" + __FILE__;

    qDebug()<< QString::fromStdString(result);

}

void CRDT_G_api_test::get_nonexistent_edge(const shared_ptr<CRDT::CRDTGraph>& G)
{
    std::string result;
    start = std::chrono::steady_clock::now();

    std::optional<Edge> n_id = G->get_edge(666666, 77777, "K");
    std::optional<Edge> n_name = G->get_edge("no existe", "otro no existe", "K");

    end = std::chrono::steady_clock::now();
    std::string time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>
                                              (end - start).count());
    if (!n_id.has_value() and !n_name.has_value())
        result = "API TEST: get_nonexistent_edge" + MARKER + "OK" + MARKER + time + MARKER + "Finished properly";
    else
        result = "API TEST: get_nonexistent_edge" + MARKER + "FAIL" + MARKER + time + MARKER +" error line:" + std::to_string(__LINE__) + " error file:" + __FILE__;

    qDebug()<< QString::fromStdString(result);
}

void CRDT_G_api_test::insert_node(const shared_ptr<CRDT::CRDTGraph>& G)
{
    std::string result;
    start = std::chrono::steady_clock::now();

    Node n;
    n.name("test");
    n.id(testutils->newID());
    n.type("testtype");
    bool r  = G->insert_or_assign_node(n);

    end = std::chrono::steady_clock::now();
    std::string time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>
                                              (end - start).count());
    if (r)
        result = "API TEST: insert_node" + MARKER + "OK" + MARKER + time + MARKER + "Finished properly";
    else
        result = "API TEST: insert_node" + MARKER + "FAIL" + MARKER + time + MARKER +" error line:" + std::to_string(__LINE__) + " error file:" + __FILE__;

    qDebug()<< QString::fromStdString(result);
}

void CRDT_G_api_test::get_existent_node(const shared_ptr<CRDT::CRDTGraph>& G)
{
    std::string result;
    start = std::chrono::steady_clock::now();

    std::optional<Node> n_id = G->get_node(testutils->getID());
    std::optional<Node> n_name = G->get_node("test");
    std::optional<shared_ptr<CRDT::Vertex>> v_id = G->get_vertex(testutils->getID());
    std::optional<shared_ptr<CRDT::Vertex>> v_name = G->get_vertex("test");

    end = std::chrono::steady_clock::now();
    std::string time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>
                                              (end - start).count());
    if (n_id.has_value() and n_name.has_value() and v_id.has_value()and  v_name.has_value())
        result = "API TEST: get_existent_node" + MARKER + "OK" + MARKER + time + MARKER + "Finished properly";
    else
        result = "API TEST: get_existent_node" + MARKER + "FAIL" + MARKER + time + MARKER +" error line:" + std::to_string(__LINE__) + " error file:" + __FILE__;

    qDebug()<< QString::fromStdString(result);
}

void CRDT_G_api_test::update_node(const shared_ptr<CRDT::CRDTGraph>& G)
{
    std::string result;
    start = std::chrono::steady_clock::now();

    Node n_id = G->get_node(testutils->getID()).value();
    G->add_attrib(n_id.attrs(), "level", 1);
    bool r = G->insert_or_assign_node(n_id);

    end = std::chrono::steady_clock::now();
    std::string time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>
                                              (end - start).count());
    if (r)
        result = "API TEST: update_node" + MARKER + "OK" + MARKER + time + MARKER + "Finished properly";
    else
        result = "API TEST: update_node" + MARKER + "FAIL" + MARKER + time + MARKER +" error line:" + std::to_string(__LINE__) + " error file:" + __FILE__;

    qDebug()<< QString::fromStdString(result);
}

void CRDT_G_api_test::insert_edge(const shared_ptr<CRDT::CRDTGraph>& G)
{
    std::string result;
    start = std::chrono::steady_clock::now();



    Node n;
    n.name("test2");
    n.id(testutils->newID());
    n.type("testtype");
    G->insert_or_assign_node(n);



    Edge e;
    e.type("testtype");
    e.to(testutils->getID());
    e.from(testutils->getID());

    bool r  = G->insert_or_assign_edge(e);
    testutils->addEdgeIDs(e.from(), e.to());

    end = std::chrono::steady_clock::now();
    std::string time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>
                                              (end - start).count());
    if (r)
        result = "API TEST: insert_edge" + MARKER + "OK" + MARKER + time + MARKER + "Finished properly";
    else
        result = "API TEST: insert_edge" + MARKER + "FAIL" + MARKER + time + MARKER +" error line:" + std::to_string(__LINE__) + " error file:" + __FILE__;

    qDebug()<< QString::fromStdString(result);
}

void CRDT_G_api_test::get_existent_edge(const shared_ptr<CRDT::CRDTGraph>& G)
{

    std::string result;
    start = std::chrono::steady_clock::now();

    auto [from, to] = testutils->getEdgeIDs();
    std::optional<Edge> e_id = G->get_edge(from, to, "testtype");
    std::optional<Edge> e_name = G->get_edge("test", "test2", "testtype");

    end = std::chrono::steady_clock::now();
    std::string time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>
                                              (end - start).count());
    if (e_id.has_value() and e_name.has_value())
        result = "API TEST: get_existent_edge" + MARKER + "OK" + MARKER + time + MARKER + "Finished properly";
    else
        result = "API TEST: get_existent_edge" + MARKER + "FAIL" + MARKER + time + MARKER +" error line:" + std::to_string(__LINE__) + " error file:" + __FILE__;

    qDebug()<< QString::fromStdString(result);
}


void CRDT_G_api_test::update_edge(const shared_ptr<CRDT::CRDTGraph>& G)
{
    std::string result;
    start = std::chrono::steady_clock::now();

    auto [from, to] = testutils->getEdgeIDs();
    Edge e_id = G->get_edge(from, to, "testtype").value();
    G->add_attrib(e_id.attrs(), "att", std::string("a"));
    bool r = G->insert_or_assign_edge(e_id);

    end = std::chrono::steady_clock::now();
    std::string time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>
                                              (end - start).count());
    if (r)
        result = "API TEST: update_edge" + MARKER + "OK" + MARKER + time + MARKER + "Finished properly";
    else
        result = "API TEST: update_edge" + MARKER + "FAIL" + MARKER + time + MARKER +" error line:" + std::to_string(__LINE__) + " error file:" + __FILE__;

    qDebug()<< QString::fromStdString(result);
}

void CRDT_G_api_test::delete_edge(const shared_ptr<CRDT::CRDTGraph>& G)
{
    std::string result;
    start = std::chrono::steady_clock::now();

    auto [from, to] = testutils->removeEdgeIDs();
    bool r = G->delete_edge(from, to, "testtype");


    end = std::chrono::steady_clock::now();
    std::string time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>
                                              (end - start).count());
    if (r)
        result = "API TEST: delete_edge" + MARKER + "OK" + MARKER + time + MARKER + "Finished properly";
    else
        result = "API TEST: delete_edge" + MARKER + "FAIL" + MARKER + time + MARKER +" error line:" + std::to_string(__LINE__) + " error file:" + __FILE__;

    qDebug()<< QString::fromStdString(result);
}

void CRDT_G_api_test::delete_nonexistent_edge(const shared_ptr<CRDT::CRDTGraph>& G)
{
    std::string result;
    start = std::chrono::steady_clock::now();

    bool r = G->delete_edge(888888, 777777, "testtype");


    end = std::chrono::steady_clock::now();
    std::string time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>
                                              (end - start).count());
    if (!r)
        result = "API TEST: delete_nonexistent_edge" + MARKER + "OK" + MARKER + time + MARKER + "Finished properly";
    else
        result = "API TEST: delete_nonexistent_edge" + MARKER + "FAIL" + MARKER + time + MARKER +" error line:" + std::to_string(__LINE__) + " error file:" + __FILE__;

    qDebug()<< QString::fromStdString(result);
}

void CRDT_G_api_test::delete_node(const shared_ptr<CRDT::CRDTGraph>& G)
{
    std::string result;
    start = std::chrono::steady_clock::now();

    bool r = G->delete_node(testutils->removeID());


    end = std::chrono::steady_clock::now();
    std::string time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>
                                              (end - start).count());
    if (r)
        result = "API TEST: delete_node" + MARKER + "OK" + MARKER + time + MARKER + "Finished properly";
    else
        result = "API TEST: delete_node" + MARKER + "FAIL" + MARKER + time + MARKER +" error line:" + std::to_string(__LINE__) + " error file:" + __FILE__;

    qDebug()<< QString::fromStdString(result);
}

void CRDT_G_api_test::delete_nonexistent_node(const shared_ptr<CRDT::CRDTGraph>& G)
{
    std::string result;
    start = std::chrono::steady_clock::now();

    bool r = G->delete_node(9999999);


    end = std::chrono::steady_clock::now();
    std::string time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>
                                              (end - start).count());
    if (!r)
        result = "API TEST: delete_nonexistent_node" + MARKER + "OK" + MARKER + time + MARKER + "Finished properly";
    else
        result = "API TEST: delete_nonexistent_node" + MARKER + "FAIL" + MARKER + time + MARKER +" error line:" + std::to_string(__LINE__) + " error file:" + __FILE__;

    qDebug()<< QString::fromStdString(result);
}

void CRDT_G_api_test::load_empty_file(const shared_ptr<CRDT::CRDTGraph>& G, const std::string& File)
{
    G->reset();
    std::string result;
    start = std::chrono::steady_clock::now();

    G->read_from_json_file(File);

    end = std::chrono::steady_clock::now();
    std::string time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>
                                              (end - start).count());
    if (G->size() == 0)
        result = "API TEST: load_empty_file" + MARKER + "OK" + MARKER + time + MARKER + "Finished properly";
    else
        result = "API TEST: load_empty_file" + MARKER + "FAIL" + MARKER + time + MARKER +" error line:" + std::to_string(__LINE__) + " error file:" + __FILE__;

    qDebug()<< QString::fromStdString(result);
}

void CRDT_G_api_test::load_file(const shared_ptr<CRDT::CRDTGraph>& G, const std::string& File)
{
    G->reset();

    std::string result;
    start = std::chrono::steady_clock::now();

    G->read_from_json_file(File);

    end = std::chrono::steady_clock::now();
    std::string time = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>
                                              (end - start).count());
    if (G->size() == 4)
        result = "API TEST: load_file" + MARKER + "OK" + MARKER + time + MARKER + "Finished properly";
    else
        result = "API TEST: load_file" + MARKER + "FAIL" + MARKER + time + MARKER +" error line:" + std::to_string(__LINE__) + " error file:" + __FILE__;

    qDebug()<< QString::fromStdString(result);
}



void CRDT_G_api_test::test(const shared_ptr<CRDT::CRDTGraph>& G) {
    get_nonexistent_node(G);
    get_nonexistent_edge(G);
    insert_node(G);
    get_existent_node(G);
    update_node(G);
    insert_edge(G);
    get_existent_edge(G);
    update_edge(G);
    delete_edge(G);
    delete_nonexistent_edge(G);
    delete_node(G);
    delete_nonexistent_node(G);
    load_empty_file(G, empty_file);
    load_file(G, test_file);
}