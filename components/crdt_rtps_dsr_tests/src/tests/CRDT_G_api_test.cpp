//
// Created by juancarlos on 7/5/20.
//

#include <iostream>
#include <optional>
#include "CRDT_G_api_test.h"
#include "../../../../graph-related-classes/vertex.h"

#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include "catch.hpp"
#include <filesystem>


class Graph {
public:
    static Graph& get() {
        static Graph instance;
        return instance;
    }

    shared_ptr<CRDT::CRDTGraph> get_G() { return G;}
private:
    Graph () {
        G = make_shared<CRDT::CRDTGraph>(0, "test", 54000, "/home/robocomp/robocomp/components/dsr-graph/components/crdt_rtps_dsr_tests/src/tests/testfiles/empty_file.json");
    }

    std::shared_ptr<CRDT::CRDTGraph> G;
};

TEST_CASE("Node operations", "[NODE]") {

    std::shared_ptr<CRDT::CRDTGraph> G = Graph::get().get_G();

    SECTION("Get a node that does not exists") {
        G->reset();

        std::optional<Node> n_id = G->get_node(666666);
        std::optional<Node> n_name = G->get_node("no existe");

        REQUIRE(n_id.has_value() == false);
        REQUIRE(n_name.has_value() == false);

    }

    SECTION("Insert a new node") {

        Node n;
        n.name("test");
        n.id(75000);
        n.type("testtype");
        bool r  = G->insert_or_assign_node(n);

        REQUIRE(r == true);

    }
    SECTION("Get an existing node") {

        std::optional<Node> n_id = G->get_node(75000);
        std::optional<Node> n_name = G->get_node("test");

        REQUIRE(n_id.has_value() == true);
        REQUIRE(n_name.has_value() == true);

    }
    SECTION("Update existing node") {

        std::optional<Node> n_id = G->get_node(75000);
        REQUIRE(n_id.has_value() == true);

        G->add_attrib(n_id->attrs(), "level", 1);
        bool r = G->insert_or_assign_node(n_id.value());

        REQUIRE(r == true);

    }
    SECTION("Delete existing node") {

        bool r = G->delete_node(75000);
        REQUIRE(r == true);
        REQUIRE(G->get_node(75000) == std::nullopt);

    }
    SECTION("Delete a node that does not exists") {

        bool r = G->delete_node(75000);
        REQUIRE(r == false);

    }
}


TEST_CASE("Edge operations", "[EDGE]") {

    std::shared_ptr<CRDT::CRDTGraph> G = Graph::get().get_G();

    SECTION("Get an edge that does not exists") {

        std::optional<Edge> e_id = G->get_edge(666666, 77777, "K");
        std::optional<Edge> e_name = G->get_edge("no existe", "otro no existe", "K");

        REQUIRE(e_id.has_value() == false);
        REQUIRE(e_name.has_value() == false);
    }
    SECTION("Insert a new edge") {


        Node n;
        n.name("test");
        n.id(85000);
        n.type("testtype");
        REQUIRE(G->insert_or_assign_node(n) == true);


        n = Node();
        n.name("test2");
        n.id(87000);
        n.type("testtype");
        REQUIRE(G->insert_or_assign_node(n) == true);

        Edge e;
        e.type("testtype");
        e.to(87000);
        e.from(85000);

        bool r  = G->insert_or_assign_edge(e);
        REQUIRE(r == true);

    }
    SECTION("Get an existing edge") {

        std::optional<Edge> e = G->get_edge(85000, 87000, "testtype");
        std::optional<Edge> e_name = G->get_edge("test", "test2", "testtype");
        REQUIRE(e.has_value() == true);
        REQUIRE(e_name.has_value() == true);

    }

    SECTION("Update existing edge") {
        std::optional<Edge> e = G->get_edge(85000, 87000, "testtype");
        REQUIRE(e.has_value() == true);
        G->add_attrib(e->attrs(), "att", std::string("a"));
        bool r = G->insert_or_assign_edge(e.value());
        REQUIRE(r == true);

    }
    SECTION("Delete existing edge") {

        bool r = G->delete_edge(85000, 87000, "testtype");
        REQUIRE(r == true);
        REQUIRE(G->get_edge(85000, 87000, "testtype") == std::nullopt);

    }
    SECTION("Delete an edge that does not exists") {
        bool r = G->delete_edge(85000, 87000, "testtype");
        REQUIRE(r == false);
    }
}

TEST_CASE("File operations", "[FILE]") {

    std::shared_ptr<CRDT::CRDTGraph> G = Graph::get().get_G();
    CRDT::Utilities u (G.get());

    const std::string empty_file = "/home/robocomp/robocomp/components/dsr-graph/components/crdt_rtps_dsr_tests/src/tests/testfiles/empty_file.json";
    const std::string wempty_file = "/home/robocomp/robocomp/components/dsr-graph/components/crdt_rtps_dsr_tests/src/tests/testfiles/write_empty_file.json";

    const std::string test_file = "/home/robocomp/robocomp/components/dsr-graph/components/crdt_rtps_dsr_tests/src/tests/testfiles/initial_dsr2.json";
    const std::string wtest_file = "/home/robocomp/robocomp/components/dsr-graph/components/crdt_rtps_dsr_tests/src/tests/testfiles/write_initial_dsr2.json";



    SECTION("Load an empty file") {
        G->reset();
        REQUIRE(G->size() == 0);

        u.read_from_json_file(empty_file);
        REQUIRE(G->size() == 0);

    }

    SECTION("Write an empty file") {
        std::filesystem::remove(wempty_file);
        REQUIRE(std::filesystem::exists(wempty_file) == false);
        u.write_to_json_file(wempty_file);
        REQUIRE(std::filesystem::exists(wempty_file) == true);
    }

    SECTION("Load a non-empty file") {
        G->reset();
        REQUIRE(G->size() == 0);

        u.read_from_json_file(test_file);
        REQUIRE(G->size() == 4);
    }

    SECTION("Write a file") {
        std::filesystem::remove(wtest_file);
        REQUIRE(std::filesystem::exists(wtest_file) == false);
        u.write_to_json_file(wtest_file);
        REQUIRE(std::filesystem::exists(wtest_file) == true);
        REQUIRE(std::filesystem::exists(wtest_file) == true);

    }
}


TEST_CASE("Join Operations", "[JOIN]") {

    std::shared_ptr<CRDT::CRDTGraph> G = Graph::get().get_G();


    SECTION("Join full graph") {}
    SECTION("Join empty full graph") {}
    SECTION("Join delta add") {}
    SECTION("Join delta remove") {}
    SECTION("Join delta update") {}
}

