//
// Created by juancarlos on 7/5/20.
//

#include <iostream>
#include <optional>
#include <filesystem>

#include "../../../../graph-related-classes/topics/DSRGraph.h"
#include "../../../../graph-related-classes/CRDT.h"

#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include "catch.hpp"
#include <iostream>


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

    SECTION("Get a node that does not exists by id") {
        G->reset();

        std::optional<Node> n_id = G->get_node(666666);

        REQUIRE(n_id.has_value() == false);

    }
    SECTION("Get a node that does not exists by name") {
        G->reset();

        std::optional<Node> n_name = G->get_node("no existe");

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

    SECTION("Get an existing node by id") {

        std::optional<Node> n_id = G->get_node(75000);
        REQUIRE(n_id.has_value() == true);

    }

    SECTION("Get an existing node by name") {

        std::optional<Node> n_name = G->get_node("test");
        REQUIRE(n_name.has_value() == true);
    }

    SECTION("Update existing node") {

        std::optional<Node> n_id = G->get_node(75000);
        REQUIRE(n_id.has_value() == true);

        G->add_attrib(n_id->attrs(), "level", 1);
        bool r = G->insert_or_assign_node(n_id.value());

        REQUIRE(r == true);

    }
    SECTION("Delete existing node by id") {

        bool r = G->delete_node(75000);
        REQUIRE(r == true);
        REQUIRE(G->get_node(75000) == std::nullopt);

    }


    SECTION("Delete existing node by name") {

        Node n;
        n.name("test1");
        n.id(75001);
        n.type("testtype");
        bool r  = G->insert_or_assign_node(n);
        REQUIRE(r == true);
        r = G->delete_node("test1");
        REQUIRE(r == true);
        REQUIRE(G->get_node("test1") == std::nullopt);

    }


    SECTION("Delete a node that does not exists by id") {

        bool r = G->delete_node(75000);
        REQUIRE(r == false);

    }


}


TEST_CASE("Edge operations", "[EDGE]") {

    std::shared_ptr<CRDT::CRDTGraph> G = Graph::get().get_G();

    SECTION("Get an edge that does not exists by id") {

        std::optional<Edge> e_id = G->get_edge(666666, 77777, "K");

        REQUIRE(e_id.has_value() == false);
    }

    SECTION("Get an edge that does not exists by name") {

        std::optional<Edge> e_name = G->get_edge("no existe", "otro no existe", "K");

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
    SECTION("Get an existing edge by id") {

        std::optional<Edge> e = G->get_edge(85000, 87000, "testtype");
        REQUIRE(e.has_value() == true);

    }

    SECTION("Get an existing edge by name") {

        std::optional<Edge> e_name = G->get_edge("test", "test2", "testtype");
        REQUIRE(e_name.has_value() == true);

    }

    SECTION("Update existing edge") {
        std::optional<Edge> e = G->get_edge(85000, 87000, "testtype");
        REQUIRE(e.has_value() == true);
        G->add_attrib(e->attrs(), "att", std::string("a"));
        bool r = G->insert_or_assign_edge(e.value());
        REQUIRE(r == true);

    }
    SECTION("Delete existing edge by id") {

        bool r = G->delete_edge(85000, 87000, "testtype");
        REQUIRE(r == true);
        REQUIRE(G->get_edge(85000, 87000, "testtype") == std::nullopt);

    }

    SECTION("Delete existing edge by name") {
        Edge e;
        e.type("testtype");
        e.to(87000);
        e.from(85000);

        bool r  = G->insert_or_assign_edge(e);
        REQUIRE(r == true);
        r = G->delete_edge("test", "test2", "testtype");
        REQUIRE(r == true);
        REQUIRE(G->get_edge("test", "test2", "testtype") == std::nullopt);

    }

    SECTION("Delete an edge that does not exists") {
        bool r = G->delete_edge(85000, 87000, "testtype");
        REQUIRE(r == false);
    }

    SECTION("Get RTMat from edge") {}

    SECTION("Get edge RT") {}



    SECTION("Insert or assign edge RT") {}

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
        const std::string content = "{\"DSRModel\":{\"symbol\":{}}}";
        std::ifstream file(wempty_file);
        REQUIRE(std::string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>()) == content );
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
        std::ifstream file(wtest_file);
        std::ifstream file2(test_file);

        QString content = QString::fromStdString(std::string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>()));
        QString content2 =  QString::fromStdString(std::string((std::istreambuf_iterator<char>(file2)), std::istreambuf_iterator<char>()));

        REQUIRE( QJsonDocument::fromJson(content.toUtf8()) ==  QJsonDocument::fromJson(content2.toUtf8()));


    }
}


TEST_CASE("Maps operations", "[UTILS]") {
    std::shared_ptr<CRDT::CRDTGraph> G = Graph::get().get_G();

    SECTION("Get id from a name") {
        Node n;
        n.name("name_node"); n.id(12345); n.type("t");
        bool r = G->insert_or_assign_node(n);
        REQUIRE(r == true);

        std::optional<int> id = G->get_id_from_name("name_node");
        REQUIRE(id.has_value() == true);
        REQUIRE(id.value() == 12345);

    }


    SECTION("Get id from a name that does not exist") {
        std::optional<int> id = G->get_id_from_name("name_no");
        REQUIRE(id.has_value() == false);
    }

    SECTION("Get name from an id") {
        Node n;
        n.name("name_node"); n.id(12345); n.type("t");
        bool r = G->insert_or_assign_node(n);
        REQUIRE(r == true);

        std::optional<std::string> name = G->get_name_from_id(12345);
        REQUIRE(name.has_value() == true);
        REQUIRE(name.value() == "name_node");
    }

    SECTION("Get name from an id that does not exist") {
        std::optional<std::string> name = G->get_name_from_id(501245);
        REQUIRE(name.has_value() == false);
    }

    SECTION("Get nodes by type") {
        Node n;
        n.name("name_node2"); n.id(12346); n.type("t");
        bool r = G->insert_or_assign_node(n);
        REQUIRE(r == true);

        vector<Node> ve = G->get_nodes_by_type("t");
        REQUIRE(ve.size() == 2);
        ve = G->get_nodes_by_type("no");
        REQUIRE(ve.size() == 0);

    }
    SECTION("Get edges by type") {
        vector<Edge> ve = G->get_edges_by_type("RT");
        REQUIRE(ve.size() == 3);
        ve = G->get_edges_by_type("no");
        REQUIRE(ve.size() == 0);
    }
    SECTION("Get edges to a node (id)") {
        vector<Edge> ve = G->get_edges_to_id(100);
        REQUIRE(ve.size() == 0);
        ve = G->get_edges_to_id(135);
        REQUIRE(ve.size() == 1);
    }

    SECTION("Get edges from a node") {
        std::optional<std::map<EdgeKey, Edge>> ve = G->get_edges(100);
        REQUIRE(ve.has_value() == true);
        REQUIRE(ve->size() == 2);
        std::optional<Node> n = G->get_node(100);
        REQUIRE(n.has_value() == true);
        REQUIRE(n->fano() == ve.value());
    }

    SECTION("Get edges from a node that does not exist") {
        std::optional<std::map<EdgeKey, Edge>> ve = G->get_edges(45550);
        REQUIRE(ve.has_value() == false);
    }

    SECTION("Get edges from a node that has no edges") {
        std::optional<std::map<EdgeKey, Edge>> ve = G->get_edges(135);
        REQUIRE(ve.has_value() == true);
        REQUIRE(ve->size() == 0);
    }
}

TEST_CASE("Attributes operations", "[ATTRIBUTES]") {
    std::shared_ptr<CRDT::CRDTGraph> G = Graph::get().get_G();

    SECTION("Insert attribute by name (node)") {
        std::optional<Node> n = G->get_node(100);
        REQUIRE(n.has_value());
        G->insert_or_assign_attrib_by_name(n.value(), "att", 123);
        REQUIRE(n->attrs().find("att") != n->attrs().end());
        bool r = G->insert_or_assign_node(n.value());
        REQUIRE(r == true);
    }

    SECTION("Insert attribute by name (edge)") {
        std::optional<Edge> e = G->get_edge(100, 135, "RT");
        REQUIRE(e.has_value());
        G->insert_or_assign_attrib_by_name(e.value(), "att", 123);
        REQUIRE(e->attrs().find("att") != e->attrs().end());
        bool r = G->insert_or_assign_edge(e.value());
        REQUIRE(r == true);
    }
    SECTION("Update attribute by name") {
        std::optional<Node> n = G->get_node(100);
        REQUIRE(n.has_value());
        G->insert_or_assign_attrib_by_name(n.value(), "att", 125);
        REQUIRE(n->attrs()["att"].value().dec() == 125);
        bool r = G->insert_or_assign_node(n.value());
        REQUIRE(r == true);
    }

    SECTION("Get attribute by name") {
        std::optional<Node> n = G->get_node(100);
        REQUIRE(n.has_value());
        std::optional<int> att = G->get_attrib_by_name<int>(n.value(), "att");
        REQUIRE(att.has_value() == true);
        REQUIRE(att.value() == 125);

    }

    //TODO: Crear delete attribute by name y crear su test.
    SECTION("Get node level") {
        std::optional<Node> n = G->get_node(100);
        REQUIRE(n.has_value());
        std::optional<int> level = G->get_node_level(n.value());
        REQUIRE(level.has_value() == true);
        REQUIRE(level.value() == 0);

        n = G->get_node(12345);
        REQUIRE(n.has_value());
        level = G->get_node_level(n.value());
        REQUIRE(level.has_value() == false);
    }
    SECTION("Get node type") {
        std::optional<Node> n = G->get_node(100);
        REQUIRE(n.has_value());
        std::optional<std::string> type = G->get_node_type(n.value());
        REQUIRE(type.has_value() == true);
        REQUIRE(type.value() == "world");

    }
}


//Scenarios
SCENARIO( "Node insertions, updates and removals", "[NODE]" ) {
    std::shared_ptr<CRDT::CRDTGraph> G = Graph::get().get_G();

    GIVEN("A new Node") {
        Node n;
        Val v;
        n.id(2222);
        n.type("robot");
        n.agent_id(0);
        n.name("robot1");
        G->add_attrib(n.attrs(), "att", "value");

        WHEN("The node is inserted") {
            size_t size = G->size();
            bool r = G->insert_or_assign_node(n);
            REQUIRE(r == true);

            THEN("The graph size is bigger") {
                REQUIRE(size < G->size());
            }THEN("You can get the node") {
                std::optional<Node> node = G->get_node(2222);
                REQUIRE(node.has_value());
                THEN("The requested node is equal to the inserted node") {
                    REQUIRE(node.value() == n);
                }
            }
        }

        AND_WHEN("The node is updated") {
            size_t size = G->size();
            G->add_attrib(n.attrs(), "new att", 11);
            bool r = G->insert_or_assign_node(n);
            REQUIRE(r == true);

            THEN("The graph size is equal") {
                REQUIRE(size == G->size());
            }THEN("You can get the node") {
                std::optional<Node> node = G->get_node(2222);
                REQUIRE(node.has_value());
                THEN("The requested node is equal to the inserted node") {
                    REQUIRE(node.value() == n);
                }
            }
        }

        AND_WHEN("The node is deleted") {
            size_t size = G->size();
            bool r = G->delete_node(2222);
            THEN("The graph size is smaller") {
                REQUIRE(size > G->size());
            }AND_THEN("You can't get the node") {
                std::optional<Node> node = G->get_node(2222);
                REQUIRE(!node.has_value());
            }AND_THEN("You can't insert the node again") {
                bool r = G->insert_or_assign_node(n);
                REQUIRE(r == false);
            }
        }
    }GIVEN("A deleted node") {
        //75000
        std::optional<Node> node = G->get_node(2222);
        THEN("Optional value is empty") {
            REQUIRE(!node.has_value());
        }AND_THEN("You can't insert the node again") {
            Node n;
            n.id(2222);
            bool r = G->insert_or_assign_node(n);
            REQUIRE(r == false);
        }
    }GIVEN("An invalid node") {
        Node n; n.id(-1);
        THEN("Can't insert it") {
            bool r = G->insert_or_assign_node(n);
            REQUIRE(r == false);
        }
    }
}
/*
TEST_CASE("Join operations", "[JOIN]") {

    std::shared_ptr<CRDT::CRDTGraph> G = Graph::get().get_G();

    SECTION("Join full graph") {}
    SECTION("Join empty full graph") {}
    SECTION("Join delta add") {}
    SECTION("Join delta remove") {}
    SECTION("Join delta update") {}
}
*/
