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
#include <string>

using namespace CRDT;

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

        REQUIRE_FALSE(n_id.has_value());

    }
    SECTION("Get a node that does not exists by name") {
        G->reset();

        std::optional<Node> n_name = G->get_node("no existe");

        REQUIRE_FALSE(n_name.has_value());

    }

    SECTION("Insert a new node") {

        Node n;
        n.name("test");
        n.id(75000);
        n.type("testtype");
        std::optional<int> r  = G->insert_node(n);
        REQUIRE(r.has_value());
        REQUIRE(r == 75000);
    }

    SECTION("Get an existing node by id") {

        std::optional<Node> n_id = G->get_node(75000);
        REQUIRE(n_id.has_value());

    }

    SECTION("Get an existing node by name") {

        std::optional<Node> n_name = G->get_node("test");
        REQUIRE(n_name.has_value());
    }

    SECTION("Update existing node") {

        std::optional<Node> n_id = G->get_node(75000);
        REQUIRE(n_id.has_value());

        G->add_attrib(n_id.value(), "level", 1);
        //bool r = G->insert_or_assign_node(n_id.value());
        bool r = G->update_node(n_id.value());

        REQUIRE(r);

    }

    SECTION("Remove an attribute") {

        std::optional<Node> n_id = G->get_node(75000);
        REQUIRE(n_id.has_value());

        G->remove_attrib_by_name(n_id.value(), "level");
        REQUIRE(n_id->attrs().find("level") == n_id->attrs().end());
    }



    SECTION("Insert an existent node") {
        Node n;
        n.id(75000);
        REQUIRE_THROWS(G->insert_node(n));
    }

    SECTION("Update an existent node with different name") {

        Node n;
        n.name("test2");
        n.id(75000);
        n.type("testtype");
        REQUIRE_THROWS(G->update_node(n));
    }

    SECTION("Update an existent node with different id") {

        Node n;
        n.name("test");
        n.id(7500166);
        n.type("testtype");
        REQUIRE_THROWS(G->update_node(n));
    }

    SECTION("Delete existing node by id") {

        bool r = G->delete_node(75000);
        REQUIRE(r);
        REQUIRE(G->get_node(75000) == std::nullopt);

    }


    SECTION("Delete existing node by name") {

        Node n;
        n.name("test1");
        n.id(75001);
        n.type("testtype");
        auto r  = G->insert_node(n);
        REQUIRE(r.has_value());
        bool r2 = G->delete_node("test1");
        REQUIRE(r2);
        REQUIRE(G->get_node("test1") == std::nullopt);

    }


    SECTION("Delete a node that does not exists by id") {

        bool r = G->delete_node(75000);
        REQUIRE_FALSE(r);

    }


}


TEST_CASE("Edge operations", "[EDGE]") {

    std::shared_ptr<CRDT::CRDTGraph> G = Graph::get().get_G();

    SECTION("Get an edge that does not exists by id") {

        std::optional<Edge> e_id = G->get_edge(666666, 77777, "K");

        REQUIRE_FALSE(e_id.has_value());
    }

    SECTION("Get an edge that does not exists by name") {

        std::optional<Edge> e_name = G->get_edge("no existe", "otro no existe", "K");

        REQUIRE_FALSE(e_name.has_value());
    }
    SECTION("Insert a new edge") {


        Node n;
        n.name("test");
        n.id(85000);
        n.type("testtype");
        REQUIRE(G->insert_node(n).has_value());


        n = Node();
        n.name("test2");
        n.id(87000);
        n.type("testtype");
        REQUIRE(G->insert_node(n).has_value());

        Edge e;
        e.type("testtype");
        e.to(87000);
        e.from(85000);

        bool r  = G->insert_or_assign_edge(e);
        REQUIRE(r);

    }
    SECTION("Get an existing edge by id") {

        std::optional<Edge> e = G->get_edge(85000, 87000, "testtype");
        REQUIRE(e.has_value());

    }

    SECTION("Get an existing edge by name") {

        std::optional<Edge> e_name = G->get_edge("test", "test2", "testtype");
        REQUIRE(e_name.has_value());

    }

    SECTION("Update existing edge") {
        std::optional<Edge> e = G->get_edge(85000, 87000, "testtype");
        REQUIRE(e.has_value());
        G->add_attrib(e.value(), "att", std::string("a"));
        bool r = G->insert_or_assign_edge(e.value());
        REQUIRE(r);

    }
    SECTION("Delete existing edge by id") {

        bool r = G->delete_edge(85000, 87000, "testtype");
        REQUIRE(r);
        REQUIRE(G->get_edge(85000, 87000, "testtype") == std::nullopt);

    }

    SECTION("Delete existing edge by name") {
        Edge e;
        e.type("testtype");
        e.to(87000);
        e.from(85000);

        bool r  = G->insert_or_assign_edge(e);
        REQUIRE(r);
        r = G->delete_edge("test", "test2", "testtype");
        REQUIRE(r);
        REQUIRE(G->get_edge("test", "test2", "testtype") == std::nullopt);

    }

    SECTION("Delete an edge that does not exists") {
        bool r = G->delete_edge(85000, 87000, "testtype");
        REQUIRE_FALSE(r);
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
        REQUIRE_FALSE(std::filesystem::exists(wempty_file));
        u.write_to_json_file(wempty_file);
        REQUIRE(std::filesystem::exists(wempty_file));
        std::ifstream file(wempty_file);
        const std::string c = "{\n"
                            "    \"DSRModel\": {\n"
                            "        \"symbols\": {\n"
                            "        }\n"
                            "    }\n"
                            "}\n";
        std::string content = std::string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

        REQUIRE( content ==  c);

    }

    SECTION("Load a non-empty file") {
        G->reset();
        REQUIRE(G->size() == 0);

        u.read_from_json_file(test_file);
        REQUIRE(G->size() == 12);
    }

    SECTION("Write a file") {
        std::filesystem::remove(wtest_file);
        REQUIRE_FALSE(std::filesystem::exists(wtest_file));
        u.write_to_json_file(wtest_file);
        REQUIRE(std::filesystem::exists(wtest_file));
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
        std::optional<int> r = G->insert_node(n);
        REQUIRE(r.has_value());

        std::optional<int> id = G->get_id_from_name("name_node");
        REQUIRE(id.has_value());
        REQUIRE(id.value() == 12345);

    }


    SECTION("Get id from a name that does not exist") {
        std::optional<int> id = G->get_id_from_name("name_no");
        REQUIRE(!id.has_value());
    }

    SECTION("Get name from an id") {
        Node n;
        n.name("name_node"); n.id(12345); n.type("t");
        bool r = G->update_node(n);
        REQUIRE(r);

        std::optional<std::string> name = G->get_name_from_id(12345);
        REQUIRE(name.has_value());
        REQUIRE(name.value() == "name_node");
    }

    SECTION("Get name from an id that does not exist") {
        std::optional<std::string> name = G->get_name_from_id(501245);
        REQUIRE(!name.has_value());
    }

    SECTION("Get nodes by type") {
        Node n;
        n.name("name_node2"); n.id(12346); n.type("t");
        auto r = G->insert_node(n);
        REQUIRE(r.has_value());

        vector<Node> ve = G->get_nodes_by_type("t");
        REQUIRE(ve.size() == 2);
        ve = G->get_nodes_by_type("no");
        REQUIRE(ve.empty());

    }
    SECTION("Get edges by type") {
        vector<Edge> ve = G->get_edges_by_type("RT");
        REQUIRE(ve.size() == 11);
        ve = G->get_edges_by_type("no");
        REQUIRE(ve.empty());
    }
    SECTION("Get edges to a node (id)") {
        vector<Edge> ve = G->get_edges_to_id(100);
        REQUIRE(ve.empty());
        ve = G->get_edges_to_id(118);
        REQUIRE(ve.size() == 1);
    }
/*
     TODO: MODIFICAR CON NUEVA REPRESENTACIÓN del grafo para el usuario.

    SECTION("Get edges from a node") {
        std::optional<std::unordered_map<std::pair<int, std::string>, Edge,pair_hash>> ve = G->get_edges(101);
        REQUIRE(ve.has_value() == true);
        REQUIRE(ve->size() == 5);
        std::optional<Node> n = G->get_node(101);
        REQUIRE(n.has_value());
        REQUIRE(n->fano() == ve.value());
    }

    SECTION("Get edges from a node that does not exist") {
        std::optional<std::map<EdgeKey, Edge>> ve = G->get_edges(45550);
        REQUIRE(!ve.has_value());
    }

    SECTION("Get edges from a node that has no edges") {
        std::optional<std::map<EdgeKey, Edge>> ve = G->get_edges(118);
        REQUIRE(ve.has_value());
        REQUIRE(ve->empty());
    }
    */
}

TEST_CASE("Attributes operations", "[ATTRIBUTES]") {
    std::shared_ptr<CRDT::CRDTGraph> G = Graph::get().get_G();

    SECTION("Insert attribute by name (node) and insert") {
        std::optional<Node> n = G->get_node(100);
        REQUIRE(n.has_value());
        G->insert_attrib_by_name(n.value(), "att", 123);
        REQUIRE(n->attrs().find("att") != n->attrs().end());
    }

    SECTION("Insert an string attribute") {
        std::optional<Node> n = G->get_node(100);
        REQUIRE(n.has_value());
        G->insert_attrib_by_name(n.value(), "string_att", std::string("string att"));
        REQUIRE(n->attrs().find("att") != n->attrs().end());
        bool r = G->update_node(n.value());
        REQUIRE(r);
    }

    SECTION("Insert attribute by name (edge) and insert") {
        std::optional<Edge> e = G->get_edge(101, 111, "RT");
        REQUIRE(e.has_value());
        G->insert_attrib_by_name(e.value(), "att", 123);
        REQUIRE(e->attrs().find("att") != e->attrs().end());
        bool r = G->insert_or_assign_edge(e.value());
        REQUIRE(r);
    }
    SECTION("Update attribute by name") {
        std::optional<Node> n = G->get_node(100);
        REQUIRE(n.has_value());
        G->update_attrib_by_name(n.value(), "att", 125);
        REQUIRE(n->attrs()["att"].read_reg().val().dec() == 125);
        bool r = G->update_node(n.value());
        REQUIRE(r);
    }

    SECTION("Get attribute by name") {
        std::optional<Node> n = G->get_node(100);
        REQUIRE(n.has_value());
        std::optional<int> att = G->get_attrib_by_name<int>(n.value(), "att");
        REQUIRE(att.has_value());
        REQUIRE(att.value() == 125);

    }

    //TODO: No hay level?.
    /*
    SECTION("Get node level") {
        std::optional<Node> n = G->get_node(101);
        REQUIRE(n.has_value());
        std::optional<int> level = G->get_node_level(n.value());
        REQUIRE(level.has_value() == true);
        REQUIRE(level.value() == 0);

        n = G->get_node(12345);
        REQUIRE(n.has_value());
        level = G->get_node_level(n.value());
        REQUIRE(level.has_value() == false);
    }
     */
    SECTION("Get node type") {
        std::optional<Node> n = G->get_node(100);
        REQUIRE(n.has_value());
        std::optional<std::string> type = G->get_node_type(n.value());
        REQUIRE(type.has_value());
        REQUIRE(type.value() == "world");

    }
}


TEST_CASE("Native types in attributes", "[ATTRIBUTES]") {
    std::shared_ptr<CRDT::CRDTGraph> G = Graph::get().get_G();

    SECTION("Insert a string attribute") {
        std::optional<Node> n = G->get_node(100);
        REQUIRE(n.has_value());
        G->insert_attrib_by_name(n.value(), "string_att", std::string("string att"));
        REQUIRE(n->attrs().find("att") != n->attrs().end());
        std::optional<Node> n2 = G->get_node(100);
        REQUIRE(n2.has_value());
        REQUIRE(n.value() == n2.value());
    }
    SECTION("Get a string attribute") {
        std::optional<Node> n = G->get_node(100);
        REQUIRE(n.has_value());
        std::optional<std::string> st = G->get_attrib_by_name<std::string>(n.value(), "string_att");
        REQUIRE(st.has_value());
    }

    SECTION("Insert an int attribute") {
        std::optional<Node> n = G->get_node(100);
        REQUIRE(n.has_value());
        G->insert_attrib_by_name(n.value(), "int_att", 11);
        REQUIRE(n->attrs().find("att") != n->attrs().end());
        std::optional<Node> n2 = G->get_node(100);
        REQUIRE(n2.has_value());
        REQUIRE(n.value() == n2.value());
    }
    SECTION("Get an int attribute") {
        std::optional<Node> n = G->get_node(100);
        REQUIRE(n.has_value());
        std::optional<int> st = G->get_attrib_by_name<int>(n.value(), "int_att");
        REQUIRE(st.has_value());
    }

    SECTION("Insert a float attribute") {
        std::optional<Node> n = G->get_node(100);
        REQUIRE(n.has_value());
        G->insert_attrib_by_name(n.value(), "float_att", static_cast<float>(11.0));
        REQUIRE(n->attrs().find("att") != n->attrs().end());
        std::optional<Node> n2 = G->get_node(100);
        REQUIRE(n2.has_value());
        REQUIRE(n.value() == n2.value());
    }
    SECTION("Get a float attribute") {
        std::optional<Node> n = G->get_node(100);
        REQUIRE(n.has_value());
        std::optional<float> st = G->get_attrib_by_name<float>(n.value(), "float_att");
        REQUIRE(st.has_value());
    }

    SECTION("Insert a float_vector attribute") {
        std::optional<Node> n = G->get_node(100);
        REQUIRE(n.has_value());
        G->insert_attrib_by_name(n.value(), "float_vec_att", vector<float>{11.0, 167.23, 55.66});
        REQUIRE(n->attrs().find("att") != n->attrs().end());
        std::optional<Node> n2 = G->get_node(100);
        REQUIRE(n2.has_value());
        REQUIRE(n.value() == n2.value());
    }
    SECTION("Get a float_vector attribute") {
        std::optional<Node> n = G->get_node(100);
        REQUIRE(n.has_value());
        std::optional<vector<float>> st = G->get_attrib_by_name<vector<float>>(n.value(), "float_vec_att");
        REQUIRE(st.has_value());
    }
    SECTION("Insert a bool attribute") {
        std::optional<Node> n = G->get_node(100);
        REQUIRE(n.has_value());
        G->insert_attrib_by_name(n.value(), "bool_att", true);
        REQUIRE(n->attrs().find("att") != n->attrs().end());
        std::optional<Node> n2 = G->get_node(100);
        REQUIRE(n2.has_value());
        REQUIRE(n.value() == n2.value());
    }
    SECTION("Get a bool attribute") {
        std::optional<Node> n = G->get_node(100);
        REQUIRE(n.has_value());
        std::optional<bool> st = G->get_attrib_by_name<bool>(n.value(), "bool_att");
        REQUIRE(st.has_value());
    }

    SECTION("Get a qvec attribute") {
        std::optional<Edge> n = G->get_edge(111, 117, "RT");
        REQUIRE(n.has_value());
        std::optional<QVec> st = G->get_attrib_by_name<QVec>(n.value(), "translation");
        REQUIRE(st.has_value());
    }
    SECTION("Get a qmat attribute") {
        std::optional<Edge> n = G->get_edge(111, 117, "RT");
        REQUIRE(n.has_value());
        std::optional<QMat> st = G->get_attrib_by_name<QMat>(n.value(), "rotation_euler_xyz");
        REQUIRE(st.has_value());
    }

    SECTION("Get an attribute with the wrong type") {
        std::optional<Edge> n = G->get_edge(111, 117, "RT");
        REQUIRE(n.has_value());
        REQUIRE_THROWS(G->get_attrib_by_name<int>(n.value(), "rotation_euler_xyz"));
    }
}


//Scenarios
SCENARIO( "Node insertions, updates and removals", "[NODE]" ) {
    std::shared_ptr<CRDT::CRDTGraph> G = Graph::get().get_G();

    GIVEN("A new Node")
    {
        Node n;
        Value v;
        n.id(2222);
        n.type("robot");
        n.agent_id(0);
        n.name("robot1");
        G->add_attrib(n, "att", std::string("value"));

        WHEN("The node is inserted")
        {
            size_t size = G->size();
            auto r = G->update_node(n);
            REQUIRE(r);

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

        AND_WHEN("The node is updated")
        {
            size_t size = G->size();
            G->add_attrib(n, "new att", 11);
            bool r = G->update_node(n);
            REQUIRE(r);

            THEN("The graph size is equal")
            {
                REQUIRE(size == G->size());
            }
            THEN("You can get the node")
            {
                std::optional<Node> node = G->get_node(2222);
                REQUIRE(node.has_value());
                THEN("The requested node is equal to the inserted node") {
                    REQUIRE(node.value() == n);
                }
            }
        }

        AND_WHEN("The node is deleted")
        {
            size_t size = G->size();
            G->delete_node(2222);
            THEN("The graph size is smaller")
            {
                REQUIRE(size > G->size());
            }
            AND_THEN("You can't get the node")
            {
                std::optional<Node> node = G->get_node(2222);
                REQUIRE(!node.has_value());
            }
            AND_THEN("You can't insert the node again")
            {
                auto r = G->insert_node(n);
                REQUIRE_FALSE(r.has_value());
            }
        }
    }
    GIVEN("A deleted node")
    {
        //75000
        std::optional<Node> node = G->get_node(2222);
        THEN("Optional value is empty")
        {
            REQUIRE_FALSE(node.has_value());
        }
        AND_THEN("You can't insert the node again")
        {
            Node n;
            n.id(2222);
            auto r = G->insert_node(n);
            REQUIRE_FALSE(r.has_value());
        }
    }
    GIVEN("An invalid node")
    {
        Node n; n.id(-1);
        THEN("Can't insert it")
        {
            auto r = G->insert_node(n);
            REQUIRE_FALSE(r.has_value());
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
