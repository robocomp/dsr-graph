//
// Created by juancarlos on 7/5/20.
//

#include <iostream>
#include <optional>
#include <filesystem>


#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include "catch.hpp"
#include "../../../../dsr/api/dsr_api.h"
#include <iostream>
#include <string>

CATCH_CONFIG_MAIN


// ICE includes
#include <Ice/Ice.h>
#include <IceStorm/IceStorm.h>
#include <Ice/Application.h>

using namespace std::literals;

REGISTER_TYPE(att, std::reference_wrapper<const string>)
REGISTER_TYPE(int_, int32_t)
REGISTER_TYPE(float_, float)
REGISTER_TYPE(bool_, bool)
REGISTER_TYPE(uint_, uint32_t)
REGISTER_TYPE(string_, std::reference_wrapper<const string>)
REGISTER_TYPE(vec_byte, std::reference_wrapper<const std::vector<uint8_t>>)
REGISTER_TYPE(vec_float, std::reference_wrapper<const std::vector<float>>)



using namespace DSR;

class Graph {
    public:
        static Graph& get() {
            static Graph instance;
            return instance;
        }

        Graph(Graph const&)           = delete;
        void operator=(Graph const&)  = delete;

        shared_ptr<DSR::DSRGraph> get_G() { return G;}
    private:
        Graph () {
            std::thread([](){
                std::system("/home/robocomp/robocomp/components/dsr-graph/components/idserver/bin/idserver --Ice.Config=/home/robocomp/robocomp/components/dsr-graph/components/crdt_rtps_dsr_tests/src/unittests/testfiles/config_idserver");
                while (true) {this_thread::yield();};
            }).detach();

            this_thread::sleep_for(300ms);
            auto c = Ice::initialize();
            auto pr = c->stringToProxy("dsrgetid:tcp -h localhost -p 11000");
            dsrgetid_proxy = Ice::uncheckedCast<RoboCompDSRGetID::DSRGetIDPrx>( pr );
            G = make_shared<DSR::DSRGraph>(0, "test", 1551, "", dsrgetid_proxy);

        }
        RoboCompDSRGetID::DSRGetIDPrxPtr dsrgetid_proxy;
        std::shared_ptr<DSR::DSRGraph> G;
};

TEST_CASE("Node operations", "[NODE]") {

    std::shared_ptr<DSR::DSRGraph> G = Graph::get().get_G();

    SECTION("Get a node that does not exists by id") {
        G->reset();
        std::optional<Node> n_id = G->get_node(3333333);
        REQUIRE_FALSE(n_id.has_value());
    }
    SECTION("Get a node that does not exists by name") {
        G->reset();
        std::optional<Node> n_name = G->get_node("no existe");
        REQUIRE_FALSE(n_name.has_value());
    }

    SECTION("Insert a new node") {
        Node n (1550, "testtype");
        std::optional<int> r  = G->insert_node(n);
        REQUIRE(r.has_value());
        REQUIRE(r.value() == static_cast<int>(G->size()));
    }

    SECTION("Create a node with an invalid type") {
        REQUIRE_THROWS(Node (1550, "aaaaa"));
    }

    SECTION("Get an existing node by id") {
        std::optional<Node> n_id = G->get_node("testtype_"+ std::to_string(G->size()));
        REQUIRE(n_id.has_value());
    }

    SECTION("Get an existing node by name") {
        std::optional<Node> n_name = G->get_node("testtype_"+ std::to_string(G->size()));
        REQUIRE(n_name.has_value());
    }

    SECTION("Update existing node") {

        std::optional<Node> n_id = G->get_node("testtype_"+ std::to_string(G->size()));
        REQUIRE(n_id.has_value());

        G->add_attrib_local<level_att>(n_id.value(), 1);
        bool r = G->update_node(n_id.value());

        REQUIRE(r);

    }

    SECTION("Remove an attribute") {

        std::optional<Node> n_id = G->get_node("testtype_"+ std::to_string(G->size()));
        REQUIRE(n_id.has_value());
        G->remove_attrib_by_name(n_id.value(), "level");
        REQUIRE(n_id->attrs().find("level") == n_id->attrs().end());
    }


    SECTION("Update an existent node with different name") {

        std::optional<Node> n_ = G->get_node("testtype_"+ std::to_string(G->size()));
        REQUIRE(n_.has_value());
        n_->name("test2");
        REQUIRE_THROWS(G->update_node(n_.value()));
    }

    SECTION("Update an existent node with different id") {

        Node n;
        n.name("testtype_"+ std::to_string(G->size()));
        n.id(7500166);
        n.type("testtype");
        REQUIRE_THROWS(G->update_node(n));
    }

    SECTION("Delete existing node by id") {

        std::optional<Node> n_ = G->get_node("testtype_"+ std::to_string(G->size()));
        REQUIRE(n_.has_value());
        bool r = G->delete_node(n_->id());
        REQUIRE(r);
        REQUIRE(G->get_node(n_->name()) == std::nullopt);

    }


    SECTION("Delete existing node by name") {

        Node n;
        n.type("testtype");
        auto r  = G->insert_node(n);
        REQUIRE(r.has_value());
        bool r2 = G->delete_node("testtype_"+ std::to_string(G->size()+1));
        REQUIRE(r2);
        REQUIRE(G->get_node(r.value()) == std::nullopt);

    }

    SECTION("Delete a node that does not exists by id") {
        bool r = G->delete_node(75000);
        REQUIRE_FALSE(r);
    }

}


TEST_CASE("Edge operations", "[EDGE]") {

    std::shared_ptr<DSR::DSRGraph> G = Graph::get().get_G();

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
        n.type("testtype");
        auto id1 = G->insert_node(n);
        REQUIRE(id1.has_value());


        n = Node();
        n.type("testtype");
        auto id2 = G->insert_node(n);
        REQUIRE(id2.has_value());

        Edge e;
        e.type("testtype");
        e.to(id2.value());
        e.from(id1.value());

        bool r  = G->insert_or_assign_edge(e);
        REQUIRE(r);

    }

    SECTION("Get an existing edge by id") {

        std::optional<Edge> e = G->get_edge(3, 4, "testtype");
        REQUIRE(e.has_value());

    }

    SECTION("Get an existing edge by name") {

        std::optional<Edge> e_name = G->get_edge("testtype_3", "testtype_4", "testtype");
        REQUIRE(e_name.has_value());

    }

    SECTION("Update existing edge") {
        std::optional<Edge> e = G->get_edge(3, 4, "testtype");
        REQUIRE(e.has_value());
        G->add_attrib_local<att_att>(e.value(),  std::string("a"));
        bool r = G->insert_or_assign_edge(e.value());
        REQUIRE(r);

    }
    SECTION("Delete existing edge by id") {

        bool r = G->delete_edge(3, 4, "testtype");
        REQUIRE(r);
        REQUIRE(G->get_edge(3, 4, "testtype") == std::nullopt);

    }

    SECTION("Delete existing edge by name") {
        Edge e;
        e.type("testtype");
        e.to(3);
        e.from(4);

        bool r  = G->insert_or_assign_edge(e);
        REQUIRE(r);
        r = G->delete_edge("testtype_4", "testtype_3", "testtype");
        REQUIRE(r);
        REQUIRE(G->get_edge("testtype_4", "testtype_3", "testtype") == std::nullopt);

    }

    SECTION("Delete an edge that does not exists") {
        bool r = G->delete_edge(3, 4, "testtype");
        REQUIRE_FALSE(r);
    }

    SECTION("Get RTMat from edge") {
        //TOOD: implement
    }

    SECTION("Get edge RT") {
        //TOOD: implement
    }

    SECTION("Insert or assign edge RT") {
        //TOOD: implement
    }

}

TEST_CASE("File operations (Utilities sub-api)", "[FILE]") {

    std::shared_ptr<DSR::DSRGraph> G = Graph::get().get_G();
    //DSR::Utilities u (G.get());

    const std::string empty_file = "/home/robocomp/robocomp/components/dsr-graph/components/crdt_rtps_dsr_tests/src/unittests/testfiles/empty_file.json";
    const std::string wempty_file = "/home/robocomp/robocomp/components/dsr-graph/components/crdt_rtps_dsr_tests/src/unittests/testfiles/write_empty_file.json";

    const std::string test_file = "/home/robocomp/robocomp/components/dsr-graph/components/crdt_rtps_dsr_tests/src/unittests/testfiles/initial_dsr2.json";
    const std::string wtest_file = "/home/robocomp/robocomp/components/dsr-graph/components/crdt_rtps_dsr_tests/src/unittests/testfiles/write_initial_dsr2.json";



    SECTION("Load an empty file") {
        G->reset();
        REQUIRE(G->size() == 0);

        G->read_from_json_file(empty_file);
        REQUIRE(G->size() == 0);

    }

    SECTION("Write an empty file") {
        std::filesystem::remove(wempty_file);
        REQUIRE_FALSE(std::filesystem::exists(wempty_file));
        G->write_to_json_file(wempty_file);
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

        G->read_from_json_file(test_file);
        REQUIRE(G->size() == 75);
    }

    SECTION("Write a file") {
        std::filesystem::remove(wtest_file);
        REQUIRE_FALSE(std::filesystem::exists(wtest_file));
        G->write_to_json_file(wtest_file);
        REQUIRE(std::filesystem::exists(wtest_file));
        std::ifstream file(wtest_file);
        std::ifstream file2(test_file);

        QString content = QString::fromStdString(std::string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>()));
        QString content2 =  QString::fromStdString(std::string((std::istreambuf_iterator<char>(file2)), std::istreambuf_iterator<char>()));

        REQUIRE( QJsonDocument::fromJson(content.toUtf8()) ==  QJsonDocument::fromJson(content2.toUtf8()));


    }
}


TEST_CASE("Maps operations", "[UTILS]") {
    std::shared_ptr<DSR::DSRGraph> G = Graph::get().get_G();

    SECTION("Get id from a name") {
        Node n;
        n.name("name_node");  n.type("t");
        std::optional<int> r = G->insert_node(n);
        REQUIRE(r.has_value());

        std::optional<int> id = G->get_id_from_name("t_" + std::to_string(r.value()));
        REQUIRE(id.has_value());
        REQUIRE(id.value() == 5);

    }


    SECTION("Get id from a name that does not exist") {
        std::optional<int> id = G->get_id_from_name("t_55");
        REQUIRE(!id.has_value());
    }

    SECTION("Get name from an id") {
        Node n;
        n.name("t_5"); n.id(5); n.type("t");
        bool r = G->update_node(n);
        REQUIRE(r);

        std::optional<std::string> name = G->get_name_from_id(5);
        REQUIRE(name.has_value());
        REQUIRE(name.value() == "t_5");
    }

    SECTION("Get name from an id that does not exist") {
        std::optional<std::string> name = G->get_name_from_id(788987);
        REQUIRE(!name.has_value());
    }

    SECTION("Get nodes by type") {
        Node n;
        n.type("t");
        auto r = G->insert_node(n);
        REQUIRE(r.has_value());

        vector<Node> ve = G->get_nodes_by_type("t");
        REQUIRE(ve.size() == 2);
        ve = G->get_nodes_by_type("no");
        REQUIRE(ve.empty());

    }
    SECTION("Get edges by type") {
        vector<Edge> ve = G->get_edges_by_type("RT");
        REQUIRE(ve.size() == 74);
        ve = G->get_edges_by_type("no");
        REQUIRE(ve.empty());
    }
    SECTION("Get edges to a node (id)") {
        vector<Edge> ve = G->get_edges_to_id(100);
        REQUIRE(ve.empty());
        ve = G->get_edges_to_id(13);
        REQUIRE(ve.size() == 1);
    }


    SECTION("Get edges from a node") {
        std::optional<std::map<std::pair<uint32_t, std::string>, DSR::Edge>> ve = G->get_edges(1);
        REQUIRE(ve.has_value() == true);
        REQUIRE(ve->size() == 2);
        std::optional<Node> n = G->get_node(1);
        REQUIRE(n.has_value());
        REQUIRE(n->fano() == ve.value());
    }

    SECTION("Get edges from a node that does not exist") {
        std::optional<std::map<std::pair<uint32_t, std::string>, DSR::Edge/*, pair_hash*/>> ve = G->get_edges(45550);
        REQUIRE(!ve.has_value());
    }

    SECTION("Get edges from a node that has no edges") {
        std::optional<std::map<std::pair<uint32_t, std::string>, DSR::Edge/*, pair_hash*/>> ve = G->get_edges(13);
        REQUIRE(ve.has_value());
        REQUIRE(ve->empty());
    }

}

TEST_CASE("Attributes operations (Compile time type-check)", "[ATTRIBUTES]") {
    std::shared_ptr<DSR::DSRGraph> G = Graph::get().get_G();

    SECTION("Insert attribute by name (node) and insert in G") {
        std::optional<Node> n = G->get_node(1);
        REQUIRE(n.has_value());
        G->insert_attrib_by_name<int_>(n.value(),  123);
        REQUIRE(n->attrs().find("int_") != n->attrs().end());
    }

    SECTION("Insert an string attribute") {
        std::optional<Node> n = G->get_node(1);
        REQUIRE(n.has_value());
        G->insert_attrib_by_name<string_>(n.value(),  std::string("string att"));
        REQUIRE(n->attrs().find("string_") != n->attrs().end());
        bool r = G->update_node(n.value());
        REQUIRE(r);
    }

    SECTION("Insert attribute by name (edge) and insert in G") {
        std::optional<Edge> e = G->get_edge(1, 2, "RT");
        REQUIRE(e.has_value());
        G->insert_attrib_by_name<int_>(e.value(), 123);
        REQUIRE(e->attrs().find("int_") != e->attrs().end());
        bool r = G->insert_or_assign_edge(e.value());
        REQUIRE(r);
    }
    SECTION("Update attribute by name") {
        std::optional<Node> n = G->get_node(1);
        REQUIRE(n.has_value());
        G->update_attrib_by_name<int_>(n.value(), 125);
        REQUIRE(get<std::int32_t>(n->attrs()["int_"].value()) == 125);
        bool r = G->update_node(n.value());
        REQUIRE(r);
    }

    SECTION("Get attribute by name") {
        std::optional<Node> n = G->get_node(1);
        REQUIRE(n.has_value());
        std::optional<int> att = G->get_attrib_by_name<int_>(n.value());
        REQUIRE(att.has_value());
        REQUIRE(att.value() == 125);

    }

}

TEST_CASE("Convenience methods", "[CONVENIENCE METHODS]") {
    std::shared_ptr<DSR::DSRGraph> G = Graph::get().get_G();

    SECTION("Get node level") {
        std::optional<Node> n = G->get_node(2);
        REQUIRE(n.has_value());
        std::optional<int> level = G->get_node_level(n.value());
        REQUIRE(level.has_value() == true);
        REQUIRE(level.value() == 1);


        Node in;
        in.type("test");
        auto id = G->insert_node(in);
        REQUIRE(id.has_value());
        n = G->get_node(id.value());
        REQUIRE(n.has_value());
        level = G->get_node_level(n.value());
        REQUIRE(level.has_value() == false);
    }

    SECTION("Get node type") {
        std::optional<Node> n = G->get_node(1);
        REQUIRE(n.has_value());
        std::optional<std::string> type = G->get_node_type(n.value());
        REQUIRE(type.has_value());
        REQUIRE(type.value() == "world");
    }

    SECTION("Get parent id") {
        std::optional<Node> n = G->get_node(2);
        REQUIRE(n.has_value());
        std::optional<uint32_t> id = G->get_parent_id(n.value());
        REQUIRE(id.has_value());
        REQUIRE(id.value() == 1);

        //std::optional<Node> parent =  G->get_node(1);
        //std::optional<uint32_t> id_empty = G->get_parent_id(parent.value());
        //REQUIRE_FALSE(id_empty.has_value());
    }

    SECTION("Get parent node") {
        std::optional<Node> n = G->get_node(2);
        REQUIRE(n.has_value());
        std::optional<Node> p = G->get_parent_node(n.value());
        REQUIRE(p.has_value());
        std::optional<Node> parent =  G->get_node(1);
        REQUIRE(parent.has_value());
        REQUIRE(p.value() == parent.value());

        std::optional<Node> parent_empty = G->get_parent_node(parent.value());
        REQUIRE_FALSE(parent_empty.has_value());
    }

    SECTION("get_node_root") {
        std::optional<Node> n = G->get_node(1);
        REQUIRE(n.has_value());
        std::optional<Node> n2 = G->get_node_root();
        REQUIRE(n2.has_value());
        REQUIRE(n2.value() == n.value());
    }
}

TEST_CASE("Attributes operations II (Runtime time type-check)", "[RUNTIME ATTRIBUTES]") {
    std::shared_ptr<DSR::DSRGraph> G = Graph::get().get_G();

    SECTION("Insert an attribute by name") {
        std::optional<Node> n = G->get_node(1);
        REQUIRE(n.has_value());
        G->runtime_checked_add_or_modify_attrib_local(n.value(), "int_", 133);
        REQUIRE(get<std::int32_t>(n->attrs()["int_"].value()) == 133);
        bool r = G->update_node(n.value());
        REQUIRE(r);
        REQUIRE_THROWS(G->runtime_checked_add_or_modify_attrib_local(n.value(), "int_", 133.0f));
    }

    SECTION("Modify an attribute by name") {
        std::optional<Node> n = G->get_node(1);
        REQUIRE(n.has_value());
        REQUIRE(G->runtime_checked_modify_attrib_local(n.value(), "int_", 111));
        REQUIRE(get<std::int32_t>(n->attrs()["int_"].value()) == 111);
        bool r = G->update_node(n.value());
        REQUIRE(r);
    }

    SECTION("Update attribute by name") {
        std::optional<Node> n = G->get_node(1);
        REQUIRE(n.has_value());
        REQUIRE(G->runtime_checked_update_attrib_by_name(n.value(), "int_", 177));
        REQUIRE(get<std::int32_t>(n->attrs()["int_"].value()) == 177);
        REQUIRE(get<std::int32_t>(n->attrs()["int_"].value()) == G->get_attrib_by_name<int_>(n.value()));
    }

    SECTION("Remove an attribute by name") {
        std::optional<Node> n = G->get_node(1);
        REQUIRE(n.has_value());
        REQUIRE(G->remove_attrib_local(n.value(), "int_"));
        REQUIRE(n->attrs().find("int_") == n->attrs().end());
        bool r = G->update_node(n.value());
        REQUIRE(r);
    }

    SECTION("Insert attribute by name (edge) and insert in G") {
        std::optional<Edge> e = G->get_edge(1, 2, "RT");
        REQUIRE(e.has_value());
        REQUIRE(G->runtime_checked_insert_attrib_by_name(e.value(), "new_int", 123));
        REQUIRE(e->attrs().find("new_int") != e->attrs().end());
        bool r = G->insert_or_assign_edge(e.value());
        REQUIRE(r);
    }

}

TEST_CASE("Native types in attributes", "[ATTRIBUTES]") {
    std::shared_ptr<DSR::DSRGraph> G = Graph::get().get_G();

    SECTION("Insert a string attribute") {
        std::optional<Node> n = G->get_node(1);
        REQUIRE(n.has_value());
        G->insert_attrib_by_name<string_>(n.value(), std::string("string att"));
        REQUIRE(n->attrs().find("string_") != n->attrs().end());
        std::optional<Node> n2 = G->get_node(1);
        REQUIRE(n2.has_value());
        REQUIRE(n.value() == n2.value());
    }
    SECTION("Get a string attribute") {
        std::optional<Node> n = G->get_node(1);
        REQUIRE(n.has_value());
        std::optional<std::string> st = G->get_attrib_by_name<string_>(n.value());
        REQUIRE(st.has_value());
    }

    SECTION("Insert an int attribute") {
        std::optional<Node> n = G->get_node(1);
        REQUIRE(n.has_value());
        G->insert_attrib_by_name<int_>(n.value(),  11);
        REQUIRE(n->attrs().find("int_") != n->attrs().end());
        std::optional<Node> n2 = G->get_node(1);
        REQUIRE(n2.has_value());
        REQUIRE(G->get_attrib_by_name<int_>( n.value()) == G->get_attrib_by_name<int_>( n2.value()));
    }
    SECTION("Get an int attribute") {
        std::optional<Node> n = G->get_node(1);
        REQUIRE(n.has_value());
        std::optional<int> st = G->get_attrib_by_name<int_>(n.value());
        REQUIRE(st.has_value());
    }

    SECTION("Insert a float attribute") {
        std::optional<Node> n = G->get_node(1);
        REQUIRE(n.has_value());
        G->insert_attrib_by_name<float_>(n.value(), static_cast<float>(11.0));
        REQUIRE(n->attrs().find("float_") != n->attrs().end());
        std::optional<Node> n2 = G->get_node(1);
        REQUIRE(n2.has_value());
        REQUIRE(G->get_attrib_by_name<float_>( n.value()) == G->get_attrib_by_name<float_>( n2.value()));

    }
    SECTION("Get a float attribute") {
        std::optional<Node> n = G->get_node(1);
        REQUIRE(n.has_value());
        std::optional<float> st = G->get_attrib_by_name<float_>(n.value());
        REQUIRE(st.has_value());
    }

    SECTION("Insert a float_vector attribute") {
        std::optional<Node> n = G->get_node(1);
        REQUIRE(n.has_value());
        G->insert_attrib_by_name<vec_float_att>(n.value(), vector<float>{11.0, 167.23, 55.66});
        REQUIRE(n->attrs().find("vec_float") != n->attrs().end());
        std::optional<Node> n2 = G->get_node(1);
        REQUIRE(n2.has_value());
        REQUIRE(G->get_attrib_by_name<vec_float_att>( n.value()).value().get() == G->get_attrib_by_name<vec_float_att>( n2.value()).value().get());
    }
    SECTION("Get a float_vector attribute") {
        std::optional<Node> n = G->get_node(1);
        REQUIRE(n.has_value());
        std::optional<vector<float>> st = G->get_attrib_by_name<vec_float_att>(n.value());
        REQUIRE(st.has_value());
    }
    SECTION("Insert a bool attribute") {
        std::optional<Node> n = G->get_node(1);
        REQUIRE(n.has_value());
        G->insert_attrib_by_name<bool_>(n.value(), true);
        REQUIRE(n->attrs().find("bool_") != n->attrs().end());
        std::optional<Node> n2 = G->get_node(1);
        REQUIRE(n2.has_value());
        REQUIRE(G->get_attrib_by_name<bool_>( n.value()) == G->get_attrib_by_name<bool_>( n2.value()));
    }
    SECTION("Get a bool attribute") {
        std::optional<Node> n = G->get_node(1);
        REQUIRE(n.has_value());
        std::optional<bool> st = G->get_attrib_by_name<bool_>(n.value());
        REQUIRE(st.has_value());
    }

}



//Scenarios
SCENARIO( "Node insertions, updates and removals", "[NODE]" ) {
    std::shared_ptr<DSR::DSRGraph> G = Graph::get().get_G();

    GIVEN("A new Node")
    {
        Node n;
        n.type("robot");
        n.agent_id(0);
        G->add_attrib_local<att_att>(n, std::string("value"));

        WHEN("When we insert a new node")
        {
            THEN("The node is inserted")
            {
                G->reset();
                auto r = G->insert_node(n);
                REQUIRE(r);
            }
            THEN("The graph size is bigger")
            {
                REQUIRE(0 < G->size());
            }
            THEN("You can get the node")
            {
                Node n;
                n.type("robot");
                n.id(8);
                n.name("robot_8");
                n.agent_id(0);
                G->add_attrib_local<att_att>(n, std::string("value"));
                std::optional<Node> node = G->get_node(8);
                REQUIRE(node.has_value());
                THEN("The requested node is equal to the inserted node") {
                    REQUIRE(node.value() == n);
                }
            }
        }

        AND_WHEN("The node is updated")
        {
            size_t size = G->size();
            n.type("robot");
            n.id(8);
            n.name("robot_8");
            G->add_attrib_local<int_>(n,  11);
            bool r = G->update_node(n);
            REQUIRE(r);

            THEN("The graph size is equal")
            {
                REQUIRE(size == G->size());
            }
            THEN("You can get the node")
            {
                std::optional<Node> node = G->get_node(8);
                REQUIRE(node.has_value());
                THEN("The requested node has the inserted attribute") {
                     REQUIRE(G->get_attrib_by_name<int_>(node.value()) == 11);
                }
            }
        }

        AND_WHEN("The node is deleted")
        {
            size_t size = G->size();
            G->delete_node(8);
            THEN("The graph size is smaller")
            {
                REQUIRE(size > G->size());
            }
            AND_THEN("You can't get the node")
            {
                std::optional<Node> node = G->get_node(8);
                REQUIRE(!node.has_value());
            }
        }
    }
    GIVEN("A deleted node")
    {
        std::optional<Node> node = G->get_node(8);
        THEN("Optional value is empty")
        {
            REQUIRE_FALSE(node.has_value());
        }
        AND_THEN("You can't update the node again")
        {
            Node n;
            n.id(8);
            REQUIRE_THROWS(G->update_node(n));
        }
    }
}

/*
TEST_CASE("Join operations", "[JOIN]") {

    std::shared_ptr<DSR::DSRGraph> G = Graph::get().get_G();

    SECTION("Join full graph") {}
    SECTION("Join empty full graph") {}
    SECTION("Join delta add") {}
    SECTION("Join delta remove") {}
    SECTION("Join delta update") {}
}
*/
