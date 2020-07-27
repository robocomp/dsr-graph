//
// Created by crivac on 17/01/19.
//

#ifndef CRDT_GRAPH
#define CRDT_GRAPH

#include <iostream>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <chrono>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <any>
#include <memory>
#include <vector>
#include <variant>
#include <qmat/QMatAll>
#include <typeinfo>
#include <optional>
#include <type_traits>

#include <DSRGetID.h>


#include "../core/crdt/delta-crdts.cc"
#include "../core/rtps/dsrparticipant.h"
#include "../core/rtps/dsrpublisher.h"
#include "../core/rtps/dsrsubscriber.h"
#include "../core/topics/IDLGraphPubSubTypes.h"
#include "../core/types/crdt_types.h"
#include "../core/types/user_types.h"
#include "../core/types/translator.h"
#include "dsr_inner_api.h"
#include "dsr_utils.h"
#include "../../components/idserver/cmake-build-debug/src/DSRGetID.h"

#define NO_PARENT -1
#define TIMEOUT 5000

// Overload pattern used inprintVisitor
//template<class... Ts> struct overload : Ts... { using Ts::operator()...; };
//template<class... Ts> overload(Ts...) -> overload<Ts...>;

namespace DSR {

    //using N = CRDTNode;
    using Nodes = ormap<uint32_t , mvreg<CRDTNode, uint32_t>, uint32_t>;
    using IDType = std::uint32_t;


    class hash_tuple {

        template<class T>
        struct component {
            const T &value;

            component(const T &value) : value(value) {}

            uintmax_t operator,(uintmax_t n) const {
                n ^= std::hash<T>()(value);
                n ^= n << (sizeof(uintmax_t) * 4 - 1);
                return n ^ std::hash<uintmax_t>()(n);
            }
        };

    public:
        template<class Tuple>
        size_t operator()(const Tuple &tuple) const {
            return std::hash<uintmax_t>()(
                    std::apply([](const auto &... xs) { return (component(xs), ..., 0); }, tuple));
        }
    };


    template<typename Va>
    static bool constexpr allowed_types = std::is_same<std::int32_t, Va>::value ||
                                          std::is_same<std::uint32_t, Va>::value ||
                                          std::is_same<std::string_view, Va>::value ||
                                          std::is_same<std::string, Va>::value ||
                                          std::is_same<std::float_t, Va>::value ||
                                          std::is_same<std::double_t, Va>::value ||
                                          std::is_same<std::vector<float_t>, Va>::value ||
                                          std::is_same<std::vector<uint8_t>, Va>::value ||
                                          std::is_same<bool, Va>::value;
    template<typename Va>
    static bool constexpr any_node_or_edge = std::is_same<DSR::CRDTNode, Va>::value ||
                                         std::is_same<DSR::CRDTEdge, Va>::value ||
                                         std::is_same<DSR::Node, Va>::value ||
                                         std::is_same<DSR::Edge, Va>::value
                                         ;

    template<typename Va>
    static bool constexpr node_or_edge = std::is_same<DSR::Node, Va>::value ||
                                         std::is_same<DSR::Edge, Va>::value
    ;

    template<typename Va>
    static bool constexpr allowed_return_types = std::is_same<std::int32_t, Va>::value ||
                                                 std::is_same<std::uint32_t, Va>::value ||
                                                 std::is_same<std::string, Va>::value ||
                                                 std::is_same<std::float_t, Va>::value ||
                                                 std::is_same<std::vector<float_t>, Va>::value ||
                                                 std::is_same<std::vector<uint8_t>, Va>::value ||
                                                 std::is_same<bool, Va>::value ||
                                                 std::is_same<QVec, Va>::value ||
                                                 std::is_same<QMat, Va>::value;

    /////////////////////////////////////////////////////////////////
    /// CRDT API
    /////////////////////////////////////////////////////////////////

    class DSRGraph : public QObject {
    Q_OBJECT
    private:
        std::function<std::optional<uint8_t>(const Node&)> insert_node_read_file = [&] (const Node& node) -> std::optional<int> {
            bool r = false;
            {
                std::unique_lock<std::shared_mutex> lock(_mutex);
                if (id_map.find(node.id()) == id_map.end() and name_map.find(node.name())  == name_map.end()) {
                    std::tie(r, std::ignore) = insert_node_(user_node_to_crdt(node));
                } else throw std::runtime_error((std::string("Cannot insert node in G, a node with the same id already exists ")
                                                 + __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__)).data());
            }
            if (r) {
                return node.id();
            }
            return {};
        };


    public:
    public:
        size_t size();

        DSRGraph(int root, std::string name, int id, std::string dsr_input_file = std::string(), RoboCompDSRGetID::DSRGetIDPrxPtr dsr_getid_proxy_ = nullptr);

        ~DSRGraph();


        //////////////////////////////////////////////////////
        ///  Graph API
        //////////////////////////////////////////////////////

        // Utils
        bool empty(const uint32_t &id);

        template<typename Ta, typename = std::enable_if_t<allowed_types<std::remove_cv_t<std::remove_reference_t<Ta>>>>>
        std::tuple<std::string, std::string, int> nativetype_to_string(const Ta &t) {
            if constexpr (std::is_same<Ta, std::string>::value) {
                return make_tuple("string", t, 1);
            } else if constexpr (std::is_same<Ta, std::vector<float>>::value) {
                std::string str;
                for (auto &f : t)
                    str += std::to_string(f) + " ";
                return make_tuple("vector<float>", str += "\n", t.size());
            } else if constexpr (std::is_same<Ta, std::vector<uint8_t>>::value) {
                std::string str;
                for (auto &f : t)
                    str += std::to_string(f) + " ";
                return make_tuple("vector<byte>", str += "\n", t.size());
            } else if constexpr (std::is_same<Ta, std::vector<uint8_t>>::value)
            {
                std::string str;
                for(auto &f : t)
                    str += std::to_string(f) + " ";
                return make_tuple("vector<uint8_t>",  str += "\n",t.size());
            }
            else return make_tuple(typeid(Ta).name(), std::to_string(t), 1);

        }; //Used by viewer

        std::map<uint32_t, Node> getCopy() const;
        std::vector<uint32_t> getKeys() const;

        // Innermodel API
        std::unique_ptr<InnerAPI> get_inner_api() { return std::make_unique<InnerAPI>(this); };

        /**
         * CORE
         **/
        // Nodes
        std::optional<Node> get_node(const std::string &name);
        std::optional<Node> get_node(int id);
        std::optional<uint32_t> insert_node(Node &node);
        bool update_node(Node &node);
        bool delete_node(const std::string &name);
        bool delete_node(uint32_t id);

        // Edges
        bool insert_or_assign_edge(const Edge& attrs);
        std::optional<Edge> get_edge(const std::string& from, const std::string& to, const std::string& key);
        std::optional<Edge> get_edge(uint32_t from, uint32_t to, const std::string& key);
        std::optional<Edge> get_edge(const Node& n, const std::string& to, const std::string& key);
        std::optional<Edge> get_edge(const Node& n, uint32_t to, const std::string& key);
        bool delete_edge(const std::string& from, const std::string& t, const std::string& key);
        bool delete_edge(uint32_t from, uint32_t t, const std::string& key);


        /**
         * CONVENIENCE METHODS
         **/
        // Nodes
        std::optional<Node> get_node_root() { return get_node("world"); };  //CHANGE THIS
        std::vector<Node> get_nodes_by_type(const std::string &type);
        std::optional<std::string> get_name_from_id(uint32_t id);  // caché
        std::optional<std::uint32_t> get_id_from_name(const std::string &name);  // caché
        std::optional<std::int32_t> get_node_level(Node &n);
        std::optional<std::uint32_t> get_parent_id(const Node &n);
        std::optional<Node> get_parent_node(const Node &n);
        std::string get_node_type(Node &n);

        // Edges
        std::vector<Edge> get_edges_by_type(const std::string &type);
        std::vector<Edge> get_node_edges_by_type(const Node &node, const std::string &type);
        std::vector<Edge> get_edges_to_id(uint32_t id);
        std::optional<std::map<std::pair<uint32_t, std::string>, Edge/*, pair_hash*/>> get_edges(uint32_t id);


        // Attributes
        template<typename Ta, typename = std::enable_if_t<allowed_return_types<Ta>>, typename Type, typename =  std::enable_if_t<any_node_or_edge<Type>>>
        std::optional<Ta> get_attrib_by_name(const Type &n, const std::string &key) {
            std::optional<CRDTAttribute> av = get_attrib_by_name_(n, key);
            if (!av.has_value()) return {};
            if constexpr (std::is_same<Ta, std::string>::value) {
                return av->val().str();
            }
            if constexpr (std::is_same<Ta, std::int32_t>::value) {
                return av->val().dec();
            }
            if constexpr (std::is_same<Ta, float>::value) {
                return av->val().fl();
            }
            if constexpr (std::is_same<Ta, std::vector<float>>::value) {
                return av->val().float_vec();
            }
            if constexpr (std::is_same<Ta, bool>::value) {
                return av->val().bl();
            }
            if constexpr (std::is_same<Ta, std::vector<uint8_t>>::value) {
                return av->val().byte_vec();
            }
            if constexpr (std::is_same<Ta, std::uint32_t>::value) {
                return av->val().uint();
            }
            if constexpr (std::is_same<Ta, QVec>::value) {
                const auto &val = av->val().float_vec();
                if ((key == "translation" or key == "rotation_euler_xyz")
                    and (val.size() == 3 or val.size() == 6))
                    return QVec{val};
                throw std::runtime_error("vec size mut be 3 or 6 in get_attrib_by_name<QVec>()");
            }
            if constexpr (std::is_same<Ta, QMat>::value) {
                if (av->val().selected() == FLOAT_VEC and key == "rotation_euler_xyz") {
                    const auto &val = av->val().float_vec();
                    return QMat{RMat::Rot3DOX(val[0]) * RMat::Rot3DOY(val[1]) * RMat::Rot3DOZ(val[2])};
                }
                throw std::runtime_error("vec size mut be 3 or 6 in get_attrib_by_name<QVec>()");
            }  //else
            //throw std::runtime_error("Illegal return type in get_attrib_by_name()");
        }



        /**
         * LOCAL ATTRIBUTES MODIFICATION METHODS (for nodes and edges)
         **/
        template<typename Type, typename = std::enable_if_t<any_node_or_edge<Type>>, typename Ta, typename = std::enable_if_t<allowed_types<Ta>>>
        void add_or_modify_attrib_local(Type &elem, const std::string &att_name, const Ta &att_value) {

            if constexpr (std::is_same_v<Type, Node> || std::is_same_v<Type, Edge>) {
                Attribute at(att_value, get_unix_timestamp(), agent_id);
                elem.attrs().insert_or_assign(att_name, at);
            } else {
                CRDTAttribute at;
                CRDTValue value;
                if constexpr (std::is_same<std::string, Ta>::value || std::is_same<std::string_view, Ta>::value ||
                              std::is_same<const string &, Ta>::value) {
                    at.type(STRING);
                    value.str(att_value);
                } else if constexpr (std::is_same<std::int32_t, Ta>::value) {
                    at.type(INT);
                    value.dec(att_value);
                } else if constexpr (std::is_same<float, Ta>::value || std::is_same<double, Ta>::value) {
                    at.type(FLOAT);
                    value.fl(att_value);
                } else if constexpr (std::is_same<std::vector<float_t>, Ta>::value) {
                    at.type(FLOAT_VEC);
                    value.float_vec(att_value);
                } else if constexpr (std::is_same<std::vector<uint8_t>, Ta>::value) {
                    at.type(BYTE_VEC);
                    value.byte_vec(att_value);
                } else if constexpr (std::is_same<bool, Ta>::value) {
                    at.type(BOOL);
                    value.bl(att_value);
                } else if constexpr (std::is_same<std::uint32_t, Ta>::value) {
                    at.type(UINT);
                    value.uint(att_value);
                }

                at.val(std::move(value));
                at.timestamp(get_unix_timestamp());
                if (elem.attrs().find(att_name) == elem.attrs().end()) {
                    mvreg<CRDTAttribute, uint32_t> mv;
                    elem.attrs().insert(make_pair(att_name, mv));
                }
                elem.attrs()[att_name].write(at);
            }
        }

        template<typename Type, typename = std::enable_if_t<any_node_or_edge<Type>>, typename Ta, typename = std::enable_if_t<allowed_types<Ta>>>
        bool add_attrib_local(Type &elem, const std::string &att_name, const Ta &att_value) {
            if (elem.attrs().find(att_name) != elem.attrs().end()) return false;
            add_or_modify_attrib_local(elem, att_name, att_value);
            return true;
        };

        template<typename Type, typename = std::enable_if_t<any_node_or_edge<Type>>>
        bool add_attrib_local(Type &elem, const std::string &att_name, Attribute &attr) {
            if (elem.attrs().find(att_name) != elem.attrs().end()) return false;
            attr.timestamp(get_unix_timestamp());
            elem.attrs()[att_name] = attr;
            return true;
        };


        template<typename Type, typename = std::enable_if_t<any_node_or_edge<Type>>, typename Ta, typename = std::enable_if_t<allowed_types<Ta>>>
        bool modify_attrib_local(Type &elem, const std::string &att_name, const Ta &att_value) {
            if (elem.attrs().find(att_name) == elem.attrs().end()) return false;
            //throw DSRException(("Cannot update attribute. Attribute: " + att_name + " does not exist. " + __FUNCTION__).data());
            add_or_modify_attrib_local(elem, att_name, att_value);
            return true;
        };

        template<typename Type, typename = std::enable_if_t<any_node_or_edge<Type>>>
        bool modify_attrib_local(Type &elem, const std::string &att_name, CRDTAttribute &attr) {
            if (elem.attrs().find(att_name) == elem.attrs().end()) return false;
            //throw DSRException(("Cannot update attribute. Attribute: " + att_name + " does not exist. " + __FUNCTION__).data());
            attr.timestamp(get_unix_timestamp());
            elem.attrs()[att_name] = attr;
            return true;
        };


        template<typename Type, typename = std::enable_if_t<any_node_or_edge<Type>>>
        bool remove_attrib_local(Type &elem, const std::string &att_name) {
            if (elem.attrs().find(att_name) == elem.attrs().end()) return false;
            elem.attrs().erase(att_name);
            return true;
        }




        // ******************************************************************
        //  DISTRIBUTED ATTRIBUTE MODIFICATION METHODS (for nodes and edges)
        // ******************************************************************
        template<typename Ta, typename = std::enable_if_t<node_or_edge<Ta>>,
                typename Va, typename = std::enable_if_t<allowed_types<Va>>>
        void insert_or_assign_attrib_by_name(Ta &elem, const std::string &att_name, const Va &att_value) {

            add_or_modify_attrib_local(elem, att_name, att_value);

            // insert in node
            if constexpr (std::is_same<Node, Ta>::value) {
                if (update_node(elem))
                    return;
                else
                    throw std::runtime_error(
                            "Could not insert Node " + std::to_string(elem.id()) + " in G in add_attrib_by_name()");
            }
                // insert in edge
            else if constexpr (std::is_same<Edge, Ta>::value) {
                //auto node = get_node(elem.from());
                //if (node.has_value()) {
                    if (insert_or_assign_edge(elem))
                        return;
                    else
                        throw std::runtime_error("Could not insert Node " + std::to_string(elem.from()) +
                                                 " in G in add_attrib_by_name()");
                //} else
                //    throw std::runtime_error("Node " + std::to_string(elem.from()) + " not found in attrib_by_name()");
            }
            //else
            //    throw std::runtime_error("Node or Edge type not valid for add_attrib_by_name()");
        }

        template<typename Type, typename = std::enable_if_t<node_or_edge<Type>>,
                typename Va, typename = std::enable_if_t<allowed_types<Va>>>
        bool insert_attrib_by_name(Type &elem, const std::string &att_name, const Va &new_val) {
            //if (elem.attrs().find(new_name) != elem.attrs().end()) return false;
            //throw DSRException(("Cannot update attribute. Attribute: " + elem + " does not exist. " + __FUNCTION__).data());

            bool res = add_attrib_local(elem, att_name, new_val);
            if (!res) return false;
            // insert in node
            if constexpr (std::is_same<Node, Type>::value) {
                if (update_node(elem))
                    return true;
                else
                    throw std::runtime_error(
                            "Could not insert Node " + std::to_string(elem.id()) + " in G in add_attrib_by_name()");
            }
                // insert in edge
            else if constexpr (std::is_same<Edge, Type>::value) {
                auto node = get_node(elem.from());
                if (node.has_value()) {
                    if (insert_or_assign_edge(elem))
                        return true;
                    else
                        throw std::runtime_error("Could not insert Node " + std::to_string(elem.from()) +
                                                 " in G in add_attrib_by_name()");
                } else
                    throw std::runtime_error("Node " + std::to_string(elem.from()) + " not found in attrib_by_name()");
            }
            return false;
        }


        template<typename Type, typename = std::enable_if_t<node_or_edge<Type>>,
                typename Va, typename = std::enable_if_t<allowed_types<Va>>>
        bool update_attrib_by_name(Type &elem, const std::string &att_name, const Va &new_val) {
            //if (elem.attrs().find(new_name) == elem.attrs().end()) return false;
            //throw DSRException(("Cannot update attribute. Attribute: " + elem + " does not exist. " + __FUNCTION__).data());

            bool res = modify_attrib_local(elem, att_name, new_val);
            if (!res) return false;
            // insert in node
            if constexpr (std::is_same<Node, Type>::value) {
                if (update_node(elem))
                    return true;
                else
                    throw std::runtime_error(
                            "Could not insert Node " + std::to_string(elem.id()) + " in G in add_attrib_by_name()");
            }
                // insert in edge
            else if constexpr (std::is_same<Edge, Type>::value) {
                auto node = get_node(elem.from());
                if (node.has_value()) {
                    if (insert_or_assign_edge(elem))
                        return true;
                    else
                        throw std::runtime_error("Could not insert Node " + std::to_string(elem.from()) +
                                                 " in G in add_attrib_by_name()");
                } else
                    throw std::runtime_error("Node " + std::to_string(elem.from()) + " not found in attrib_by_name()");
            }
        }


        template<typename Type, typename = std::enable_if_t<node_or_edge<Type>>>
        bool remove_attrib_by_name(Type &elem, const std::string &att_name) {

            if (elem.attrs().find(att_name) == elem.attrs().end()) return false;
            elem.attrs().erase(att_name);

            // insert in node
            if constexpr (std::is_same<Node, Type>::value) {
                if (update_node(elem))
                    return true;
                else
                    throw std::runtime_error(
                            "Could not insert Node " + std::to_string(elem.id()) + " in G in remove_attrib_by_name()");
            }
                // insert in edge
            else if constexpr (std::is_same<Edge, Type>::value) {
                //auto node = get_node(elem.from());
                //if (node.has_value()) {
                    if (insert_or_assign_edge(elem))
                        return true;
                    else
                        throw std::runtime_error("Could not insert Node " + std::to_string(elem.from()) +
                                                 " in G in add_attrib_by_name()");
            } else
                    throw std::runtime_error(
                            "Node " + std::to_string(elem.from()) + " not found in remove_attrib_by_name()");
            //}
        }

        // Mixed
        inline uint32_t get_agent_id() const { return agent_id; };
        inline std::string get_agent_name() const { return agent_name; };
        void reset() {
            nodes.reset();
            deleted.clear();
            name_map.clear();
            id_map.clear();
            edges.clear();
            edgeType.clear();
            nodeType.clear();
        }

        /////////////////////////////////////////////////
        /// AUXILIARY RT SUB-API
        /////////////////////////////////////////////////

        void insert_or_assign_edge_RT(Node &n, uint32_t to, const std::vector<float> &trans, const std::vector<float> &rot_euler);
        void insert_or_assign_edge_RT(Node &n, uint32_t to, std::vector<float> &&trans, std::vector<float> &&rot_euler);
        Edge get_edge_RT(const Node &n, uint32_t to);
        RTMat get_edge_RT_as_RTMat(const Edge &edge);
        RTMat get_edge_RT_as_RTMat(Edge &&edge);


        /////////////////////////////////////////////////
        /// AUXILIARY IO SUB-API
        /////////////////////////////////////////////////
        void print() { utils->print(); };
        void print_edge(const Edge &edge) { utils->print_edge(edge); };
        void print_node(const Node &node) { utils->print_node(node); };
        void print_node(uint32_t id) { utils->print_node(id); };
        void print_RT(uint32_t root) const { utils->print_RT(root); };
        void write_to_json_file(const std::string &file) const { utils->write_to_json_file(file); };
        void read_from_json_file(const std::string &file) const { utils->read_from_json_file(file, insert_node_read_file); };

        //////////////////////////////////////////////////
        ///// PRIVATE COPY
        /////////////////////////////////////////////////
        DSRGraph G_copy();
        bool is_copy();







    private:

        DSRGraph(const DSRGraph& G);

        Nodes nodes;
        int graph_root;
        bool work;
        mutable std::shared_mutex _mutex;
        std::string filter;
        const uint32_t agent_id;
        std::string agent_name;
        std::unique_ptr<Utilities> utils;
        RoboCompDSRGetID::DSRGetIDPrxPtr dsr_getid_proxy; // proxy to obtain unique node ids
        const bool copy;


        //////////////////////////////////////////////////////////////////////////
        // Cache maps
        ///////////////////////////////////////////////////////////////////////////
        std::unordered_map<uint32_t, std::unordered_map<std::string, mvreg<CRDTAttribute, uint32_t>>> temp_node_attr; //temporal storage for attributes to solve problems with unsorted messages.
        std::unordered_map<std::tuple<uint32_t, uint32_t, std::string>, std::unordered_map<std::string, mvreg<CRDTAttribute, uint32_t>>, hash_tuple> temp_edge_attr; //temporal storage for attributes to solve problems with unsorted messages.
        std::unordered_map<int, std::unordered_map<std::tuple<uint32_t, uint32_t, std::string>, mvreg<CRDTEdge, uint32_t>, hash_tuple>> temp_edge; //temporal storage for attributes to solve problems with unsorted messages.

        std::unordered_set<uint32_t> deleted;     // deleted nodes, used to avoid insertion after remove.
        std::unordered_map<string, uint32_t> name_map;     // mapping between name and id of nodes.
        std::unordered_map<uint32_t, string> id_map;       // mapping between id and name of nodes.
        std::unordered_map<pair<uint32_t, uint32_t>, std::unordered_set<std::string>, hash_tuple> edges;      // collection with all graph edges. ((from, to), key)
        std::unordered_map<std::string, std::unordered_set<pair<uint32_t, uint32_t>, hash_tuple>> edgeType;  // collection with all edge types.
        std::unordered_map<std::string, std::unordered_set<uint32_t>> nodeType;  // collection with all node types.

        void update_maps_node_delete(uint32_t id, const CRDTNode &n);
        void update_maps_node_insert(uint32_t id, const CRDTNode &n);
        void update_maps_edge_delete(uint32_t from, uint32_t to, const std::string &key);
        void update_maps_edge_insert(uint32_t from, uint32_t to, const std::string &key);


        //////////////////////////////////////////////////////////////////////////
        // Non-blocking graph operations
        //////////////////////////////////////////////////////////////////////////
        std::optional<CRDTNode> get(uint32_t id);
        bool in(uint32_t id) const;
        std::optional<CRDTNode> get_(uint32_t id);
        std::optional<CRDTEdge> get_edge_(uint32_t from, uint32_t to, const std::string &key);
        std::tuple<bool, std::optional<IDL::Mvreg>> insert_node_(const CRDTNode &node);
        std::tuple<bool, std::optional<std::vector<IDL::MvregNodeAttr>>> update_node_(const CRDTNode &node);
        std::tuple<bool, vector<tuple<uint32_t, uint32_t, std::string>>, std::optional<IDL::Mvreg>, vector<IDL::MvregEdge>> delete_node_(uint32_t id);
        std::optional<IDL::MvregEdge> delete_edge_(uint32_t from, uint32_t t, const std::string &key);
        std::tuple<bool, std::optional<IDL::MvregEdge>, std::optional<std::vector<IDL::MvregEdgeAttr>>> insert_or_assign_edge_(const CRDTEdge &attrs, uint32_t from, uint32_t to);

        //////////////////////////////////////////////////////////////////////////
        // Other methods
        //////////////////////////////////////////////////////////////////////////
        uint32_t id();
        IDL::DotContext context();
        std::map<uint32_t, IDL::Mvreg> Map();


        static uint64_t get_unix_timestamp(const std::time_t *t = nullptr) {
            //if specific time is not passed then get current time
            std::time_t st = t == nullptr ? std::time(nullptr) : *t;
            auto secs = static_cast<std::chrono::seconds>(st).count();
            return static_cast<uint64_t>(secs);
        }

        template<typename T, typename = std::enable_if_t<any_node_or_edge<T>>>
        std::optional<CRDTAttribute> get_attrib_by_name_(const T &n, const std::string &key) {
            auto attrs = n.attrs();
            auto value = attrs.find(key);
            if (value != attrs.end()) {
                if constexpr(std::is_same_v<T, Node> || std::is_same_v<T, Edge>)
                    return user_attribute_to_crdt(value->second);
                else
                    return value->second.read_reg();
            } else {
                if constexpr (std::is_same<CRDTNode, T>::value)
                    std::cout << "ERROR: " << __FUNCTION__ << ":" << __LINE__ << " "
                              << "Attribute " << key << " not found in node  -> " << n.id() << std::endl;
                if constexpr (std::is_same<CRDTEdge, T>::value)
                    std::cout << "ERROR: " << __FUNCTION__ << ":" << __LINE__ << " "
                              << "Atribute " << key << " not found in edge -> " << n.to() << std::endl;
            }
            return {};
        }

        //////////////////////////////////////////////////////////////////////////
        // CRDT join operations
        ///////////////////////////////////////////////////////////////////////////
        void join_delta_node(IDL::Mvreg &mvreg);
        void join_delta_edge(IDL::MvregEdge &mvreg);
        void join_delta_node_attr(IDL::MvregNodeAttr &mvreg);
        void join_delta_edge_attr(IDL::MvregEdgeAttr &mvreg);
        void join_full_graph(IDL::OrMap &full_graph);

        //Custom function for each rtps topic
        class NewMessageFunctor {
        public:
            DSRGraph *graph{};
            bool *work{};
            std::function<void(eprosima::fastrtps::Subscriber *sub, bool *work, DSR::DSRGraph *graph)> f;

            NewMessageFunctor(DSRGraph *graph_, bool *work_,
                              std::function<void(eprosima::fastrtps::Subscriber *sub, bool *work,
                                                 DSR::DSRGraph *graph)> f_)
                    : graph(graph_), work(work_), f(std::move(f_)) {}

            NewMessageFunctor() = default;


            void operator()(eprosima::fastrtps::Subscriber *sub) const { f(sub, work, graph); };
        };


        //Threads
        bool start_fullgraph_request_thread();
        void start_fullgraph_server_thread();
        void start_subscription_threads(bool showReceived);

        std::thread delta_node_thread, delta_edge_thread, delta_node_attrs_thread, delta_edge_attrs_thread, fullgraph_thread;

        // Threads handlers
        void node_subscription_thread(bool showReceived);
        void edge_subscription_thread(bool showReceived);
        void node_attrs_subscription_thread(bool showReceived);
        void edge_attrs_subscription_thread(bool showReceived);
        void fullgraph_server_thread();
        bool fullgraph_request_thread();


        // RTSP participant
        DSRParticipant dsrparticipant;
        DSRPublisher dsrpub_node;
        DSRSubscriber dsrsub_node;
        NewMessageFunctor dsrpub_call_node;


        DSRPublisher dsrpub_edge;
        DSRSubscriber dsrsub_edge;
        NewMessageFunctor dsrpub_call_edge;

        DSRPublisher dsrpub_node_attrs;
        DSRSubscriber dsrsub_node_attrs;
        NewMessageFunctor dsrpub_call_node_attrs;

        DSRPublisher dsrpub_edge_attrs;
        DSRSubscriber dsrsub_edge_attrs;
        NewMessageFunctor dsrpub_call_edge_attrs;

        DSRSubscriber dsrsub_graph_request;
        DSRPublisher dsrpub_graph_request;
        NewMessageFunctor dsrpub_graph_request_call;

        DSRSubscriber dsrsub_request_answer;
        DSRPublisher dsrpub_request_answer;
        NewMessageFunctor dsrpub_request_answer_call;

    signals:                                                                  // for graphics update
        void update_node_signal(std::uint32_t, const std::string &type); // REMOVE type

        void update_attrs_signal(std::uint32_t id,
                                 const std::map<string, Attribute> &attribs); //Signal to show node attribs.
        void update_edge_signal(std::uint32_t from, std::uint32_t to,
                                const std::string &type);                   // Signal to show edge attribs.

        void del_edge_signal(std::uint32_t from, std::uint32_t to,
                             const std::string &edge_tag); // Signal to del edge.
        void del_node_signal(
                std::uint32_t from);                                                     // Signal to del node.

    };
} // namespace CRDT

#endif
