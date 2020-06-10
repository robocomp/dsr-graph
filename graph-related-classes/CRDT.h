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

#include "libs/delta-crdts.cc"
#include "fast_rtps/dsrparticipant.h"
#include "fast_rtps/dsrpublisher.h"
#include "fast_rtps/dsrsubscriber.h"
#include "topics/DSRGraphPubSubTypes.h"
#include "CRDT_types.h"
#include "inner_api.h"
#include "dsr_utils.h"

#define NO_PARENT -1
#define TIMEOUT 5000

// Overload pattern used inprintVisitor
//template<class... Ts> struct overload : Ts... { using Ts::operator()...; };
//template<class... Ts> overload(Ts...) -> overload<Ts...>;

namespace CRDT
{
    using N = CRDT::Node;
    using Nodes = ormap<int, mvreg<CRDT::Node,  int >, int>;
    using IDType = std::int32_t;



    class hash_tuple {

        template<class T>
        struct component {
            const T& value;
            component(const T& value) : value(value) {}
            uintmax_t operator,(uintmax_t n) const {
                n ^= std::hash<T>()(value);
                n ^= n << (sizeof(uintmax_t) * 4 - 1);
                return n ^ std::hash<uintmax_t>()(n);
            }
        };

    public:
        template<class Tuple>
        size_t operator()(const Tuple& tuple) const {
            return std::hash<uintmax_t>()(
                    std::apply([](const auto& ... xs) { return (component(xs), ..., 0); }, tuple));
        }
    };


    template <typename Va>
    static bool constexpr allowed_types = std::is_same<std::int32_t, Va>::value ||
                                          std::is_same<std::string_view, Va>::value ||
                                          std::is_same<std::string, Va>::value ||
                                          std::is_same<std::float_t, Va>::value ||
                                          std::is_same<std::double_t , Va>::value ||
                                          std::is_same<std::vector<float_t>, Va>::value ||
                                          std::is_same<std::vector<byte>, Va>::value ||
                                          std::is_same<bool, Va>::value;
    template <typename Va>
    static bool constexpr node_or_edge = std::is_same<CRDT::Node, Va>::value ||
                                         std::is_same<CRDT::Edge, Va>::value ;

    template <typename Va>
    static bool constexpr allowed_return_types = std::is_same<std::int32_t, Va>::value ||
                                          std::is_same<std::string, Va>::value ||
                                          std::is_same<std::float_t, Va>::value ||
                                          std::is_same<std::vector<float_t>, Va>::value ||
                                          std::is_same<std::vector<byte>, Va>::value ||
                                          std::is_same<bool, Va>::value ||
                                          std::is_same<QVec, Va>::value ||
                                          std::is_same<QMat, Va>::value;

    /////////////////////////////////////////////////////////////////
    /// CRDT API
    /////////////////////////////////////////////////////////////////

    class CRDTGraph : public QObject
    {
        Q_OBJECT
        public:
        size_t size();
        CRDTGraph(int root, std::string name, int id, std::string dsr_input_file = std::string());
        ~CRDTGraph();


        //////////////////////////////////////////////////////
        ///  Graph API
        //////////////////////////////////////////////////////

        // Utils
        bool empty(const int &id);
        template <typename Ta, typename = std::enable_if_t<allowed_types<Ta>>>
        std::tuple<std::string, std::string, int> nativetype_to_string(const Ta& t) {
            if constexpr (std::is_same<Ta, std::string>::value)
            {
                return  make_tuple("string", t,1);
            }
            if constexpr (std::is_same<Ta, std::vector<float>>::value)
            {
                std::string str;
                for(auto &f : t)
                    str += std::to_string(f) + " ";
                return make_tuple("vector<float>",  str += "\n",t.size());
            }
            if constexpr (std::is_same<Ta, std::vector<byte>>::value)
            {
                std::string str;
                for(auto &f : t)
                    str += std::to_string(f) + " ";
                return make_tuple("vector<byte>",  str += "\n",t.size());
            }
            return  make_tuple(typeid(Ta).name(), std::to_string(t),1);

        }; //Used by viewer

        std::map<long, Node> getCopy() const;
        std::vector<long> getKeys() const ;   
        inline int32_t get_agent_id() const                     { return agent_id; };
        inline std::string get_agent_name() const               { return agent_name; };
        void print() { utils->print(); };
        void print_edge(const Edge &edge)                       { utils->print_edge(edge); };
        void print_node(const Node &node)                       { utils->print_node(node);} ;
        void print_node(int id) { utils->print_node(id);};
        void print_RT(std::int32_t root) const                  { utils->print_RT(root);};
        void write_to_json_file(const std::string &file) const  { utils->write_to_json_file(file); };
        void read_from_json_file(const std::string &file) const { utils->read_from_json_file(file); };

        // not working yet
        typename std::map<int, mvreg<Node,int>>::const_iterator begin() const { return nodes.getMap().begin(); };
        typename std::map<int, mvreg<Node,int>>::const_iterator end() const { return nodes.getMap().end(); };

        // Innermodel API
        std::unique_ptr<InnerAPI> get_inner_api() { return std::make_unique<InnerAPI>(this); };

        // Nodes
        std::optional<Node> get_node_root()  { return get_node(100); };  //CHANGE THIS
        std::optional<Node> get_node(const std::string& name);
        std::optional<Node> get_node(int id);
        //[[deprecated ("You should be using \"insert_node\" to insert new nodes and \"update_node\" to update them")]]
        //bool insert_or_assign_node(Node &node);
        std::optional<uint32_t> insert_node(Node& node);
        bool update_node(Node& node);
        bool delete_node(const std::string &name);
        bool delete_node(int id);
        std::vector<Node> get_nodes_by_type(const std::string& type);
        std::optional<std::string> get_name_from_id(std::int32_t id);  // caché
        std::optional<int> get_id_from_name(const std::string &name);  // caché
        std::optional<std::int32_t> get_node_level(Node& n);
        std::optional<std::int32_t> get_parent_id(const Node& n);
        std::optional<Node> get_parent_node(const Node& n);
        std::string get_node_type(Node& n);

        template <typename Type, typename = std::enable_if_t<node_or_edge<Type>>, typename Ta , typename = std::enable_if_t<allowed_types<Ta>>>
        void add_or_modify(Type &elem, const std::string& att_name, const Ta& att_value) {


            Attribute at;  Value value;
            if constexpr (std::is_same<std::string,  Ta>::value || std::is_same<std::string_view,  Ta>::value || std::is_same<const string&,  Ta>::value)
            {
                at.type(STRING);
                value.str(att_value);
            }
            else if constexpr (std::is_same<std::int32_t,  Ta>::value)
            {
                at.type(INT);
                value.dec(att_value);
            }
            else if constexpr (std::is_same<float,  Ta>::value || std::is_same<double,  Ta>::value)
            {
                at.type(FLOAT);
                value.fl(att_value);
            }
            else if constexpr (std::is_same<std::vector<float_t>,  Ta>::value)
            {
                at.type(FLOAT_VEC);
                value.float_vec(att_value);
            }
            else if constexpr (std::is_same<bool,  Ta>::value)
            {
                at.type(BOOL);
                value.bl(att_value);
            }

            at.val( std::move(value));
            elem.attrs()[att_name] = at;

        }

        template <typename Type, typename = std::enable_if_t<node_or_edge<Type>>, typename Ta , typename = std::enable_if_t<allowed_types<Ta>>>
         bool add_attrib(Type &elem, const std::string& att_name, const Ta& att_value) {
            if (elem.attrs().find(att_name) != elem.attrs().end()) return false;
            add_or_modify(elem, att_name, att_value);
            return true;
        };

        template <typename Type, typename = std::enable_if_t<node_or_edge<Type>>>
         bool add_attrib(Type &elem, const std::string& att_name, const Attribute &attr) {
            if (elem.attrs().find(att_name) != elem.attrs().end()) return false;
            elem.attrs()[att_name] = attr;
            return true;
        };


        template <typename Type, typename = std::enable_if_t<node_or_edge<Type>>, typename Ta , typename = std::enable_if_t<allowed_types<Ta>>>
         bool modify_attrib(Type &elem, const std::string& att_name, const Ta& att_value) {
            if (elem.attrs().find(att_name) == elem.attrs().end()) return false;
                //throw DSRException(("Cannot update attribute. Attribute: " + att_name + " does not exist. " + __FUNCTION__).data());
            add_or_modify(elem, att_name, att_value);
            return true;
        };

        template <typename Type, typename = std::enable_if_t<node_or_edge<Type>>>
         bool modify_attrib(Type &elem, const std::string& att_name, const Attribute &attr) {
            if (elem.attrs().find(att_name) == elem.attrs().end()) return false;
                //throw DSRException(("Cannot update attribute. Attribute: " + att_name + " does not exist. " + __FUNCTION__).data());
            elem.attrs()[att_name] = attr;
            return true;
        };


        template <typename Type, typename = std::enable_if_t<node_or_edge<Type>>>
        bool remove_attrib(Type& elem, const std::string &att_name) {
            if (elem.attrs().find(att_name) == elem.attrs().end()) return false;
            elem.attrs().erase(att_name);
            return true;
        }

        // Edges
        std::optional<Edge> get_edge(const std::string& from, const std::string& to, const std::string& key);
        std::optional<Edge> get_edge(int from, int to, const std::string& key);
        std::optional<Edge> get_edge(const Node &n, int to, const std::string& key) {
            EdgeKey ek; ek.to(to); ek.type(key);
            return (n.fano().getMapRef().find(make_pair{to, key}) != n.fano().end()) ?  std::make_optional(n.fano().find(ek)->second) : std::nullopt;
        };
        bool insert_or_assign_edge(const Edge& attrs);
        void insert_or_assign_edge_RT(Node& n, int to, const std::vector<float>& trans, const std::vector<float>& rot_euler);
        void insert_or_assign_edge_RT(Node& n, int to, std::vector<float>&& trans, std::vector<float>&& rot_euler);
        bool delete_edge(const std::string& from, const std::string& t, const std::string& key);
        bool delete_edge(int from, int t, const std::string& key);
        std::vector<Edge> get_edges_by_type(const std::string& type);
        std::vector<Edge> get_edges_by_type(const Node& node, const std::string& type);
        std::vector<Edge> get_edges_to_id(int id);
        std::optional<std::unordered_map<std::pair<int, std::string>, mvreg<Edge, int>,pair_hash>>  get_edges(int id); //TODO: Cambiar con la nueva representación.
        Edge get_edge_RT(const Node &n, int to);
        RTMat get_edge_RT_as_RTMat(const Edge &edge);
        RTMat get_edge_RT_as_RTMat(Edge &&edge);
        
        
        // Both
        template <typename Ta, typename = std::enable_if_t<allowed_return_types<Ta>>, typename Type, typename =  std::enable_if_t<node_or_edge<Type>>>
        std::optional<Ta> get_attrib_by_name(const Type& n, const std::string &key) 
        {
            std::optional<Attrib> av = get_attrib_by_name_(n, key);
            if (!av.has_value()) return {};
            if constexpr (std::is_same<Ta, std::string>::value)
            {
                return  av->value().str();
            }
            if constexpr (std::is_same<Ta, std::int32_t>::value)
            {
                return av->value().dec();
            }
            if constexpr (std::is_same<Ta, float>::value) 
            {
                return av->value().fl();
            }
            if constexpr (std::is_same<Ta, std::vector<float>>::value)
            {
                return av->value().float_vec();
            }
            if constexpr (std::is_same<Ta, bool>::value)
            {
                return av.value().value().bl();
            }
            if constexpr (std::is_same<Ta, QVec>::value)
            {
                const auto &val = av.value().value().float_vec();
                if ((key == "translation" or key == "rotation_euler_xyz")
                    and (val.size() == 3 or val.size() == 6))
                    return QVec{val};
                throw std::runtime_error("vec size mut be 3 or 6 in get_attrib_by_name<QVec>()");
            }
            if constexpr (std::is_same<Ta, QMat>::value)
            {
                if (av.value().value()._d() == FLOAT_VEC and key=="rotation_euler_xyz")
                {
                    const auto& val = av.value().value().float_vec();
                    return QMat{RMat::Rot3DOX(val[0])*RMat::Rot3DOY(val[1])*RMat::Rot3DOZ(val[2])};
                }
                throw std::runtime_error("vec size mut be 3 or 6 in get_attrib_by_name<QVec>()");
            }  //else
            //throw std::runtime_error("Illegal return type in get_attrib_by_name()");
        }

        template <typename Type, typename = std::enable_if_t<node_or_edge<Type>>,
                typename Va, typename = std::enable_if_t<allowed_types<Va>>>
        void insert_or_assign_attrib_by_name(Type& elem, const std::string &att_name, const Va &new_val)
        {
            add_or_modify(elem, att_name, new_val);

            // insert in node
            if constexpr (std::is_same<Node,  Type>::value)
            {
                if(update_node(elem))
                    return;
                else
                    throw std::runtime_error("Could not insert Node " + std::to_string(elem.id()) + " in G in add_attrib_by_name()");
            }
                // insert in edge
            else if constexpr (std::is_same<Edge,  Type>::value)
            {
                auto node = get_node(elem.from());
                if(node.has_value())
                {
                    if(insert_or_assign_edge(elem))
                        return;
                    else
                        throw std::runtime_error("Could not insert Node " + std::to_string(elem.from()) + " in G in add_attrib_by_name()");
                }
                else
                    throw std::runtime_error("Node " + std::to_string(elem.from()) + " not found in attrib_by_name()");
            }
            //else
            //    throw std::runtime_error("Node or Edge type not valid for add_attrib_by_name()");
        }

        template <typename Type, typename = std::enable_if_t<node_or_edge<Type>>,
                  typename Va, typename = std::enable_if_t<allowed_types<Va>>>
         bool insert_attrib_by_name(Type& elem, const std::string &att_name, const Va &new_val)
        {
            //if (elem.attrs().find(new_name) != elem.attrs().end()) return false;
                //throw DSRException(("Cannot update attribute. Attribute: " + elem + " does not exist. " + __FUNCTION__).data());

            bool res = add_attrib(elem, att_name, new_val);
            if (!res) return false;
            // insert in node 
            if constexpr (std::is_same<Node,  Type>::value)
            {
                if(update_node(elem))
                    return true;
                else
                    throw std::runtime_error("Could not insert Node " + std::to_string(elem.id()) + " in G in add_attrib_by_name()");
            }
            // insert in edge
            else if constexpr (std::is_same<Edge,  Type>::value)
            {
                auto node = get_node(elem.from());
                if(node.has_value())
                {
                    if(insert_or_assign_edge(elem))
                        return true;
                    else
                        throw std::runtime_error("Could not insert Node " + std::to_string(elem.from()) + " in G in add_attrib_by_name()");
                }
                else 
                    throw std::runtime_error("Node " + std::to_string(elem.from()) + " not found in attrib_by_name()");
            }
        }


        template <typename Type, typename = std::enable_if_t<node_or_edge<Type>>,
                typename Va, typename = std::enable_if_t<allowed_types<Va>>>
        bool update_attrib_by_name(Type& elem, const std::string &att_name, const Va &new_val)
        {
            //if (elem.attrs().find(new_name) == elem.attrs().end()) return false;
                //throw DSRException(("Cannot update attribute. Attribute: " + elem + " does not exist. " + __FUNCTION__).data());

            bool res = modify_attrib(elem, att_name, new_val);
            if (!res) return false;
            // insert in node
            if constexpr (std::is_same<Node,  Type>::value)
            {
                if(update_node(elem))
                    return true;
                else
                    throw std::runtime_error("Could not insert Node " + std::to_string(elem.id()) + " in G in add_attrib_by_name()");
            }
                // insert in edge
            else if constexpr (std::is_same<Edge,  Type>::value)
            {
                auto node = get_node(elem.from());
                if(node.has_value())
                {
                    if(insert_or_assign_edge(elem))
                        return true;
                    else
                        throw std::runtime_error("Could not insert Node " + std::to_string(elem.from()) + " in G in add_attrib_by_name()");
                }
                else
                    throw std::runtime_error("Node " + std::to_string(elem.from()) + " not found in attrib_by_name()");
            }
        }


        template <typename Type, typename = std::enable_if_t<node_or_edge<Type>>>
        bool remove_attrib_by_name(Type& elem, const std::string &att_name) {

            if (elem.attrs().find(att_name) == elem.attrs().end()) return false;
            elem.attrs().erase(att_name);

            // insert in node
            if constexpr (std::is_same<Node,  Type>::value)
            {
                if(update_node(elem))
                    return true;
                else
                    throw std::runtime_error("Could not insert Node " + std::to_string(elem.id()) + " in G in remove_attrib_by_name()");
            }
                // insert in edge
            else if constexpr (std::is_same<Edge,  Type>::value)
            {
                auto node = get_node(elem.from());
                if(node.has_value())
                {
                    bool r = insert_or_assign_edge(elem);
                    if(r)
                        return true;
                    else
                        throw std::runtime_error("Could not insert Node " + std::to_string(elem.from()) + " in G in add_attrib_by_name()");
                }
                else
                    throw std::runtime_error("Node " + std::to_string(elem.from()) + " not found in remove_attrib_by_name()");
            }
        }

        void reset() {
            nodes.reset();
            deleted.clear();
            name_map.clear();
            id_map.clear();
            edges.clear();
            edgeType.clear();
            nodeType.clear();
        }

    
    private:
        Nodes nodes;

        int graph_root;
        bool work;
        mutable std::shared_mutex _mutex;
        std::string filter;
        const int agent_id;
        std::string agent_name;
        std::unique_ptr<Utilities> utils;


        //////////////////////////////////////////////////////////////////////////
        // Cache maps
        ///////////////////////////////////////////////////////////////////////////
        std::unordered_map<int, Attribute> temp_node_attr; //temporal storage for attributes to solve problems with unsorted messages.
        std::unordered_map<std::tuple<int, int, std::string>, Attribute, hash_tuple> temp_node_attr; //temporal storage for attributes to solve problems with unsorted messages.

        std::unordered_set<int> deleted;     // deleted nodes, used to avoid insertion after remove.
        std::unordered_map<string, int> name_map;     // mapping between name and id of nodes.
        std::unordered_map<int, string> id_map;       // mapping between id and name of nodes.
        std::unordered_map<pair<int, int>, std::unordered_set<std::string>, hash_tuple> edges;      // collection with all graph edges. ((from, to), key)
        std::unordered_map<std::string, std::unordered_set<pair<int, int>, hash_tuple>> edgeType;  // collection with all edge types.
        std::unordered_map<std::string, std::unordered_set<int>> nodeType;  // collection with all node types.
        void update_maps_node_delete(int id, const Node& n);
        void update_maps_node_insert(int id, const Node& n);
        void update_maps_edge_delete(int from, int to, const std::string& key);




        //////////////////////////////////////////////////////////////////////////
        // Non-blocking graph operations
        //////////////////////////////////////////////////////////////////////////
        std::optional<Node> get(int id);
        bool in(const int &id) const;
        std::optional<Node> get_(int id);
        std::optional<Edge> get_edge_(int from, int to, const std::string& key);
        std::tuple<bool, std::optional<IDL::Mvreg>, std::optional<std::vector<IDL::MvregNodeAttr>>> insert_or_assign_node_(const N &node);
        std::tuple<bool, vector<tuple<int, int, std::string>>, IDL::Mvreg, vector<IDL::MvregEdge>> delete_node_(int id);
        std::pair<bool, std::optional<IDL::MvregEdge>> delete_edge_(int from, int t, const std::string& key);
        std::tuple<bool, std::optional<IDL::MvregEdge>, std::optional<std::vector<IDL::MvregEdgeAttr>>> insert_or_assign_edge_(const Edge &attrs);

        //////////////////////////////////////////////////////////////////////////
        // Other methods
        //////////////////////////////////////////////////////////////////////////
        int id();
        IDL::DotContext context();
        std::map<int, IDL::Mvreg> Map();


        template <typename T, typename = std::enable_if_t<node_or_edge<T>>>
        std::optional<Attrib> get_attrib_by_name_(const T& n, const std::string &key)
        {
            auto attrs = n.attrs();
            auto value  = attrs.find(key);
            if (value != attrs.end())
                return value->second;
            else
            {
                if constexpr (std::is_same<Node,  T>::value)
                    std::cout << "ERROR: " <<  __FUNCTION__ << ":" << __LINE__ << " "
                              << "Attribute " << key << " not found in node  -> " << n.id() << std::endl;
                if constexpr (std::is_same<Edge,  T>::value)
                    std::cout << "ERROR: " <<  __FUNCTION__ << ":" << __LINE__ << " "
                              << "Atribute " << key << " not found in edge -> " << n.to() << std::endl;
            }
            return {};
        }

        void join_delta_node(Mvreg &mvreg);
        void join_delta_edge(MvregEdge &mvreg);
        void join_delta_node_attr(MvregNodeAttr &mvreg);
        void join_delta_edge_attr(MvregEdgeAttr &mvreg);
        void join_full_graph(OrMap &full_graph);

        class NewMessageFunctor
        {
            public:
                CRDTGraph *graph{};
                bool *work{};
                std::function<void(eprosima::fastrtps::Subscriber *sub, bool *work, CRDT::CRDTGraph *graph)> f;

                NewMessageFunctor(CRDTGraph *graph_, bool *work_,
                                std::function<void(eprosima::fastrtps::Subscriber *sub, bool *work, CRDT::CRDTGraph *graph)> f_)
                        : graph(graph_), work(work_), f(std::move(f_)){}

                NewMessageFunctor() = default;


                void operator()(eprosima::fastrtps::Subscriber *sub) { f(sub, work, graph); };
        };


        //Threads
        // threads
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


        // Translators
        IDL::Mvreg translateNodeMvCRDTtoIDL(int id, mvreg<Node, int> &data);
        mvreg<N, int> translateNodeMvIDLtoCRDT(IDL::Mvreg &data);

        IDL::MvregEdge translateEdgeMvCRDTtoIDL(int id, mvreg<Edge, int> &data);
        mvreg<Edge, int> translateEdgeMvIDLtoCRDT(IDL::MvregEdge &data);

        IDL::MvregNodeAttr translateNodeAttrMvCRDTtoIDL(int id, mvreg<Attribute, int> &data);
        mvreg<Attribute, int> translateNodeAttrMvIDLtoCRDT(IDL::MvregNodeAttr &data);

        IDL::MvregEdgeAttr translateEdgeAttrMvCRDTtoIDL(int id, mvreg<Attribute, int> &data);
        mvreg<Attribute, int> translateEdgeAttrMvIDLtoCRDT(IDL::MvregEdgeAttr &data);

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
        void update_node_signal(const std::int32_t, const std::string &type); // REMOVE type

        void update_attrs_signal(const std::int32_t &id, const std::map<string, Attrib> &attribs); //Signal to show node attribs.
        void update_edge_signal(const std::int32_t from, const std::int32_t to, const std::string& type);                   // Signal to show edge attribs.

        void del_edge_signal(const std::int32_t from, const std::int32_t to, const std::string &edge_tag); // Signal to del edge.
        void del_node_signal(const std::int32_t from);                                                     // Signal to del node.

    };
} // namespace CRDT

#endif
