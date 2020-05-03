//
// Created by crivac on 17/01/19.
//

#ifndef CRDT_GRAPH
#define CRDT_GRAPH

#include <iostream>
#include <map>
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
#include <iter/zip.hpp>

#include "libs/delta-crdts.cc"
#include "fast_rtps/dsrparticipant.h"
#include "fast_rtps/dsrpublisher.h"
#include "fast_rtps/dsrsubscriber.h"
#include "topics/DSRGraphPubSubTypes.h"



#define NO_PARENT -1
#define TIMEOUT 5

// Overload pattern used inprintVisitor
template<class... Ts> struct overload : Ts... { using Ts::operator()...; };
template<class... Ts> overload(Ts...) -> overload<Ts...>;

namespace CRDT
{
    using N = Node;
    using Nodes = ormap<int, aworset<N,  int >, int>;
    using MTypes = std::variant<std::uint32_t, std::int32_t, float, std::string, std::vector<float>, RMat::RTMat>;
    using IDType = std::int32_t;
    using Attribs = std::unordered_map<std::string, MTypes>;

    /////////////////////////////////////////////////////////////////
    /// DSR Exceptions
    /////////////////////////////////////////////////////////////////
    class DSRException : std::runtime_error
    {
        public:
            explicit DSRException(const std::string &message): std::runtime_error(buildMsg(message)){};
        private:
        std::string buildMsg(const std::string &message)
        {
            std::ostringstream buffer;
            buffer << "DSRException: " << message;
            return buffer.str();
        };
    };

    /////////////////////////////////////////////////////////////////
    /// Wrapper for Node struct to include nice access API
    /////////////////////////////////////////////////////////////////
    // // New API proposal
	// auto [node, success] = G->getNode("base");
	
	// node->addOrAssignAttrib("id", val);
	// auto att = node->getAttrib("id");
	// auto atts = node->getAttribsByType("type");
	// auto edge = node->getEdge(key(to, "type"))
	// node->removeAttrib("id")
	// node->removeEdge(key(to, "type"))
	// RTMat node->getEdgeRTAttrib(to)

	// auto eatt = edge->getAttrib("name");
	// edge->addOrAssignAttrib("name", val);
	// auto eatt = edge->removeAttrib("name");
	// auto eatts = edge->getAttribsByType("type");
	// RTMat edge->getRTAttrib();
	
	// G->insertOrAssignNode(node);
	// G->removeNode(node);
    class Vertex
    {
        public:
            Vertex(N _node) : node(std::move(_node)) {};
            Vertex(Vertex &v) : node(std::move(v.node)) {};
            N& getNode() { return node; };
            int id() const {return node.id();};
            int getLevel() const { return std::stoi(getAttrib("level").value()); };
            int getParentId() const { return std::stoi(getAttrib("parent").value()); };
            AttribValue getAttrib(const std::string &name) const 
            {
                auto value  = node.attrs().find(name);
                 if (value != node.attrs().end()) 
                    return value->second;
                AttribValue av;
                av.type("unknown");
                av.value("unknow");
                av.key(name);
                av.length(0);
                return av;
            };     
            RTMat getEdgeRT(int to) 
            {
                for(auto &[key, edge]: node.fano())
                {
                    std::cout << edge.to() << " " << edge.label() << std::endl;
                    if(edge.to() == to and edge.label() == "RT")
                    {   
                        // std::istringstream all(edge.attrs()["RT"].value());
                        // double rx, ry, rz, tx, ty, tz;
                        // std::string w;
                        // all >> w; tx = std::stod(w); all >> w; ty = std::stod(w); all >> w; tz = std::stod(w);
                        // all >> w; rx = std::stod(w); all >> w; ry = std::stod(w); all >> w; rz = std::stod(w);
                        auto &ats = edge.attrs();
                        return RTMat( std::stod(ats["rx"].value()),
                                      std::stod(ats["ry"].value()),
                                      std::stod(ats["rz"].value()),
                                      std::stod(ats["tx"].value()),
                                      std::stod(ats["ty"].value()),
                                      std::stod(ats["tz"].value()));
                    }
                }
                return RTMat();
            }
            EdgeAttribs edge(int to, const std::string &label) { edgeKey key; key.to(to); key.key(label); return node.fano()[key];};
            void insertOrAssignRTEdge(int to, const std::vector<double> &values) 
            { 
                if(values.size() < 6)
                    return;
                EdgeAttribs rt_edge; 
                rt_edge.label("RT"); rt_edge.from(node.id()); rt_edge.to(to);
                edgeKey key; key.to(to); key.key("RT");
                AttribValue rv; 
                std::vector<string> key_list{"tx","ty","tz","rx","ry","rz"};
                for(const auto& [key, val]: iter::zip(key_list, values))
                {
                    rv.key(key); rv.type("double"); rv.value(std::to_string(val)); rv.length(1);
                    rt_edge.attrs().insert_or_assign(key, rv);
                }
                node.fano().insert_or_assign(key, rt_edge);
            }
        private:
            N node;
    };

    /////////////////////////////////////////////////////////////////
    /// CRDT API
    /////////////////////////////////////////////////////////////////

    class CRDTGraph : public QObject
    {
        Q_OBJECT
        public:
            size_t size();
            CRDTGraph(int root, std::string name, int id);
            ~CRDTGraph();

            // threads
            void start_fullgraph_request_thread();
            void start_fullgraph_server_thread();
            void start_subscription_thread(bool showReceived);

            //////////////////////////////////////////////////////
            ///  Graph API
            //////////////////////////////////////////////////////

            // Utils
            void read_from_file(const std::string &xml_file_path);
            void read_from_json_file(const std::string &json_file_path);
            void write_to_json_file(const std::string &json_file_path);
            bool empty(const int &id);
            void print();
            std::map<long,Node> getCopy() const;
            std::vector<long> getKeys() const;
            
            // Nodes
            std::shared_ptr<Vertex> getNode(const std::string& name) { return std::make_shared<Vertex>(get_node(name));};
            std::shared_ptr<Vertex> getNode(int id)                  { return std::make_shared<Vertex>(get_node(id));};
            Node get_node(const std::string& name);
            Node get_node(int id);
            bool insert_or_assign_node(const N &node);
            bool delete_node(const std::string &name);
            bool delete_node(int id);

            // not working yet
            typename std::map<int, aworset<N,int>>::const_iterator begin() const { return nodes.getMap().begin(); };
            typename std::map<int, aworset<N,int>>::const_iterator end() const { return nodes.getMap().end(); };

            //Edges
            EdgeAttribs get_edge(const std::string& from, const std::string& to, const std::string& key);
            EdgeAttribs get_edge(int from, int to, const std::string& key);

            bool insert_or_assign_edge(const EdgeAttribs& attrs);
            bool delete_edge(const std::string& from, const std::string& t);
            bool delete_edge(int from, int t);

            std::string get_name_from_id(std::int32_t id);
            int get_id_from_name(const std::string &name);

            //////////////////////////////////////////////////////
            ///  Not sure who uses these
            //////////////////////////////////////////////////////

            Nodes get();
            N get(int id);
            
            //////////////////////////////////////////////////////
            /// API to access attributes
            //////////////////////////////////////////////////////
            uint getLevel(){return 0;};
            long getParentId(){return 0;};
            // gets a const node ref and searches an attrib by its name. 
            AttribValue get_node_attrib_by_name(const Node& n, const std::string &key);
        
            // Templated version to convert returned value to template type. 
            template <typename Ta>
            Ta get_node_attrib_by_name(Node& n, const std::string &key)
            {
                AttribValue av = get_node_attrib_by_name(n, key);
                return icevalue_to_nativetype<Ta>(key, av.value());
            }
            std::tuple<std::string, std::string, int> mtype_to_icevalue(const MTypes &t);
            template <typename Ta>
            Ta icevalue_to_nativetype(const std::string &name, const std::string &val)
            {
                return std::get<Ta>(icevalue_to_mtypes(name, val));
            };
            MTypes icevalue_to_mtypes(const std::string &name, const std::string &val);
            std::int32_t get_node_level(Node& n);
            std::string get_node_type(Node& n);

            // Adds an atribute to the map v with name att_name and one of the permitted types
            void add_attrib(std::map<string, AttribValue> &v, std::string att_name, CRDT::MTypes att_value);
            //void add_edge_attribs(vector<EdgeAttribs> &v, EdgeAttribs& ea);

            //For debug
            int count = 0;

        private:
            Nodes nodes;
            int graph_root;
            bool work;
            mutable std::shared_mutex _mutex;
            //std::thread read_thread, request_thread, server_thread; // Threads
            std::string filter;
            std::string agent_name;
            const int agent_id;

            std::map<string, int> name_map;     // mapping between name and id of nodes
            std::map<int, string> id_map;

            bool in(const int &id);
            N get_(int id);
            bool insert_or_assign_node_(const N &node);
            std::pair<bool, vector<tuple<int, int, std::string>>> delete_node_(int id);
            bool delete_edge_(int from, int t);
            EdgeAttribs get_edge_(int from, int to, const std::string& key);

            int id();
            DotContext context();
            std::map<int, AworSet> Map();
            
            void join_delta_node(AworSet aworSet);
            void join_full_graph(OrMap full_graph);


            class NewMessageFunctor
            {
                public:
                    CRDTGraph *graph;
                    bool *work;
                    std::function<void(eprosima::fastrtps::Subscriber *sub, bool *work, CRDT::CRDTGraph *graph)> f;

                    NewMessageFunctor(CRDTGraph *graph_, bool *work_,
                                    std::function<void(eprosima::fastrtps::Subscriber *sub, bool *work, CRDT::CRDTGraph *graph)> f_)
                            : graph(graph_), work(work_), f(std::move(f_)){}

                    NewMessageFunctor() {};
                    void operator()(eprosima::fastrtps::Subscriber *sub) { f(sub, work, graph); };
            };

            // Threads handlers
            void subscription_thread(bool showReceived);
            void fullgraph_server_thread();
            void fullgraph_request_thread();

            // Translators
            AworSet translateAwCRDTtoICE(int id, aworset<N, int> &data);
            aworset<N, int> translateAwICEtoCRDT(AworSet &data);

            // RTSP participant
            DSRParticipant dsrparticipant;
            DSRPublisher dsrpub;
            DSRSubscriber dsrsub;
            NewMessageFunctor dsrpub_call;

            DSRSubscriber dsrsub_graph_request;
            DSRPublisher dsrpub_graph_request;
            NewMessageFunctor dsrpub_graph_request_call;

            DSRSubscriber dsrsub_request_answer;
            DSRPublisher dsrpub_request_answer;
            NewMessageFunctor dsrpub_request_answer_call;

        signals:      // for graphics update
            void update_node_signal(const std::int32_t, const std::string &type);                               // Signal to update CRDT
            void update_attrs_signal(const std::int32_t &id, const std::map<string, AttribValue> &attribs);     //Signal to show node attribs.
            void update_edge_signal(const std::int32_t from, const std::int32_t to);                            // Signal to show edge attribs.
            void del_edge_signal(const std::int32_t from, const std::int32_t to, const std::string &edge_tag);  // Signal to del edge.
            void del_node_signal(const std::int32_t from);                                                      // Signal to del node.
    };

} // namespace CRDT

#endif
