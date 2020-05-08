//
// Created by crivac on 5/02/19.
//

#include "CRDT.h"
#include <fstream>
#include <unistd.h>
#include <algorithm>

#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/attributes/SubscriberAttributes.h>
#include <fastrtps/transport/UDPv4TransportDescriptor.h>
#include <fastrtps/Domain.h>

using namespace CRDT;

/////////////////////////////////////////////////
///// PUBLIC METHODS
/////////////////////////////////////////////////

CRDTGraph::CRDTGraph(int root, std::string name, int id, std::string dsr_input_file) : agent_id(id) 
{
    graph_root = root;
    agent_name = name;
    nodes = Nodes(graph_root);
    utils = std::make_unique<Utilities>(this);

    work = true;

    // RTPS Create participant 
    auto [suc, participant_handle] = dsrparticipant.init();

    // RTPS Initialize publisher with general topic
    dsrpub.init(participant_handle, "DSR", dsrparticipant.getDSRTopicName());
    dsrpub_graph_request.init(participant_handle, "DSR_GRAPH_REQUEST", dsrparticipant.getRequestTopicName());
    dsrpub_request_answer.init(participant_handle, "DSR_GRAPH_ANSWER", dsrparticipant.getAnswerTopicName());

    // RTPS Initialize comms threads
     if(dsr_input_file != std::string())
    {
        utils->read_from_json_file(dsr_input_file);
        start_fullgraph_server_thread();
        start_subscription_thread(true);
    }
    else
    {    
        start_subscription_thread(true);     // regular subscription to deltas
        bool res = start_fullgraph_request_thread();    // for agents that want to request the graph for other agent
        if(res == false) {
            eprosima::fastrtps::Domain::removeParticipant(participant_handle); // Remove a Participant and all associated publishers and subscribers.
            qFatal("CRDTGraph aborting: could not get DSR from the network after timeout");  //JC ¿se pueden limpiar aquí cosas antes de salir?

        }
    }
    qDebug() << __FUNCTION__ << "Constructor finished OK";
}


CRDTGraph::~CRDTGraph() {
    eprosima::fastrtps::Domain::removeParticipant(dsrparticipant.getParticipant()); // Remove a Participant and all associated publishers and subscribers.
}

//////////////////////////////////////
/// NODE METHODS
/////////////////////////////////////

std::optional<Node> CRDTGraph::get_node(const std::string& name)
{
    std::shared_lock<std::shared_mutex>  lock(_mutex);
    if (name.empty()) return {};
    int id = get_id_from_name(name).value_or(-1);
    if (in(id)) {
        auto n = &nodes[id].dots().ds.rbegin()->second;
        if (n->name() == name) return Node(*n);
    }

    return {};
}


std::optional<Node> CRDTGraph::get_node(int id)
{
    std::shared_lock<std::shared_mutex>  lock(_mutex);
    return get_(id);
}

std::optional<VertexPtr> CRDTGraph::get_vertex(const std::string& name)
{
    auto n = get_node(name);
    return n.has_value() ?  std::make_optional(std::make_shared<Vertex>(n.value())) : std::nullopt;
}

std::optional<VertexPtr> CRDTGraph::get_vertex(int id)
{
    auto n = get_node(id);
    return n.has_value() ?  std::make_optional(std::make_shared<Vertex>(n.value())) : std::nullopt;
}

bool CRDTGraph::insert_or_assign_node(const N &node)
{
    if (node.id() == -1) return false;
    bool r;
    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        r = insert_or_assign_node_(node);
    }
    if (r) {
        emit update_node_signal(node.id(), node.type());
        for (const auto &[k,v]: node.fano())
            emit update_edge_signal(node.id(), k.to(), v.type());
    }
    return r;
}

bool CRDTGraph::insert_or_assign_node(const VertexPtr &vertex)
{
    return insert_or_assign_node(vertex->get_CRDT_node());
}

bool CRDTGraph::insert_or_assign_node(Vertex &vertex)
{
    return insert_or_assign_node(vertex.get_CRDT_node());
}

bool CRDTGraph::insert_or_assign_node_(const N &node)
{
    if (!nodes[node.id()].dots().ds.empty() and nodes[node.id()].dots().ds.rbegin()->second == node) {
        //Esto debería ser true?
        return true;
    }
    if (deleted.find(node.id()) == deleted.end()) {
        aworset<Node, int> delta = nodes[node.id()].add(node, node.id());
        update_maps_node_insert(node.id(), node);
        auto val = translateAwCRDTtoIDL(node.id(), delta);
        dsrpub.write(&val);

        return true;
    }
    return false;
}

bool CRDTGraph::delete_node(const std::string& name)
{
    bool result = false;
    vector<tuple<int,int, std::string>> edges_;
    std::optional<int> id = {};
    {
        std::unique_lock<std::shared_mutex>  lock(_mutex);
        id = get_id_from_name(name);
        if(id.has_value()) {
            auto[r, e] = delete_node_(id.value());
            result = r;
            edges_ = e;
        } else { return false; }
    }

    if (result) {
        emit del_node_signal(id.value());

        for (auto &[id0, id1, label] : edges_)
                emit del_edge_signal(id0, id1, label);
        return true;
    }
    return false;
}

bool CRDTGraph::delete_node(int id)
{
    bool result;
    vector<tuple<int,int, std::string>> edges_;
    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        if (in(id)) {
            auto[r, e] = delete_node_(id);
            result = r;
            edges_ = e;
        } else { return false; }
    }

    if (result) {
        emit del_node_signal(id);

        for (auto &[id0, id1, label] : edges_)
                emit del_edge_signal(id0, id1, label);
        return true;
    }
    return false;
}

std::pair<bool, vector<tuple<int, int, std::string>>> CRDTGraph::delete_node_(int id)
{
    // std::cout << "Tomando lock único para borrar un nodo" << std::endl;
    vector<tuple<int,int, std::string>> edges_;
    //std::unique_lock<std::shared_mutex>  lock(_mutex);
    //1. Get and remove node.
    auto node = get_(id);
    if (!node.has_value()) return make_pair(false, edges_);
    for (const auto &v : node.value().fano()) { // Delete all edges from this node.
        std::cout << id << " -> " << v.first.to() << " " << v.first.type() << std::endl;
         edges_.emplace_back(make_tuple(id, v.first.to(), v.first.type()));
    }
    // Get remove delta.
    auto delta = nodes[id].rmv(nodes[id].dots().ds.rbegin()->second);
    auto val = translateAwCRDTtoIDL(id, delta);
    dsrpub.write(&val);
    update_maps_node_delete(id, node.value());

    //2. search and remove edges.
    //For each node check if there is an edge to remove.
    for (auto [k, v] : nodes.getMapRef()) {
        if (edges.find({k, id}) == edges.end()) continue;
        for (const auto &key : edges[{k, id}]) {

            EdgeKey ek;
            ek.to(k);
            ek.type(key);
            auto visited_node =  Node(v.dots().ds.rbegin()->second);

            visited_node.fano().erase(ek);
            auto delta = nodes[visited_node.id()].add(visited_node, visited_node.id());
            edges_.emplace_back(make_tuple(visited_node.id(), id, key));
            update_maps_edge_delete(visited_node.id(), id, key);

            // Send changes.
            auto val = translateAwCRDTtoIDL(visited_node.id(), delta);
            dsrpub.write(&val);
        }
    }
    return make_pair(true,  edges_);
    //return make_pair(false, edges_);
}

std::vector<Node> CRDTGraph::get_nodes_by_type(const std::string& type)
{
    std::shared_lock<std::shared_mutex>  lock(_mutex);
    std::vector<Node> nodes_;
    if (nodeType.find(type) != nodeType.end()) {
        for (auto id: nodeType[type]) {
            auto n = get_(id);
            if (n.has_value())
                nodes_.emplace_back(n.value());
        }
    }
    return nodes_;
}

//////////////////////////////////////////////////////////////////////////////////
// EDGE METHODS
//////////////////////////////////////////////////////////////////////////////////
std::optional<Edge> CRDTGraph::get_edge(const std::string& from, const std::string& to, const std::string& key)
{
    std::shared_lock<std::shared_mutex>  lock(_mutex);
    std::optional<int> id_from = get_id_from_name(from);
    std::optional<int> id_to = get_id_from_name(to);
    if (id_from.has_value() and id_to.has_value())
        return get_edge_(id_from.value(), id_to.value(), key);
    return {};
}


std::optional<Edge> CRDTGraph::get_edge(int from, int to, const std::string &key)
{
    return get_edge_(from, to, key);
}


std::optional<Edge> CRDTGraph::get_edge_(int from, int  to, const std::string& key)
{
     std::shared_lock<std::shared_mutex>  lock(_mutex);
     if (in(from) && in(to)) {
        auto n = get_(from);
        if (n.has_value()) {
            EdgeKey ek;
            ek.to(to);
            ek.type(key);
            auto edge = n.value().fano().find(ek);
            if (edge != n.value().fano().end()) {
                return Edge(edge->second);
            }
        }
         std::cout << __FUNCTION__ <<":" << __LINE__ << " Error obteniedo edge from: "<< from  << " to: " << to <<" key " << key << endl;
     }
     return {};
}

bool CRDTGraph::insert_or_assign_edge(const Edge& attrs)
{
    bool r = false;
    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        int from = attrs.from();
        int to = attrs.to();
        if (in(from) && in(to))
        {
            auto node = get_(from);
            if (node.has_value()) {
                EdgeKey ek;
                ek.to(to);
                ek.type(attrs.type());
                node.value().fano().insert_or_assign(ek, attrs);

                node.value().agent_id(agent_id);
                r = insert_or_assign_node_(node.value());
            }
        } else
        {
            std::cout << __FUNCTION__ <<":" << __LINE__ <<" Error. ID:"<<from<<" or "<<to<<" not found. Cant update. "<< std::endl;
            return false;
        }
    }
    if (r)
        emit update_edge_signal( attrs.from(),  attrs.to(), attrs.type());
    return true;
}


bool CRDTGraph::delete_edge(int from, int to, const std::string& key)
{
    bool result;
    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        if (!in(from) || !in(to)) return false;
        result = delete_edge_(from, to, key);
    }
    if (result)
            emit del_edge_signal(from, to, key);
    return result;
}

bool CRDTGraph::delete_edge(const std::string& from, const std::string& to, const std::string& key)
{
    std::optional<int> id_from = {};
    std::optional<int> id_to = {};
    bool result = false;
    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        id_from = get_id_from_name(from);
        id_to = get_id_from_name(to);

        if (id_from.has_value() && id_to.has_value())
            result = delete_edge_(id_from.value(), id_to.value(), key);
    }

    if (result)
        emit del_edge_signal(id_from.value(), id_to.value(), key);
    return result;
}

bool CRDTGraph::delete_edge_(int from, int to, const std::string& key)
{
    auto node = get_(from);
    if (node.has_value()) {
        EdgeKey ek;
        ek.to(to);
        ek.type(key);
        if (node.value().fano().find(ek) != node.value().fano().end()) {
            node.value().fano().erase(ek);
            update_maps_edge_delete(from, to, key);
            node.value().agent_id(agent_id);
            return insert_or_assign_node_(node.value());
        }
    }
    return false;
}

std::vector<Edge> CRDTGraph::get_edges_by_type(const std::string& type)
{
    std::shared_lock<std::shared_mutex>  lock(_mutex);
    std::vector<Edge> edges_;
    if (edgeType.find(type) != edgeType.end())
    {
        for (auto &[from, to] : edgeType[type]) {
            auto n = get_edge_(from, to, type);
            if (n.has_value())
                edges_.emplace_back(n.value());
        }
    }
    return edges_;
}

std::vector<Edge> CRDTGraph::get_edges_to_id(int id) {
    std::shared_lock<std::shared_mutex>  lock(_mutex);
    std::vector<Edge> edges_;
    for (const auto &[key, types] : edges)
    {
        auto [from, to] = key;
        if (to == id) {
            for (const std::string& type : types) {
                auto n = get_edge_(from, to, type);
                if (n.has_value())
                    edges_.emplace_back(n.value());
            }
        }
    }
    return edges_;
}


/////////////////////////////////////////////////
///// Utils
/////////////////////////////////////////////////

std::map<long,Node> CRDTGraph::getCopy() const
{
    std::map<long,Node> mymap;
    std::shared_lock<std::shared_mutex>  lock(_mutex);
    for (auto &[key, val] : nodes.getMap())
        mymap[key] = val.dots().ds.rbegin()->second;
    return mymap;
}

 std::vector<long> CRDTGraph::getKeys() const
{
    std::vector<long> keys;
    std::shared_lock<std::shared_mutex>  lock(_mutex);
    for (auto &[key, val] : nodes.getMap())
        keys.emplace_back(key);
    return keys;
}

void CRDTGraph::print()
{
    for (auto kv : nodes.getMap())
    {
        Node node = kv.second.dots().ds.rbegin()->second;
        std::cout << "Node: " << node.id() << std::endl;
        std::cout << "  Type:" << node.type() << std::endl;
        std::cout << "  Name:" << node.name() << std::endl;
        std::cout << "  Agent_id:" << node.agent_id()  << std::endl;
        for(auto [key, val] : node.attrs())
            std::cout << "      [ " << key << ", " << val.type() << ", " << val.value() << " ]"  << std::endl;
        for(auto [key, val] : node.fano())
        {
            std::cout << "          Edge-type->" << val.type() << " from:" << val.from() << " to:" << val.to()  << std::endl;
            for(auto [k, v] : val.attrs())
                std::cout << "              Key->" << k << " Type->" << v.type() << " Value->" << v.value()  << std::endl;
        }
    }
}

//////////////////////////////////////////////////////////////////////////////
/////  CORE
//////////////////////////////////////////////////////////////////////////////

/*
Nodes CRDTGraph::get() {
    std::shared_lock<std::shared_mutex>  lock(_mutex);
    return nodes;
}
*/

std::optional<Node> CRDTGraph::get(int id) {
    std::shared_lock<std::shared_mutex>  lock(_mutex);
    return get_(id);
}

std::optional<Node> CRDTGraph::get_(int id) {

    if (in(id)) {
        if (!nodes[id].dots().ds.empty()) {
            return nodes[id].dots().ds.rbegin()->second;
        }
    }
    return {};
}

std::optional<std::int32_t> CRDTGraph::get_node_level(Node& n)
{
    return get_attrib_by_name<int32_t>(n, "level");
}

std::string CRDTGraph::get_node_type(Node& n)
{
    try {
        return n.type();
    } catch(const std::exception &e){
        std::cout <<"EXCEPTION: "<<__FILE__ << " " << __FUNCTION__ <<":"<<__LINE__<< " "<< e.what() << std::endl;};
    return "error";
}

inline void CRDTGraph::update_maps_node_delete(int id, const Node& n)
{
    nodes.erase(id);
    name_map.erase(id_map[id]);
    id_map.erase(id);
    deleted.insert(id);

    if (nodeType.find(n.type()) != nodeType.end())
        nodeType[n.type()].erase(id);

    for (const auto &[k,v] : n.fano()) {
        edges[{k.to(), id}].erase(k.type());
        edgeType[k.type()].erase({id, k.to()});
    }
}

inline void CRDTGraph::update_maps_node_insert(int id, const Node& n)
{
    name_map[n.name()] = id;
    id_map[id] = n.name();
    nodeType[n.type()].insert(id);

    for (const auto &[k,v] : n.fano()) {
        edges[{k.to(), id}].insert(k.type());
        edgeType[k.type()].insert({id, k.to()});
    }
}

inline void CRDTGraph::update_maps_edge_delete(int from, int to, const std::string& key)
{
    edges[{from, to}].erase(key);
    edgeType[key].erase({from, to});
}

std::optional<int> CRDTGraph::get_id_from_name(const std::string &name)
{
        auto v = name_map.find(name);
        if (v != name_map.end()) return v->second;
        return {};

}

std::optional<std::string> CRDTGraph::get_name_from_id(std::int32_t id)
{
    auto v = id_map.find(id);
    if (v != id_map.end()) return v->second;
    return {};
}

size_t CRDTGraph::size ()
{
    std::shared_lock<std::shared_mutex>  lock(_mutex);
    return nodes.getMapRef().size();
};

bool CRDTGraph::in(const int &id)
{
    return nodes.in(id);
}

bool CRDTGraph::empty(const int &id)
{
    if (nodes.in(id)) {
        return nodes[id].dots().ds.empty();
    } else
        return false;
}


void CRDTGraph::join_delta_node(AworSet aworSet)
{
    try{
        vector<tuple<int, int , std::string>> remove;
        bool signal = true;
        auto d = translateAwIDLtoCRDT(aworSet);
        {
            std::unique_lock<std::shared_mutex> lock(_mutex);
            std::cout << "JOINING " << aworSet.id();
            Node nd = (nodes[aworSet.id()].dots().ds.rbegin() == nodes[aworSet.id()].dots().ds.rend()) ? Node() : nodes[aworSet.id()].dots().ds.rbegin()->second;
            for (const auto &[k,v] : nd.fano() )
                remove.emplace_back(make_tuple(v.from(), k.to(), k.type()));

            nodes[aworSet.id()].join_replace(d);
            if (nodes[aworSet.id()].dots().ds.size() == 0)
            {
                signal = false;
                update_maps_node_delete(aworSet.id(), nd);
                std::cout << " REMOVE" << endl;
            } else {
                update_maps_node_insert(aworSet.id(), nodes[aworSet.id()].dots().ds.rbegin()->second);
                std::cout << " INSERT"  << endl;
            }
        }

        if (signal) {
            emit update_node_signal(aworSet.id(), nodes[aworSet.id()].dots().ds.rbegin()->second.type());

            for (const auto &[from, to, type] : remove)
                emit del_edge_signal(from, to, type);

            for (const auto &[k,v] : nodes[aworSet.id()].dots().ds.rbegin()->second.fano()) {
                emit update_edge_signal(aworSet.id(), k.to(), k.type());
            }
        }
        else {
            emit del_node_signal(aworSet.id());
        }

    } catch(const std::exception &e){std::cout <<"EXCEPTION: "<<__FILE__ << " " << __FUNCTION__ <<":"<<__LINE__<< " "<< e.what() << std::endl;};
}

void CRDTGraph::join_full_graph(OrMap full_graph) 
{
    vector<tuple<bool, int, std::string>> updates;
    vector<tuple<int, int , std::string>> remove;

    {

        std::unique_lock<std::shared_mutex> lock(_mutex);
        auto m = static_cast<map<int, int>>(full_graph.cbase().cc());
        std::set<pair<int, int>> s;
        for (auto &v : full_graph.cbase().dc())
            s.emplace(std::make_pair(v.first(), v.second()));
        //nodes.context().setContext(m, s);

        for (auto &[k, val] : full_graph.m()) 
        {
            auto awor = translateAwIDLtoCRDT(val);
            Node nd = (nodes[k].dots().ds.rbegin() == nodes[k].dots().ds.rend()) ? Node() : nodes[k].dots().ds.rbegin()->second;
            for (const auto &[k,v] : nd.fano() )
                remove.emplace_back(make_tuple(v.from(), k.to(), k.type()));

            if (awor.dots().ds.size() == 0) {
                nodes[k].rmv(nd);
                update_maps_node_delete(k, nd);
                updates.emplace_back(make_tuple( false, k, ""));
            } 
            else
            {
                if (deleted.find(k) == deleted.end() and (nodes[k].dots().ds.empty() or (*awor.dots().ds.rbegin() != *nodes[k].dots().ds.rbegin()))) {
                    nodes[k].setContext(awor.context());
                    nodes[k].add(awor.dots().ds.begin()->second);
                    update_maps_node_insert(k, awor.dots().ds.begin()->second);
                    updates.emplace_back(make_tuple(true, k, nodes[k].dots().ds.begin()->second.type()));
                } 
                else 
                {
                    update_maps_node_delete(k, nd);
                }
            }
        }
    }
    for (auto &[signal, id, type] : updates)
        if (signal) {
            emit update_node_signal(id, type);

            for (const auto &[from, to, type] : remove)
                emit del_edge_signal(from, to, type);

            for (const auto &[k,v] : nodes[id].dots().ds.rbegin()->second.fano()) {
                emit update_edge_signal(id, k.to(), k.type());
            }
        }
        else {
            emit del_node_signal(id);
        }
}

bool CRDTGraph::start_fullgraph_request_thread() 
{
    return fullgraph_request_thread();
}

void CRDTGraph::start_fullgraph_server_thread() 
{
    fullgraph_server_thread();
}

void CRDTGraph::start_subscription_thread(bool showReceived) 
{
    subscription_thread(showReceived);
}

int CRDTGraph::id() 
{
    return nodes.getId();
}

DotContext CRDTGraph::context() 
{ // Context to ICE
    DotContext om_dotcontext;
    for (auto &kv_cc : nodes.context().getCcDc().first) 
    {
        om_dotcontext.cc().emplace(make_pair(kv_cc.first, kv_cc.second));
    }
    for (auto &kv_dc : nodes.context().getCcDc().second)
    {
        PairInt p_i;
        p_i.first(kv_dc.first);
        p_i.second(kv_dc.second);
        om_dotcontext.dc().push_back(p_i);
    }
    return om_dotcontext;
}

std::map<int,AworSet> CRDTGraph::Map() 
{
    std::shared_lock<std::shared_mutex>  lock(_mutex);
    std::map<int,AworSet>  m;
    for (auto kv : nodes.getMapRef()) { // Map of Aworset to ICE
        aworset<Node, int> n;

        auto last = *kv.second.dots().ds.rbegin();
        n.dots().ds.insert(last);
        n.dots().c = kv.second.dots().c;
        m[kv.first] = translateAwCRDTtoIDL(kv.first, n);
    }
    return m;
}

void CRDTGraph::subscription_thread(bool showReceived) 
{
	 // RTPS Initialize subscriptor
    auto lambda_general_topic = [&] (eprosima::fastrtps::Subscriber* sub, bool* work, CRDT::CRDTGraph *graph) {
        if (*work) {
            try {
                eprosima::fastrtps::SampleInfo_t m_info;
                AworSet sample;


                //std::cout << "Unreaded: " << sub->get_unread_count() << std::endl;
                //read or take
                if (sub->takeNextData(&sample, &m_info)) { // Get sample
                    if(m_info.sampleKind == eprosima::fastrtps::rtps::ALIVE) {
                        if( m_info.sample_identity.writer_guid().is_on_same_process_as(sub->getGuid()) == false) {
                            //std::cout << " Received: node " << sample << " from " << m_info.sample_identity.writer_guid() << std::endl;
                            std::cout << " Received: node from: " << m_info.sample_identity.writer_guid() << std::endl;
                            graph->join_delta_node(sample);
                        } /*else {
                                std::cout << "filtered" << std::endl;
                            }*/
                    }
                }
            }
            catch (const std::exception &ex) { cerr << ex.what() << endl; }
        }

    };
    dsrpub_call = NewMessageFunctor(this, &work, lambda_general_topic);
	auto res = dsrsub.init(dsrparticipant.getParticipant(), "DSR", dsrparticipant.getDSRTopicName(), dsrpub_call);
    std::cout << (res == true ? "Ok" : "Error") << std::endl;

}

void CRDTGraph::fullgraph_server_thread() 
{
    std::cout << __FUNCTION__ << "->Entering thread to attend full graph requests" << std::endl;
    // Request Topic

    auto lambda_graph_request = [&] (eprosima::fastrtps::Subscriber* sub, bool* work, CRDT::CRDTGraph *graph) {

        eprosima::fastrtps::SampleInfo_t m_info;
        GraphRequest sample;
        //readNextData o takeNextData
        if (sub->takeNextData(&sample, &m_info)) { // Get sample
            if(m_info.sampleKind == eprosima::fastrtps::rtps::ALIVE) {
                if( m_info.sample_identity.writer_guid().is_on_same_process_as(sub->getGuid()) == false) {
                    std::cout << " Received Full Graph request: from " << m_info.sample_identity.writer_guid()
                              << std::endl;

                    *work = false;
                    OrMap mp;
                    mp.id(graph->id());
                    mp.m(graph->Map());
                    mp.cbase(graph->context());
                    std::cout << "nodos enviados: " << mp.m().size()  << std::endl;

                    dsrpub_request_answer.write(&mp);

                    for (auto &[k, v] : Map())
                        std::cout << k << "," << v.dk() << std::endl;
                    std::cout << "Full graph written" << std::endl;
                    *work = true;

                }
            }
        }

    };
    dsrpub_graph_request_call = NewMessageFunctor(this, &work, lambda_graph_request);
    dsrsub_graph_request.init(dsrparticipant.getParticipant(), "DSR_GRAPH_REQUEST", dsrparticipant.getRequestTopicName(), dsrpub_graph_request_call);
};

bool CRDTGraph::fullgraph_request_thread() 
{
    bool sync = false;
    // Answer Topic
    auto lambda_request_answer = [&sync] (eprosima::fastrtps::Subscriber* sub, bool* work, CRDT::CRDTGraph *graph) {

        eprosima::fastrtps::SampleInfo_t m_info;
        OrMap sample;
        std::cout << "Mensajes sin leer " << sub->get_unread_count() << std::endl;
        if (sub->takeNextData(&sample, &m_info)) { // Get sample
            if(m_info.sampleKind == eprosima::fastrtps::rtps::ALIVE) {
                if( m_info.sample_identity.writer_guid().is_on_same_process_as(sub->getGuid()) == false) {
                    std::cout << " Received Full Graph from " << m_info.sample_identity.writer_guid() << " whith " << sample.m().size() << " elements" << std::endl;
                    graph->join_full_graph(sample);
                    std::cout << "Synchronized." <<std::endl;
                    sync = true;
                }
            }
        }
    };

    dsrpub_request_answer_call = NewMessageFunctor(this, &work, lambda_request_answer);
    dsrsub_request_answer.init(dsrparticipant.getParticipant(), "DSR_GRAPH_ANSWER", dsrparticipant.getAnswerTopicName(),dsrpub_request_answer_call);

    std::this_thread::sleep_for(300ms);

    std::cout << " Requesting the complete graph " << std::endl;
    GraphRequest gr;
    gr.from(agent_name);
    dsrpub_graph_request.write(&gr);

    bool timeout = false;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    while (!sync and !timeout) 
    {
        std::this_thread::sleep_for(500ms);
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        timeout = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() > TIMEOUT;
    }
    eprosima::fastrtps::Domain::removeSubscriber(dsrsub_request_answer.getSubscriber());
    return sync;
}

AworSet CRDTGraph::translateAwCRDTtoIDL(int id, aworset<N, int> &data) {
    AworSet delta_crdt;
    for (auto &kv_dots : data.dots().ds) {
        PairInt pi;
        pi.first(kv_dots.first.first);
        pi.second(kv_dots.first.second);

        delta_crdt.dk().ds().emplace(make_pair(pi, kv_dots.second));
    }
    for (auto &kv_cc : data.context().getCcDc().first){
        delta_crdt.dk().cbase().cc().emplace(make_pair(kv_cc.first, kv_cc.second));
    }
    for (auto &kv_dc : data.context().getCcDc().second){
        PairInt pi;
        pi.first(kv_dc.first);
        pi.second(kv_dc.second);

        delta_crdt.dk().cbase().dc().push_back(pi);
    }

    delta_crdt.id(id);
    return delta_crdt;
}

aworset<N, int> CRDTGraph::translateAwIDLtoCRDT(AworSet &data) {
    // Context
    dotcontext<int> dotcontext_aux;
    //auto m = static_cast<std::map<int, int>>(data.dk().cbase().cc());
    std::map<int, int> m;
    for (auto &v : data.dk().cbase().cc())
        m.insert(std::make_pair(v.first, v.second));
    std::set <pair<int, int>> s;
    for (auto &v : data.dk().cbase().dc())
        s.insert(std::make_pair(v.first(), v.second()));
    dotcontext_aux.setContext(m, s);
    // Dots
    std::map <pair<int, int>, N> ds_aux;
    for (auto &[k,v] : data.dk().ds())
        ds_aux[pair<int, int>(k.first(), k.second())] = v;
    // Join
    aworset<N, int> aw = aworset<N, int>(data.id());
    aw.setContext(dotcontext_aux);
    aw.dots().set(ds_aux);
    return aw;
}

/*
// Converts string values into Mtypes
CRDT::MTypes CRDTGraph::string_to_mtypes(const std::string &type, const std::string &val)
{
    // WE NEED TO ADD A TYPE field to the Attribute values and get rid of this shit
    static const std::list<std::string> string_types{ "imName", "imType", "tableType", "texture", "engine", "path", "render", "color", "type"};
    static const std::list<std::string> bool_types{ "collidable", "collide"};
    static const std::list<std::string> RT_types{ "RT"};
    static const std::list<std::string> vector_float_types{ "laser_data_dists", "laser_data_angles"};

    MTypes res;
    if(std::find(string_types.begin(), string_types.end(), type) != string_types.end())
        res = val;
    else if(std::find(bool_types.begin(), bool_types.end(), type) != bool_types.end())
    { if( val == "true") res = true; else res = false; }
    else if(std::find(vector_float_types.begin(), vector_float_types.end(), type) != vector_float_types.end())
    {
        std::vector<float> numbers;
        std::istringstream iss(val);
        std::transform(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(),
                       std::back_inserter(numbers), [](const std::string &s){ return (float)std::stof(s);});
        res = numbers;

    }
        // instantiate a QMat from string marshalling
    else if(std::find(RT_types.begin(), RT_types.end(), type) != RT_types.end())
    {
        std::vector<float> ns;
        std::istringstream iss(val);
        std::transform(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(),
                       std::back_inserter(ns), [](const std::string &s){ return QString::fromStdString(s).toFloat();});
        RMat::RTMat rt;
        if(ns.size() == 6)
        {
            rt.set(ns[3],ns[4],ns[5],ns[0],ns[1],ns[2]);
        }
        else
        {
            std::cout << __FUNCTION__ <<":"<<__LINE__<< "Error reading RTMat. Initializing with identity";
            rt = QMat::identity(4);
        }
        return(rt);
    }
    else
    {
        try
        { res = (float)std::stod(val); }
        catch(const std::exception &e)
        { std::cout << __FUNCTION__ <<":"<<__LINE__<<" catch: " << type << " " << val << std::endl; res = std::string{""}; }
    }
    return res;
}
*/

std::tuple<std::string, std::string, int> CRDTGraph::nativetype_to_string(const MTypes &t)
{
    return std::visit(overload
      {
              [](RMat::RTMat m) -> std::tuple<std::string, std::string, int> { return make_tuple("RTMat", m.serializeAsString(),m.getDataSize()); },
              [](std::vector<float> a)-> std::tuple<std::string, std::string, int>
              {
                  std::string str;
                  for(auto &f : a)
                      str += std::to_string(f) + " ";
                  return make_tuple("vector<float>",  str += "\n",a.size());
              },
              [](std::string a) -> std::tuple<std::string, std::string, int>	{ return  make_tuple("string", a,1); },
              [](auto a) -> std::tuple<std::string, std::string, int>			{ return make_tuple(typeid(a).name(), std::to_string(a),1);}
      }, t);
}


void CRDTGraph::add_attrib(std::map<string, Attrib> &v, std::string att_name, CRDT::MTypes att_value) {

    Attrib av;
    av.type(att_value.index());

    Val value;
    switch(att_value.index()) {
        case 0:
            value.str(std::get<std::string>(att_value));
            av.value( value);
            break;
        case 1:
            value.dec(std::get<std::int32_t>(att_value));
            av.value( value);
            break;
        case 2:
            value.fl(std::get<float>(att_value));
            av.value( value);
            break;
        case 3:
            value.float_vec(std::get<std::vector<float>>(att_value));
            av.value( value);
            break;
        // case 4:
        //     value.rtmat(std::get<RTMat>(att_value).toVector().toStdVector());
        //     av.value(value);
        //     break;
    }

    v[att_name] = av;
}

/////////////////////////////////////////////////////////////////////////
//// UTILS
////////////////////////////////////////////////////////////////////////

