//
// Created by crivac on 5/02/19.
//

#include "CRDT.h"
#include <iostream>
#include <unistd.h>
#include <algorithm>

#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/transport/UDPv4TransportDescriptor.h>
#include <fastrtps/Domain.h>

using namespace CRDT;

/////////////////////////////////////////////////
///// PUBLIC METHODS
/////////////////////////////////////////////////

CRDTGraph::CRDTGraph(int root, std::string name, int id, std::string dsr_input_file) : agent_id(id) , agent_name(name)
{
    graph_root = root;
    nodes = Nodes(graph_root);
    utils = std::make_unique<Utilities>(this);
    std::cout << "Agent name: " << agent_name << std::endl;
    work = true;

    // RTPS Create participant 
    auto [suc, participant_handle] = dsrparticipant.init(agent_id);

    // RTPS Initialize publisher with general topic
    dsrpub_node.init(participant_handle, "DSR_NODE", dsrparticipant.getNodeTopicName());
    dsrpub_node_attrs.init(participant_handle, "DSR_NODE_ATTRS", dsrparticipant.getNodeAttrTopicName());
    dsrpub_edge.init(participant_handle, "DSR_EDGE", dsrparticipant.getEdgeTopicName());
    dsrpub_edge_attrs.init(participant_handle, "DSR_EDGE_ATTRS", dsrparticipant.getEdgeAttrTopicName());

    dsrpub_graph_request.init(participant_handle, "DSR_GRAPH_REQUEST", dsrparticipant.getRequestTopicName());
    dsrpub_request_answer.init(participant_handle, "DSR_GRAPH_ANSWER", dsrparticipant.getAnswerTopicName());

    // RTPS Initialize comms threads
     if(dsr_input_file != std::string())
    {
        try
        {   
            utils->read_from_json_file(dsr_input_file); 
            qDebug() << __FUNCTION__ << "Warning, graph read from file " << QString::fromStdString(dsr_input_file);     
        }
        catch(const CRDT::DSRException& e)
        {  
            std::cout << e.what() << '\n';  
            qFatal("Aborting program. Cannot continue without intial file");
        }
        start_fullgraph_server_thread();
        start_subscription_threads(false);
    }
    else
    {    
        start_subscription_threads(false);     // regular subscription to deltas
        bool res = start_fullgraph_request_thread();    // for agents that want to request the graph for other agent
        if(res == false) {
            eprosima::fastrtps::Domain::removeParticipant(participant_handle); // Remove a Participant and all associated publishers and subscribers.
            qFatal("CRDTGraph aborting: could not get DSR from the network after timeout");  //JC ¿se pueden limpiar aquí cosas antes de salir?

        }
    }
    qDebug() << __FUNCTION__ << "Constructor finished OK";
}

CRDTGraph::~CRDTGraph() 
{
    qDebug() << "Removing rtps participant";
    eprosima::fastrtps::Domain::removeParticipant(dsrparticipant.getParticipant()); // Remove a Participant and all associated publishers and subscribers.
    fullgraph_thread.join();
    delta_node_thread.join();
    delta_edge_thread.join();
    delta_node_attrs_thread.join();
    delta_edge_attrs_thread.join();
}

//////////////////////////////////////
/// NODE METHODS
/////////////////////////////////////

std::optional<Node> CRDTGraph::get_node(const std::string& name)
{
    std::shared_lock<std::shared_mutex>  lock(_mutex);
    if (name.empty()) return {};
    int id = get_id_from_name(name).value_or(-1);
    return get_(id);
}

std::optional<Node> CRDTGraph::get_node(int id)
{
    std::shared_lock<std::shared_mutex>  lock(_mutex);
    return get_(id);
}



std::tuple<bool, std::optional<IDL::Mvreg>> CRDTGraph::insert_node_(const Node &node)
{
    if (deleted.find(node.id()) == deleted.end()) {
        if (!nodes[node.id()].read().empty() and *nodes[node.id()].read().begin() == node){
                return {true, {} };
        }
        mvreg<Node, int> delta = nodes[node.id()].write(node);
        update_maps_node_insert(node.id(), node);

        return { true, translateNodeMvCRDTtoIDL(node.id(), delta) };
    }
    return {false, {} };
}


std::optional<uint32_t> CRDTGraph::insert_node(Node& node) {
    if (node.id() == -1) return {};
    std::optional<IDL::Mvreg> aw;

    bool r = false;
    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        if (id_map.find(node.id()) == id_map.end() and name_map.find(node.name())  == name_map.end()) {
            //TODO: Poner id con el proxy y generar el nombre

            std::tie(r, aw) = insert_node_(node);
        } else throw std::runtime_error((std::string("Cannot insert node in G, a node with the same id already exists ")
                                         + __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__)).data());
    }
    if (r) {
        if (aw.has_value()) {
            dsrpub_node.write(&aw.value());
            emit update_node_signal(node.id(), node.type());
            for (const auto &[k, v]: node.fano())
                    emit update_edge_signal(node.id(), k.first, k.second);
        }

        return node.id();
    }
    return {};
}



std::tuple<bool,  std::optional<std::vector<IDL::MvregNodeAttr>>> CRDTGraph::update_node_(const Node &node)
{
    if (deleted.find(node.id()) == deleted.end()) {
        if (!nodes[node.id()].read().empty()){
            if (*nodes[node.id()].read().begin() == node) {
                return {true, {}};
            }

            vector<IDL::MvregNodeAttr> atts_deltas;
            auto iter = nodes[node.id()].read().begin()->attrs();
            for (auto [k,att]: node.attrs()) {
                //comparar timestamp o inexistencia
                if (iter.find(k) != iter.end() and att.read().begin()->timestamp() > iter.at(k).read().begin()->timestamp()) {
                    auto delta = iter.at(k).write(*iter.at(k).read().begin());
                    atts_deltas.emplace_back(translateNodeAttrMvCRDTtoIDL(node.id(), node.id(), k, delta ));
                }
            }

            if (!atts_deltas.empty()) return { true, atts_deltas};
        }
    }
    return {false, {}};
}


bool CRDTGraph::update_node(N &node)
{
    if (node.id() == -1) return false;
    bool r = false;
    std::optional<std::vector<IDL::MvregNodeAttr>> vec_node_attr;

    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        if ((id_map.find(node.id()) != id_map.end() and id_map[node.id()] != node.name())  or (name_map.find(node.name()) != name_map.end() and name_map[node.name()] != node.id()))
            throw std::runtime_error((std::string("Cannot update node in G, id and name must be unique")  + __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__)).data());
        //node.agent_id(agent_id);
        std::tie(r, vec_node_attr) = update_node_(node);
    }
    if (r) {
         if (vec_node_attr.has_value()){
            for (auto &v: vec_node_attr.value())
                dsrpub_node_attrs.write(&v);

            emit update_node_signal(node.id(), node.type());
        }
    }
    return r;
}




std::tuple<bool, vector<tuple<int, int, std::string>>, std::optional<IDL::Mvreg>, vector<IDL::MvregEdge>> CRDTGraph::delete_node_(int id)
{

    vector<tuple<int,int, std::string>> edges_;
    vector<IDL::MvregEdge> aw;

    //1. Get and remove node.
    auto node = get_(id);
    if (!node.has_value()) return make_tuple(false, edges_, std::nullopt, aw);
    for (const auto &v : node.value().fano()) { // Delete all edges from this node.
        std::cout << id << " -> " << v.first.first << " " << v.first.second<< std::endl;
         edges_.emplace_back(make_tuple(id, v.first.first, v.first.second));
    }
    // Get remove delta.
    auto delta = nodes[id].reset();
    IDL::Mvreg delta_remove = translateNodeMvCRDTtoIDL(id, delta);
    update_maps_node_delete(id, node.value());

    //2. search and remove edges.
    //For each node check if there is an edge to remove.
    for (auto &[k, v] : nodes.getMapRef()) {
        if (edges.find({k, id}) == edges.end()) continue;
        // Remove all edges between them
        auto visited_node =  Node(*v.read().begin());
        for (const auto &key : edges[{k, id}]) {

            auto delta_fano =  visited_node.fano()[{k, key}].reset();
            aw.emplace_back( translateEdgeMvCRDTtoIDL(visited_node.id(), delta_fano));
            visited_node.fano().erase({k, key});
            edges_.emplace_back(make_tuple(visited_node.id(), id, key));

            edgeType[key].erase({visited_node.id(), id});
        }


        //Remove all from cache
        edges.erase({visited_node.id(), id});
    }
    return make_tuple(true,  edges_, delta_remove, aw);

}

bool CRDTGraph::delete_node(const std::string& name)
{

    bool result = false;
    vector<tuple<int,int, std::string>> edges_;
    std::optional<IDL::Mvreg> deleted_node;
    vector<IDL::MvregEdge> aw_;

    std::optional<int> id = {};
    {
        std::unique_lock<std::shared_mutex>  lock(_mutex);
        id = get_id_from_name(name);
        if(id.has_value()) {
            std::tie(result, edges_, deleted_node, aw_) = delete_node_(id.value());
        } else {
            return false;
        }
    }

    if (result) {
        emit del_node_signal(id.value());

        dsrpub_node.write(&deleted_node.value());

        for (auto &a  : aw_)
            dsrpub_edge.write(&a);

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
    std::optional<IDL::Mvreg> deleted_node;
    vector<IDL::MvregEdge> aw_;
    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        if (in(id)) {
            std::tie(result, edges_, deleted_node, aw_) = delete_node_(id);
        } else { return false; }
    }

    if (result) {
        emit del_node_signal(id);

        dsrpub_node.write(&deleted_node.value());

        for (auto &a  : aw_)
            dsrpub_edge.write(&a);

        for (auto &[id0, id1, label] : edges_)
                emit del_edge_signal(id0, id1, label);
        return true;
    }

    return false;
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
std::optional<Edge> CRDTGraph::get_edge_(int from, int  to, const std::string& key)
{
    std::shared_lock<std::shared_mutex>  lock(_mutex);
    if (in(from) && in(to)) {
        auto n = get_(from);
        if (n.has_value()) {
            IDL::EdgeKey ek; ek.to(to); ek.type(key);
            auto edge = n.value().fano().find({to, key});
            if (edge != n.value().fano().end()) {
                return *edge->second.read().begin();
            }
        }
        std::cout << __FUNCTION__ <<":" << __LINE__ << " Error obteniedo edge from: "<< from  << " to: " << to <<" key " << key << endl;
    }
    return {};
}

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


std::tuple<bool, std::optional<IDL::MvregEdge>, std::optional<std::vector<IDL::MvregEdgeAttr>>> CRDTGraph::insert_or_assign_edge_(const Edge& attrs, int from, int to)
{

    std::optional<IDL::MvregEdge> delta_edge;
    std::optional<std::vector<IDL::MvregEdgeAttr>> delta_attrs;

    auto node = get_(from);
    if (node.has_value()) {
        //check if we are creating an edge or we are updating it.
        if (node->fano().find({to, attrs.type()}) != node->fano().end()) {//Update

            vector<IDL::MvregEdgeAttr> atts_deltas;
            auto iter = nodes[from].read().begin()->fano().find({attrs.to(), attrs.type()});
            if (iter != nodes[from].read().begin()->fano().end()) {
                auto iter_edge = iter->second.read().begin()->attrs();
                for (auto[k, att]: attrs.attrs()) {
                    //comparar timestamp o inexistencia
                    if (iter_edge.find(k) != iter_edge.end() and
                        att.read().begin()->timestamp() >
                        iter_edge.at(k).read().begin()->timestamp()) {
                        auto delta = iter_edge.at(k).write(*iter_edge.at(k).read().begin());
                        atts_deltas.emplace_back(translateEdgeAttrMvCRDTtoIDL(from, from, to, attrs.type(), k, delta));
                    }
                }
                return {true, {}, atts_deltas};
            }
        } else { // Insert
            auto delta = node->fano()[{to, attrs.type()}].write(attrs);
            return {true, translateEdgeMvCRDTtoIDL(from, delta), {}};
        }
    }
    return {false, {}, {}};
}


bool CRDTGraph::insert_or_assign_edge(const Edge& attrs)
{
    bool r = false;
    std::optional<IDL::MvregEdge> delta_edge;
    std::optional<std::vector<IDL::MvregEdgeAttr>> delta_attrs;


    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        int from = attrs.from();
        int to = attrs.to();
        if (in(from) && in(to))
        {
            std::tie(r, delta_edge, delta_attrs) = insert_or_assign_edge_(attrs, from, to);
        } else
        {
            std::cout << __FUNCTION__ <<":" << __LINE__ <<" Error. ID:"<<from<<" or "<<to<<" not found. Cant update. "<< std::endl;
            return false;
        }
    }
    if (r) {
        if (delta_edge.has_value()) { //Insert
            dsrpub_edge.write(&delta_edge.value());
        }
        if (delta_attrs.has_value()) { //Update
            for (auto &d : delta_attrs.value())
                dsrpub_edge_attrs.write(&d);
        }
        emit update_edge_signal(attrs.from(), attrs.to(), attrs.type());

    }
    return true;
}





void CRDTGraph::insert_or_assign_edge_RT(Node& n, int to, std::vector<float>&& trans, std::vector<float>&& rot_euler)
{

    bool r1 = false;
    bool r2 = false;
    std::optional<IDL::MvregEdge> node1_insert;
    std::optional<vector<IDL::MvregEdgeAttr>> node1_update;
    std::optional<vector<IDL::MvregNodeAttr>> node2;
    std::optional<Node> to_n;
    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        if (in(to))
        {
            Edge e; e.to(to); e.from(n.id()); e.type("RT"); e.agent_id(agent_id);
            Attribute tr; tr.type(3); tr.val(Value(std::move(trans))); tr.timestamp(get_unix_timestamp());
            Attribute rot; rot.type(3); rot.val(Value(std::move(rot_euler))); rot.timestamp(get_unix_timestamp());
            e.attrs()["rotation_euler_xyz"].write(rot);
            e.attrs()["translation"].write(tr);

            to_n = get_(to).value();
            bool res1 = modify_attrib(to_n.value(), "parent", n.id());
            if (!res1) (void) add_attrib(to_n.value(), "parent", n.id());
            bool res2 = modify_attrib(to_n.value(), "level",  get_node_level(n).value() + 1 );
            if (!res2) (void) add_attrib(to_n.value(), "level",  get_node_level(n).value() + 1 );

            //Check if RT edge exist.
            if (n.fano().find({to, "RT"}) == n.fano().end()) {
                //Create -> from: IDL::MvregEdge, to: vector<IDL::MvregNodeAttr>

                std::tie(r1, node1_insert, std::ignore) = insert_or_assign_edge_(e, n.id(), to);
                std::tie(r2,  node2) = update_node_(to_n.value());

                if(!r1 || !r2) {
                    throw std::runtime_error("Could not insert Node " + std::to_string(n.id()) + " in G in insert_or_assign_edge_RT() " +  __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__));
                }
            } else {
                //Update -> from: IDL::MvregEdgeAttr, to: vector<IDL::MvregNodeAttr>

                std::tie(r1,  std::ignore, node1_update) = insert_or_assign_edge_(e, n.id(), to);
                std::tie(r2,  node2) = update_node_(to_n.value());

                if(!r1 || !r2) {
                    throw std::runtime_error("Could not insert Node " + std::to_string(n.id()) + " in G in insert_or_assign_edge_RT() " +  __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__));
                }
            }

          } else
                throw std::runtime_error("Destination node " + std::to_string(n.id()) + " not found in G in insert_or_assign_edge_RT() "  +  __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__));
    }
    if (node1_insert.has_value() and node2.has_value()) {
        dsrpub_edge.write(&node1_insert.value());
        for (auto &d : node2.value())
             dsrpub_node_attrs.write(&d);
    }

    if (node1_update.has_value() and node2.has_value()) {
        for (auto &d : node1_update.value())
            dsrpub_edge_attrs.write(&d);

        for (auto &d : node2.value())
            dsrpub_node_attrs.write(&d);
    }

    emit update_edge_signal(n.id(), to, "RT");
    emit update_node_signal(to_n->id(), to_n->type());

}

void CRDTGraph::insert_or_assign_edge_RT(Node& n, int to, const std::vector<float>& trans, const std::vector<float>& rot_euler)
{

    bool r1 = false;
    bool r2 = false;
    std::optional<IDL::MvregEdge> node1_insert;
    std::optional<vector<IDL::MvregEdgeAttr>> node1_update;
    std::optional<vector<IDL::MvregNodeAttr>> node2;
    std::optional<Node> to_n;
    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        if (in(to))
        {
            Edge e; e.to(to); e.from(n.id()); e.type("RT"); e.agent_id(agent_id);
            Attribute tr; tr.type(3); tr.val(Value(trans)); tr.timestamp(get_unix_timestamp());
            Attribute rot; tr.type(3); rot.val(Value(rot_euler)); tr.timestamp(get_unix_timestamp());
            e.attrs()["rotation_euler_xyz"].write(rot);
            e.attrs()["translation"].write(tr);


            to_n = get_(to).value();
            bool res1 = modify_attrib(to_n.value(), "parent", n.id());
            if (!res1) (void) add_attrib(to_n.value(), "parent", n.id());
            bool res2 = modify_attrib(to_n.value(), "level",  get_node_level(n).value() + 1 );
            if (!res2) (void) add_attrib(to_n.value(), "level",  get_node_level(n).value() + 1 );

            //Check if RT edge exist.
            if (n.fano().find({to, "RT"}) == n.fano().end()) {
                //Create -> from: IDL::MvregEdge, to: vector<IDL::MvregNodeAttr>
                std::tie(r1, node1_insert, std::ignore) = insert_or_assign_edge_(e, n.id(), to);
                std::tie(r2,  node2) = update_node_(to_n.value());

                if(!r1 || !r2) {
                    throw std::runtime_error("Could not insert Node " + std::to_string(n.id()) + " in G in insert_or_assign_edge_RT() " +  __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__));
                }
            } else {
                //Update -> from: IDL::MvregEdgeAttr, to: vector<IDL::MvregNodeAttr>

                std::tie(r1,  std::ignore, node1_update) = insert_or_assign_edge_(e, n.id(), to);
                std::tie(r2,  node2) = update_node_(to_n.value());

                if(!r1 || !r2) {
                    throw std::runtime_error("Could not insert Node " + std::to_string(n.id()) + " in G in insert_or_assign_edge_RT() " +  __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__));
                }
            }

        } else
            throw std::runtime_error("Destination node " + std::to_string(n.id()) + " not found in G in insert_or_assign_edge_RT() "  +  __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__));
    }
    if (node1_insert.has_value() and node2.has_value()) {
        dsrpub_edge.write(&node1_insert.value());
        for (auto &d : node2.value())
            dsrpub_node_attrs.write(&d);
    }

    if (node1_update.has_value() and node2.has_value()) {
        for (auto &d : node1_update.value())
            dsrpub_edge_attrs.write(&d);

        for (auto &d : node2.value())
            dsrpub_node_attrs.write(&d);
    }

    emit update_edge_signal(n.id(), to, "RT");
    emit update_node_signal(to_n->id(), to_n->type());

}


std::optional<IDL::MvregEdge> CRDTGraph::delete_edge_(int from, int to, const std::string& key)
{
    auto node = get_(from);
    if (node.has_value()) {
        if (node.value().fano().find({to, key}) != node.value().fano().end()) {
            auto delta = node.value().fano().at({to, key}).reset();
            node.value().fano().erase({to, key});
            update_maps_edge_delete(from, to, key);
            //node.value().agent_id(agent_id);
            return translateEdgeMvCRDTtoIDL(from, delta);
        }
    }
    return {};
}

bool CRDTGraph::delete_edge(int from, int to, const std::string& key)
{

    std::optional<IDL::MvregEdge> delta;
    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        if (!in(from) || !in(to)) return false;
        delta = delete_edge_(from, to, key);
    }
    if (delta.has_value()) {
        emit del_edge_signal(from, to, key);
        dsrpub_edge.write(&delta.value());
        return true;
    }
    return false;

}

bool CRDTGraph::delete_edge(const std::string& from, const std::string& to, const std::string& key)
{

    std::optional<int> id_from = {};
    std::optional<int> id_to = {};
    std::optional<IDL::MvregEdge> delta;
    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        id_from = get_id_from_name(from);
        id_to = get_id_from_name(to);

        if (id_from.has_value() && id_to.has_value()) {
            delta = delete_edge_(id_from.value(), id_to.value(), key);
        }
    }

    if (delta.has_value()) {
        emit del_edge_signal(id_from.value(), id_to.value(), key);
        dsrpub_edge.write(&delta.value());
        return true;
    }
    return false;
}



std::vector<Edge> CRDTGraph::get_edges_by_type(const Node& node, const std::string& type)
{
    std::vector<Edge> edges_;
    for (auto &[key, edge] : node.fano()) 
        if( key.second == type )
            edges_.emplace_back(*edge.read().begin());
    return edges_;
}

std::vector<Edge> CRDTGraph::get_edges_by_type(const std::string& type)
{
    std::shared_lock<std::shared_mutex>  lock(_mutex);
    std::vector<Edge> edges_;
    if (edgeType.find(type) != edgeType.end())
    {
        for (auto &[from, to] : edgeType[type]) 
        {
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

std::optional<std::unordered_map<std::pair<int, std::string>, Edge,pair_hash>> CRDTGraph::get_edges(int id)
{
    std::unordered_map<std::pair<int, std::string>, Edge,pair_hash> pa;
    std::shared_lock<std::shared_mutex>  lock(_mutex);
    std::optional<Node> n = get_node(id);
    if (n.has_value()) {
       for (auto & [k,v] : n.value().fano()) {
           pa.emplace(make_pair(k, v.read_reg()));
       }
       return pa;
    }
    return  std::nullopt;

};

Edge CRDTGraph::get_edge_RT(const Node &n, int to)
{
    auto edges = n.fano();
    //EdgeKey key; key.to(to); key.type("RT");
    auto res  = edges.find({to, "RT"});
    if (res != edges.end())
        return *res->second.read().begin();
    else
        throw std::runtime_error("Could not find edge " + std::to_string(to) + " in node " + std::to_string(n.id()) + " in edge_to_RTMat() " +  __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__));
    
}

RTMat CRDTGraph::get_edge_RT_as_RTMat(const Edge &edge)
{
    auto r = get_attrib_by_name<std::vector<float>>(edge, "rotation_euler_xyz");
    auto t = get_attrib_by_name<std::vector<float>>(edge, "translation");
    if( r.has_value() and t.has_value() )
        return RTMat { r.value()[0], r.value()[1], r.value()[2], t.value()[0], t.value()[1], t.value()[2] } ;
    else
        throw std::runtime_error("Could not find required attributes in node " + edge.type() + " " + std::to_string(edge.to()) + " in get_edge_RT as_RTMat() " +  __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__));
   
}

 RTMat CRDTGraph::get_edge_RT_as_RTMat(Edge &&edge)
 {
    auto r = get_attrib_by_name<std::vector<float>>(edge, "rotation_euler_xyz");
    auto t = get_attrib_by_name<std::vector<float>>(edge, "translation");
    if( r.has_value() and t.has_value() )
        return RTMat { r.value()[0], r.value()[1], r.value()[2], t.value()[0], t.value()[1], t.value()[2] } ;
    else
        throw std::runtime_error("Could not find required attributes in node " + edge.type() + " " + std::to_string(edge.to()) + " in get_edge_RT as_RTMat() "  +  __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__));
 } 
     

/////////////////////////////////////////////////
///// Utils
/////////////////////////////////////////////////

std::map<long,Node> CRDTGraph::getCopy() const
{
    std::map<long,Node> mymap;
    std::shared_lock<std::shared_mutex>  lock(_mutex);
    for (auto &[key, val] : nodes.getMap())
        mymap[key] = *val.read().begin();

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


//////////////////////////////////////////////////////////////////////////////
/////  CORE
//////////////////////////////////////////////////////////////////////////////

std::optional<Node> CRDTGraph::get(int id) {
    std::shared_lock<std::shared_mutex>  lock(_mutex);
    return get_(id);
}

std::optional<Node> CRDTGraph::get_(int id) {

    if (in(id)) {
        if (!nodes[id].read().empty()) {
            return make_optional(*nodes[id].read().begin());
        }
    }
    return {};
}

std::optional<std::int32_t> CRDTGraph::get_node_level(Node& n)
{
    return get_attrib_by_name<std::int32_t>(n, "level");
}

std::optional<std::int32_t> CRDTGraph::get_parent_id(const Node& n)
{
    return get_attrib_by_name<std::int32_t>(n, "parent");
}

std::optional<Node> CRDTGraph::get_parent_node(const Node &n)
{
    auto p =  get_attrib_by_name<std::int32_t>(n, "parent");
    if (p.has_value()) {
        std::shared_lock<std::shared_mutex> lock(_mutex);
        return get_(p.value());
    }
    return {};
}


std::string CRDTGraph::get_node_type(Node& n)
{
    return n.type();
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
        edges[{id, v.read().begin()->to()}].erase(k.second);
        if(edges[{id,k.first}].empty()) edges.erase({id,k.first});
        edgeType[k.second].erase({id, k.first});
    }
}

inline void CRDTGraph::update_maps_node_insert(int id, const Node& n)
{
    name_map[n.name()] = id;
    id_map[id] = n.name();
    nodeType[n.type()].emplace(id);

    for (const auto &[k,v] : n.fano()) {
        edges[{id, k.first}].insert(k.second);
        edgeType[k.second].insert({id, k.first});
    }
}


inline void CRDTGraph::update_maps_edge_delete(int from, int to, const std::string& key)
{
    edges[{from, to}].erase(key);
    if(edges[{from, to}].empty()) edges.erase({from, to});
    edgeType[key].erase({from, to});
}

inline void CRDTGraph::update_maps_edge_insert(int from, int to, const std::string& key)
{
    edges[{from, to}].insert(key);
    edgeType[key].insert({from, to});
}


inline std::optional<int> CRDTGraph::get_id_from_name(const std::string &name)
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

bool CRDTGraph::in(const int &id) const
{
    return nodes.in(id);
}

bool CRDTGraph::empty(const int &id)
{
    if (nodes.in(id)) 
    {
        return nodes[id].read().empty();
    } else
        return false;
}

void CRDTGraph::join_delta_node(IDL::Mvreg &mvreg)
{
    try{

        bool signal = false, ok = false;
        auto d = translateNodeMvIDLtoCRDT(mvreg);
        {
            std::unique_lock<std::shared_mutex> lock(_mutex);
            if (deleted.find(mvreg.id()) == deleted.end()) {
                ok = true;
                Node nd = (nodes[mvreg.id()].read().empty()) ?
                      Node() :  *nodes[mvreg.id()].read().begin() ;

                nodes[mvreg.id()].join(d);
                if (nodes[mvreg.id()].read().empty() or mvreg.dk().ds().empty()) {
                    std::cout << "JOIN REMOVE" << std::endl;
                    //Delete all nodes and attributes received out of order
                    temp_edge.erase(mvreg.id());
                    temp_node_attr.erase(mvreg.id());
                    //Update Maps
                    update_maps_node_delete(mvreg.id(), nd);
                } else {
                    std::cout << "JOIN INSERT" << std::endl;
                    signal = true;
                    //We have to consume all unordered delta edges for this node. Normally there won't be any.
                    for (auto &[k,v] : temp_edge[mvreg.id()]) {
                        nodes[mvreg.id()].read_reg().fano()[{std::get<1>(k), std::get<2>(k)}].join(v);
                        if (nodes[mvreg.id()].read_reg().fano()[{std::get<1>(k), std::get<2>(k)}].read().empty()) {
                            nodes[mvreg.id()].read_reg().fano().erase({std::get<1>(k), std::get<2>(k)});
                        }
                        temp_edge[mvreg.id()].erase(k);
                    }

                    //We have to consume all unordered delta attributes for this node. Normally there won't be any.
                    for (auto &[k,v] : temp_node_attr[mvreg.id()]) {
                        nodes[mvreg.id()].read_reg().attrs()[k].join(v);
                        if (nodes[mvreg.id()].read_reg().attrs()[k].read().empty()) {
                            nodes[mvreg.id()].read_reg().attrs().erase(k);
                        }
                        temp_node_attr[mvreg.id()].erase(k);
                    }

                    update_maps_node_insert(mvreg.id(), *nodes[mvreg.id()].read().begin());
                }
            }
        }

        if (ok) {
            if (signal) {
                emit update_node_signal(mvreg.id(), nodes[mvreg.id()].read().begin()->type());
            } else {
                emit del_node_signal(mvreg.id());
            }
        }

    } catch(const std::exception &e){
         std::cout <<"EXCEPTION: "<<__FILE__ << " " << __FUNCTION__ <<":"<<__LINE__<< " "<< e.what() << std::endl;};

}


void CRDTGraph::join_delta_edge(IDL::MvregEdge &mvreg)
{
    try{
        bool signal = false, ok = false;
        auto d = translateEdgeMvIDLtoCRDT(mvreg);
        {
            std::unique_lock<std::shared_mutex> lock(_mutex);

            //Check if the node where we are joining the edge exist.
            if (!nodes[mvreg.id()].read().empty()) {
                ok = true;
                auto& n = nodes[mvreg.id()].read_reg();
                n.fano()[{mvreg.to(), mvreg.type()}].join(d);

                //Check if we are inserting or deleting.
                if (mvreg.dk().ds().empty() or n.fano().find({mvreg.to(), mvreg.type()}) == n.fano().end()) { //Remove
                    //Delete received items out of order
                    temp_edge_attr.erase({mvreg.from(), mvreg.to(), mvreg.type()});
                    //Update maps
                    update_maps_edge_delete(mvreg.from(), mvreg.to(), mvreg.type());
                } else { //Insert
                    signal = true;

                    //We have to consume all unordered delta attributes for this edge. Normally there won't be any.
                    for (auto &[k,v] : temp_edge_attr[{mvreg.from(), mvreg.to(), mvreg.type()}]) {
                        n.fano()[{mvreg.to(), mvreg.type()}].read_reg().attrs()[k].join(v);
                        if (n.fano()[{mvreg.to(), mvreg.type()}].read_reg().attrs()[k].read().empty()) {
                            n.fano()[{mvreg.to(), mvreg.type()}].read_reg().attrs().erase(k);
                        }
                        temp_edge_attr[{mvreg.from(), mvreg.to(), mvreg.type()}].erase(k);
                    }
                    //Update maps
                    update_maps_edge_insert(mvreg.from(), mvreg.to(), mvreg.type());
                }
            }
            else if (deleted.find(mvreg.id()) == deleted.end()) {
                //If the node is not found but it is not deleted, we save de delta for later.
                //We use a CRDT because we can receive multiple deltas for the same edge unordered before we get the node.
                temp_edge[mvreg.from()][{mvreg.from(), mvreg.to(), mvreg.type()}].join( d);

                //If we are deleting the edge
                if (temp_edge[mvreg.from()].find({mvreg.from(), mvreg.to(), mvreg.type()}) == temp_edge[mvreg.from()].end()) {
                    temp_edge[mvreg.from()].erase({mvreg.from(), mvreg.to(), mvreg.type()}); //Delete the mvreg if the edge is deleted.
                    if (temp_edge[mvreg.id()].empty())  { temp_edge.erase(mvreg.id()); }
                    temp_edge_attr.erase({mvreg.from(), mvreg.to(), mvreg.type()});
                } else {
                    //Consume al attributes
                    for (auto &[k, v] : temp_edge_attr[{mvreg.from(), mvreg.to(), mvreg.type()}]) {
                        temp_edge[mvreg.from()][{mvreg.from(), mvreg.to(), mvreg.type()}].read_reg().attrs()[k].join(v);
                        if (temp_edge[mvreg.from()][{mvreg.from(), mvreg.to(), mvreg.type()}].read_reg().attrs()[k].read().empty()) {
                            temp_edge[mvreg.from()][{mvreg.from(), mvreg.to(), mvreg.type()}].read_reg().attrs().erase(k);
                        }
                        temp_edge_attr[{mvreg.from(), mvreg.to(), mvreg.type()}].erase(k);
                    }
                }
            }
        }

        if (ok) {
            if (signal) {
                emit update_edge_signal(mvreg.from(), mvreg.to(), mvreg.type());
            } else {
                emit del_edge_signal(mvreg.from(), mvreg.to(), mvreg.type());
            }
        }

    } catch(const std::exception &e){
        std::cout <<"EXCEPTION: "<<__FILE__ << " " << __FUNCTION__ <<":"<<__LINE__<< " "<< e.what() << std::endl;};


}


void CRDTGraph::join_delta_node_attr(IDL::MvregNodeAttr &mvreg)
{

    try{
        bool signal = false, ok = false;
        auto d = translateNodeAttrMvIDLtoCRDT(mvreg);
        {
            std::unique_lock<std::shared_mutex> lock(_mutex);
            //Check if the node where we are joining the edge exist.
            if (!nodes[mvreg.id()].read().empty()) {
                ok = true;
                auto& n = nodes[mvreg.id()].read_reg();
                n.attrs()[mvreg.attr_name()].join(d);

                //Check if we are inserting or deleting.
                if (mvreg.dk().ds().empty() or n.attrs().find(mvreg.attr_name()) == n.attrs().end()) { //Remove
                    n.attrs().erase(mvreg.attr_name());
                    //Update maps
                    update_maps_node_insert(mvreg.id(), n);
                } else { //Insert
                    signal = true;
                    //Update maps
                    update_maps_node_insert(mvreg.id(), n);
                }
            }
            else if (deleted.find(mvreg.id()) == deleted.end()) {
                //If the node is not found but it is not deleted, we save de delta for later.
                //We use a CRDT because we can receive multiple deltas for the same edge unordered before we get the node.
                temp_node_attr[mvreg.id()][mvreg.attr_name()].join( d);

                //If we are deleting the edge
                if (temp_node_attr[mvreg.id()].find(mvreg.attr_name()) == temp_node_attr[mvreg.id()].end()) {
                    temp_node_attr[mvreg.id()].erase(mvreg.attr_name()); //Delete the mvreg if the edge is deleted.
                    if (temp_node_attr[mvreg.id()].empty())  { temp_node_attr.erase(mvreg.id()); }
                }
            }
        }

        if (ok) {
            if (signal) {
                emit update_node_signal(mvreg.id(), nodes[mvreg.id()].read().begin()->type());
            }
        }

    } catch(const std::exception &e){
        std::cout <<"EXCEPTION: "<<__FILE__ << " " << __FUNCTION__ <<":"<<__LINE__<< " "<< e.what() << std::endl;};



}


void CRDTGraph::join_delta_edge_attr(IDL::MvregEdgeAttr &mvreg)
{
    try{
        bool signal = false, ok = false;
        auto d = translateEdgeAttrMvIDLtoCRDT(mvreg);
        {
            std::unique_lock<std::shared_mutex> lock(_mutex);
            //Check if the node where we are joining the edge exist.
            if (!nodes[mvreg.id()].read().empty() and !nodes[mvreg.id()].read_reg().fano()[{mvreg.to(), mvreg.type()}].read().empty()) {
                ok = true;
                auto& n = nodes[mvreg.id()].read_reg().fano()[{mvreg.to(), mvreg.type()}].read_reg();
                n.attrs()[mvreg.attr_name()].join(d);

                //Check if we are inserting or deleting.
                if (mvreg.dk().ds().empty() or n.attrs().find(mvreg.attr_name()) == n.attrs().end()) { //Remove
                    n.attrs().erase(mvreg.attr_name());
                    //Update maps
                    update_maps_node_insert(mvreg.id(), nodes[mvreg.id()].read_reg());
                } else { //Insert
                    signal = true;
                    //Update maps
                    update_maps_node_insert(mvreg.id(), nodes[mvreg.id()].read_reg());
                }
            }
            else if (deleted.find(mvreg.id()) == deleted.end()) {
                //If the node is not found but it is not deleted, we save de delta for later.
                //We use a CRDT because we can receive multiple deltas for the same edge unordered before we get the node.
                temp_edge_attr[{mvreg.from(), mvreg.to(), mvreg.type()}][mvreg.attr_name()].join( d);

                //If we are deleting the edge
                if (temp_edge_attr[{mvreg.from(), mvreg.to(), mvreg.type()}].find(mvreg.attr_name()) == temp_edge_attr[{mvreg.from(), mvreg.to(), mvreg.type()}].end()) {
                    temp_edge_attr[{mvreg.from(), mvreg.to(), mvreg.type()}].erase(mvreg.attr_name()); //Delete the mvreg if the edge is deleted.
                    if (temp_edge_attr[{mvreg.from(), mvreg.to(), mvreg.type()}].empty())  { temp_edge_attr.erase({mvreg.from(), mvreg.to(), mvreg.type()}); }
                }
            }
        }

        if (ok) {
            if (signal) {
                emit update_node_signal(mvreg.id(), nodes[mvreg.id()].read().begin()->type());
            }
        }

    } catch(const std::exception &e){
        std::cout <<"EXCEPTION: "<<__FILE__ << " " << __FUNCTION__ <<":"<<__LINE__<< " "<< e.what() << std::endl;};


}

void CRDTGraph::join_full_graph(IDL::OrMap &full_graph)
{

    vector<tuple<bool, int, std::string, Node>> updates;
    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        auto m = static_cast<map<int, int>>(full_graph.cbase().cc());
        std::set<pair<int, int>> s;
        for (auto &v : full_graph.cbase().dc())
            s.emplace(std::make_pair(v.first(), v.second()));

        for (auto &[k, val] : full_graph.m())
        {
            auto mv = translateNodeMvIDLtoCRDT(val);
            Node nd = (nodes[k].read().empty()) ? Node() : *nodes[k].read().begin();

            if (deleted.find(k) == deleted.end()) {
                nodes[k].join(mv);
                if (mv.read().empty() or nodes[k].read().empty()) {
                    update_maps_node_delete(k, nd);
                    updates.emplace_back(make_tuple(false, k, "", nd));
                } else {

                    for (auto &[kk,v] : temp_edge[k]) {
                        nodes[k].read_reg().fano()[{std::get<1>(kk), std::get<2>(kk)}].join(v);
                        if (nodes[k].read_reg().fano()[{std::get<1>(kk), std::get<2>(kk)}].read().empty()) {
                            nodes[k].read_reg().fano().erase({std::get<1>(kk), std::get<2>(kk)});
                        }
                        temp_edge[k].erase(kk);
                    }

                    //We have to consume all unordered delta attributes for this node. Normally there won't be any.
                    for (auto &[kk,v] : temp_node_attr[k]) {
                        nodes[k].read_reg().attrs()[kk].join(v);
                        if (nodes[k].read_reg().attrs()[kk].read().empty()) {
                            nodes[k].read_reg().attrs().erase(kk);
                        }
                        temp_node_attr[k].erase(kk);
                    }

                    update_maps_node_insert(k, *mv.read().begin());
                    updates.emplace_back(make_tuple(true, k, nodes[k].read().begin()->type(), nd));
                }
            } else {
                //Delete all temporary information.
                temp_edge.erase(k);
                temp_node_attr.erase(k);
                for (auto &[kk,v] : temp_edge_attr)
                    if (std::get<0>(kk) == k) temp_edge_attr.erase(kk);
            }
        }


    }
    for (auto &[signal, id, type, nd] : updates)
        if (signal) {
            //check what change is joined
            if (nd.attrs() != nodes[id].read().begin()->attrs()) {
                emit update_node_signal(id, nodes[id].read().begin()->type());
            } else if (nd != *nodes[id].read().begin()){
                std::map<std::pair<int, std::string>, mvreg<Edge, int>, pair_hash> diff_remove;
                if (!nodes[id].read().begin()->fano().empty()) {
                    std::set_difference(nd.fano().begin(), nd.fano().end(),
                                        nodes[id].read().begin()->fano().begin(),
                                        nodes[id].read().begin()->fano().end(),
                                        std::inserter(diff_remove, diff_remove.begin()));
                }
                std::map<std::pair<int, std::string>, mvreg<Edge, int>, pair_hash> diff_insert;
                if (!nd.fano().empty()) {
                    std::set_difference(nodes[id].read().begin()->fano().begin(),
                                        nodes[id].read().begin()->fano().end(),
                                        nd.fano().begin(), nd.fano().end(),
                                        std::inserter(diff_insert, diff_insert.begin()));
                }
                for (const auto &[k,v] : diff_remove)
                        emit del_edge_signal(id, k.first, k.second);

                for (const auto &[k,v] : diff_insert) {
                    emit update_edge_signal(id, k.first, k.second);
                }
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
    fullgraph_thread = std::thread(&CRDTGraph::fullgraph_server_thread, this);
}

void CRDTGraph::start_subscription_threads(bool showReceived)
{
    delta_node_thread = std::thread(&CRDTGraph::node_subscription_thread, this, showReceived);
    delta_edge_thread = std::thread(&CRDTGraph::edge_subscription_thread, this, showReceived);
    delta_node_attrs_thread = std::thread(&CRDTGraph::node_attrs_subscription_thread, this, showReceived);
    delta_edge_attrs_thread = std::thread(&CRDTGraph::edge_attrs_subscription_thread, this, showReceived);

}

int CRDTGraph::id() 
{
    return nodes.getId();
}

IDL::DotContext CRDTGraph::context()
{
    IDL::DotContext om_dotcontext;
    for (auto &kv_cc : nodes.context().getCcDc().first) 
    {
        om_dotcontext.cc().emplace(make_pair(kv_cc.first, kv_cc.second));
    }
    for (auto &kv_dc : nodes.context().getCcDc().second)
    {
        IDL::PairInt p_i;
        p_i.first(kv_dc.first);
        p_i.second(kv_dc.second);
        om_dotcontext.dc().push_back(p_i);
    }
    return om_dotcontext;
}

std::map<int,IDL::Mvreg> CRDTGraph::Map()
{
    std::shared_lock<std::shared_mutex>  lock(_mutex);
    std::map<int,IDL::Mvreg>  m;
    for (auto kv : nodes.getMapRef())
    {
        m[kv.first] = translateNodeMvCRDTtoIDL(kv.first, kv.second);
    }
    return m;
}

void CRDTGraph::node_subscription_thread(bool showReceived)
{
    // RTPS Initialize subscriptor
    auto name = __FUNCTION__;
    auto lambda_general_topic = [&, name = name] (eprosima::fastrtps::Subscriber* sub, bool* work, CRDT::CRDTGraph *graph) {
        if (*work) {
            try {
                eprosima::fastrtps::SampleInfo_t m_info;
                IDL::Mvreg sample;
                if (sub->takeNextData(&sample, &m_info)) { // Get sample
                    if(m_info.sampleKind == eprosima::fastrtps::rtps::ALIVE) {
                        //if( m_info.sample_identity.writer_guid().is_on_same_process_as(sub->getGuid()) == false) {
                        if (sample.agent_id() != agent_id) {
                            if (showReceived)  std::cout << name << " Received:" << sample.id() << " node from: " << m_info.sample_identity.writer_guid() << std::endl;
                            graph->join_delta_node(sample);
                        }
                    }
                }
            }
            catch (const std::exception &ex) { cerr << ex.what() << endl; }
        }
    };
    dsrpub_call_node = NewMessageFunctor(this, &work, lambda_general_topic);
	dsrsub_node.init(dsrparticipant.getParticipant(), "DSR", dsrparticipant.getNodeTopicName(), dsrpub_call_node);
}

void CRDTGraph::edge_subscription_thread(bool showReceived)
{
    // RTPS Initialize subscriptor
    auto name = __FUNCTION__;
    auto lambda_general_topic = [&, name = name] (eprosima::fastrtps::Subscriber* sub, bool* work, CRDT::CRDTGraph *graph) {
        if (*work) {
            try {
                eprosima::fastrtps::SampleInfo_t m_info;
                IDL::MvregEdge sample;
                if (sub->takeNextData(&sample, &m_info)) { // Get sample
                    if(m_info.sampleKind == eprosima::fastrtps::rtps::ALIVE) {
                        //if( m_info.sample_identity.writer_guid().is_on_same_process_as(sub->getGuid()) == false) {
                        if (sample.agent_id() != agent_id) {
                            if (showReceived)  std::cout << name  << " Received:" << sample.id() << " node from: " << m_info.sample_identity.writer_guid() << std::endl;
                            graph->join_delta_edge(sample);
                        }
                    }
                }
            }
            catch (const std::exception &ex) { cerr << ex.what() << endl; }
        }
    };
    dsrpub_call_edge = NewMessageFunctor(this, &work, lambda_general_topic);
    dsrsub_edge.init(dsrparticipant.getParticipant(), "DSR_EDGE", dsrparticipant.getEdgeTopicName(), dsrpub_call_edge);
}

void CRDTGraph::edge_attrs_subscription_thread(bool showReceived)
{
    // RTPS Initialize subscriptor
    auto name = __FUNCTION__;
    auto lambda_general_topic = [&, name = name] (eprosima::fastrtps::Subscriber* sub, bool* work, CRDT::CRDTGraph *graph) {
        if (*work) {
            try {
                eprosima::fastrtps::SampleInfo_t m_info;
                IDL::MvregEdgeAttr sample;
                if (sub->takeNextData(&sample, &m_info)) { // Get sample
                    if(m_info.sampleKind == eprosima::fastrtps::rtps::ALIVE) {
                        //if( m_info.sample_identity.writer_guid().is_on_same_process_as(sub->getGuid()) == false) {
                        if (sample.agent_id() != agent_id) {
                            if (showReceived)  std::cout << name  << " Received:" << sample.id() << " node from: " << m_info.sample_identity.writer_guid() << std::endl;
                            graph->join_delta_edge_attr(sample);
                        }
                    }
                }
            }
            catch (const std::exception &ex) { cerr << ex.what() << endl; }
        }
    };
    dsrpub_call_edge_attrs = NewMessageFunctor(this, &work, lambda_general_topic);
    dsrsub_edge_attrs.init(dsrparticipant.getParticipant(), "DSR_EDGE_ATTRS", dsrparticipant.getEdgeAttrTopicName(), dsrpub_call_edge_attrs);
}

void CRDTGraph::node_attrs_subscription_thread(bool showReceived)
{
    // RTPS Initialize subscriptor
    auto name = __FUNCTION__;
    auto lambda_general_topic = [&, name = name] (eprosima::fastrtps::Subscriber* sub, bool* work, CRDT::CRDTGraph *graph) {
        if (*work) {
            try {
                eprosima::fastrtps::SampleInfo_t m_info;
                IDL::MvregNodeAttr sample;
                if (sub->takeNextData(&sample, &m_info)) { // Get sample
                    if(m_info.sampleKind == eprosima::fastrtps::rtps::ALIVE) {
                        //if( m_info.sample_identity.writer_guid().is_on_same_process_as(sub->getGuid()) == false) {
                        if (sample.agent_id() != agent_id) {
                            if (showReceived)  std::cout << name  << " Received:" << sample.id() << " node from: " << m_info.sample_identity.writer_guid() << std::endl;
                            graph->join_delta_node_attr(sample);
                        }
                    }
                }
            }
            catch (const std::exception &ex) { cerr << ex.what() << endl; }
        }
    };
    dsrpub_call_node_attrs = NewMessageFunctor(this, &work, lambda_general_topic);
    dsrsub_node_attrs.init(dsrparticipant.getParticipant(), "DSR_NODE_ATTRS", dsrparticipant.getEdgeAttrTopicName(), dsrpub_call_node_attrs);
}

void CRDTGraph::fullgraph_server_thread() 
{
    std::cout << __FUNCTION__ << "->Entering thread to attend full graph requests" << std::endl;
    // Request Topic
    auto lambda_graph_request = [&] (eprosima::fastrtps::Subscriber* sub, bool* work, CRDT::CRDTGraph *graph) {

        eprosima::fastrtps::SampleInfo_t m_info;
        IDL::GraphRequest sample;
        //readNextData o takeNextData
        if (sub->takeNextData(&sample, &m_info)) { // Get sample
            if(m_info.sampleKind == eprosima::fastrtps::rtps::ALIVE) {
                if( m_info.sample_identity.writer_guid().is_on_same_process_as(sub->getGuid()) == false) {
                    std::cout << " Received Full Graph request: from " << m_info.sample_identity.writer_guid()
                              << std::endl;
                    *work = false;
                    IDL::OrMap mp;
                    mp.id(graph->id());
                    mp.m(graph->Map());
                    mp.cbase(graph->context());
                    std::cout << "nodos enviados: " << mp.m().size()  << std::endl;

                    dsrpub_request_answer.write(&mp);

                    //for (auto &[k, v] : Map())
                    //    std::cout << k << "," << v.dk().ds().begin()->second << std::endl;
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
        IDL::OrMap sample;
        //std::cout << "Mensajes sin leer " << sub->get_unread_count() << std::endl;
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

    std::this_thread::sleep_for(300ms);   // NEEDED ?

    std::cout << " Requesting the complete graph " << std::endl;
    IDL::GraphRequest gr;
    gr.from(agent_name);
    dsrpub_graph_request.write(&gr);

    bool timeout = false;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    while (!sync and !timeout) 
    {
        std::this_thread::sleep_for(500ms);
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        timeout = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() > TIMEOUT*3;
    }
    eprosima::fastrtps::Domain::removeSubscriber(dsrsub_request_answer.getSubscriber());
    return sync;
}


IDL::Mvreg CRDTGraph::translateNodeMvCRDTtoIDL(int id, mvreg<Node, int> &data)
{
    IDL::Mvreg delta_crdt;
    for (auto &kv_dots : data.dk.ds) {
        IDL::PairInt pi;
        pi.first(kv_dots.first.first);
        pi.second(kv_dots.first.second);

        delta_crdt.dk().ds().emplace(make_pair(pi, kv_dots.second.toIDLNode(id)));
    }
    for (auto &kv_cc : data.context().getCcDc().first){
        delta_crdt.dk().cbase().cc().emplace(make_pair(kv_cc.first, kv_cc.second));
    }
    for (auto &kv_dc : data.context().getCcDc().second){
        IDL::PairInt pi;
        pi.first(kv_dc.first);
        pi.second(kv_dc.second);

        delta_crdt.dk().cbase().dc().push_back(pi);
    }
    delta_crdt.id(id);
    delta_crdt.agent_id(agent_id);
    return delta_crdt;
}

mvreg<Node, int> CRDTGraph::translateNodeMvIDLtoCRDT(IDL::Mvreg &data)
{
    // Context
    dotcontext<int> dotcontext_aux;
    std::map<int, int> m;
    for (auto &v : data.dk().cbase().cc())
        m.insert(std::make_pair(v.first, v.second));
    std::set <pair<int, int>> s;
    for (auto &v : data.dk().cbase().dc())
        s.insert(std::make_pair(v.first(), v.second()));
    dotcontext_aux.setContext(m, s);
    // Dots
    std::map <pair<int, int>, Node> ds_aux;
    for (auto &[k,v] : data.dk().ds())
        ds_aux[pair<int, int>(k.first(), k.second())] = std::move(v);
    // Join
    mvreg<Node, int> aw = mvreg<N, int>(data.id());
    aw.dk.c = dotcontext_aux;
    aw.dk.set(ds_aux);
    return aw;
}


IDL::MvregEdgeAttr CRDTGraph::translateEdgeAttrMvCRDTtoIDL(int id, int from, int to,  const std::string& type, const std::string& attr, mvreg<Attribute, int> &data) {
    IDL::MvregEdgeAttr delta_crdt;

    for (auto &kv_dots : data.dk.ds) {
        IDL::PairInt pi;
        pi.first(kv_dots.first.first);
        pi.second(kv_dots.first.second);

        delta_crdt.dk().ds().emplace(make_pair(pi, kv_dots.second.toIDLAttrib()));
    }
    pair<map<int, int>, set<pair<int, int>>> d = data.context().getCcDc();
    for (auto &kv_cc : data.context().getCcDc().first){
        delta_crdt.dk().cbase().cc().emplace(make_pair(kv_cc.first, kv_cc.second));
    }
    for (auto &kv_dc : data.context().getCcDc().second){
        IDL::PairInt pi;
        pi.first(kv_dc.first);
        pi.second(kv_dc.second);

        delta_crdt.dk().cbase().dc().push_back(pi);
    }
    delta_crdt.id(id);
    delta_crdt.attr_name(attr);
    delta_crdt.from(from);
    delta_crdt.to(to);
    delta_crdt.agent_id(agent_id);
    return delta_crdt;

}

mvreg<Attribute, int> CRDTGraph::translateEdgeAttrMvIDLtoCRDT(IDL::MvregEdgeAttr &data) {
    // Context
    dotcontext<int> dotcontext_aux;
    std::map<int, int> m;
    for (auto &v : data.dk().cbase().cc())
        m.insert(std::make_pair(v.first, v.second));
    std::set <pair<int, int>> s;
    for (auto &v : data.dk().cbase().dc())
        s.insert(std::make_pair(v.first(), v.second()));
    dotcontext_aux.setContext(m, s);
    // Dots
    std::map <pair<int, int>, Attribute> ds_aux;
    for (auto &[k,v] : data.dk().ds())
        ds_aux[pair<int, int>(k.first(), k.second())] = std::move(v);
    // Join
    mvreg<Attribute, int> aw = mvreg<Attribute, int>(data.id());
    aw.dk.c = dotcontext_aux;
    aw.dk.set(ds_aux);
    return aw;
}

IDL::MvregNodeAttr CRDTGraph::translateNodeAttrMvCRDTtoIDL(int id, int node, const std::string& attr, mvreg<Attribute, int> &data) {
    IDL::MvregNodeAttr delta_crdt;

    for (auto &kv_dots : data.dk.ds) {
        IDL::PairInt pi;
        pi.first(kv_dots.first.first);
        pi.second(kv_dots.first.second);

        delta_crdt.dk().ds().emplace(make_pair(pi, kv_dots.second.toIDLAttrib()));
    }
    pair<map<int, int>, set<pair<int, int>>> d = data.context().getCcDc();
    for (auto &kv_cc : data.context().getCcDc().first){
        delta_crdt.dk().cbase().cc().emplace(make_pair(kv_cc.first, kv_cc.second));
    }
    for (auto &kv_dc : data.context().getCcDc().second){
        IDL::PairInt pi;
        pi.first(kv_dc.first);
        pi.second(kv_dc.second);

        delta_crdt.dk().cbase().dc().push_back(pi);
    }
    delta_crdt.id(id);
    delta_crdt.attr_name(attr);
    delta_crdt.node(node);
    delta_crdt.agent_id(agent_id);
    return delta_crdt;
}

mvreg<Attribute, int> CRDTGraph::translateNodeAttrMvIDLtoCRDT(IDL::MvregNodeAttr &data) {
    // Context
    dotcontext<int> dotcontext_aux;
    std::map<int, int> m;
    for (auto &v : data.dk().cbase().cc())
        m.insert(std::make_pair(v.first, v.second));
    std::set <pair<int, int>> s;
    for (auto &v : data.dk().cbase().dc())
        s.insert(std::make_pair(v.first(), v.second()));
    dotcontext_aux.setContext(m, s);
    // Dots
    std::map <pair<int, int>, Attribute> ds_aux;
    for (auto &[k,v] : data.dk().ds())
        ds_aux[pair<int, int>(k.first(), k.second())] = std::move(v);
    // Join
    mvreg<Attribute, int> aw = mvreg<Attribute, int>(data.id());
    aw.dk.c = dotcontext_aux;
    aw.dk.set(ds_aux);
    return aw;
}

mvreg<Edge, int> CRDTGraph::translateEdgeMvIDLtoCRDT(IDL::MvregEdge &data) {
    // Context
    dotcontext<int> dotcontext_aux;
    std::map<int, int> m;
    for (auto &v : data.dk().cbase().cc())
        m.insert(std::make_pair(v.first, v.second));
    std::set <pair<int, int>> s;
    for (auto &v : data.dk().cbase().dc())
        s.insert(std::make_pair(v.first(), v.second()));
    dotcontext_aux.setContext(m, s);
    // Dots
    std::map <pair<int, int>, Edge> ds_aux;
    for (auto &[k,v] : data.dk().ds())
        ds_aux[pair<int, int>(k.first(), k.second())] = std::move(v);
    // Join
    mvreg<Edge, int> aw = mvreg<Edge, int>(data.id());
    aw.dk.c = dotcontext_aux;
    aw.dk.set(ds_aux);
    return aw;
}

IDL::MvregEdge CRDTGraph::translateEdgeMvCRDTtoIDL(int id, mvreg<Edge, int> &data) {
    IDL::MvregEdge delta_crdt;

    for (auto &kv_dots : data.dk.ds) {
        IDL::PairInt pi;
        pi.first(kv_dots.first.first);
        pi.second(kv_dots.first.second);

        auto edge = kv_dots.second.toIDLEdge(id);
        delta_crdt.dk().ds().emplace(make_pair(pi, edge));
        delta_crdt.from(edge.from());
        delta_crdt.to(edge.to());
        delta_crdt.type(edge.type());
    }
    pair<map<int, int>, set<pair<int, int>>> d = data.context().getCcDc();
    for (auto &kv_cc : data.context().getCcDc().first){
        delta_crdt.dk().cbase().cc().emplace(make_pair(kv_cc.first, kv_cc.second));
    }
    for (auto &kv_dc : data.context().getCcDc().second){
        IDL::PairInt pi;
        pi.first(kv_dc.first);
        pi.second(kv_dc.second);

        delta_crdt.dk().cbase().dc().push_back(pi);
    }
    delta_crdt.id(id);
    delta_crdt.agent_id(agent_id);
    return delta_crdt;
}
