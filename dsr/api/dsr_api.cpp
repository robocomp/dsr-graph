//
// Created by crivac on 5/02/19.
//

#include "dsr_api.h"
#include <iostream>
#include <unistd.h>
#include <algorithm>

#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/transport/UDPv4TransportDescriptor.h>
#include <fastrtps/Domain.h>

using namespace DSR;


/////////////////////////////////////////////////
///// PUBLIC METHODS
/////////////////////////////////////////////////

DSRGraph::DSRGraph(int root, std::string name, int id, std::string dsr_input_file, RoboCompDSRGetID::DSRGetIDPrxPtr dsr_getid_proxy_) : agent_id(id), agent_name(name) , copy(false) {


    dsr_getid_proxy = dsr_getid_proxy_;
    graph_root = root;
    nodes = Nodes(graph_root);
    utils = std::make_unique<Utilities>(this);
    std::cout << "Agent name: " << agent_name << std::endl;
    work = true;

    // RTPS Create participant 
    auto[suc, participant_handle] = dsrparticipant.init(agent_id);

    // RTPS Initialize publisher with general topic
    dsrpub_node.init(participant_handle, "DSR_NODE", dsrparticipant.getNodeTopicName());
    dsrpub_node_attrs.init(participant_handle, "DSR_NODE_ATTRS", dsrparticipant.getNodeAttrTopicName());
    dsrpub_edge.init(participant_handle, "DSR_EDGE", dsrparticipant.getEdgeTopicName());
    dsrpub_edge_attrs.init(participant_handle, "DSR_EDGE_ATTRS", dsrparticipant.getEdgeAttrTopicName());

    dsrpub_graph_request.init(participant_handle, "DSR_GRAPH_REQUEST", dsrparticipant.getRequestTopicName());
    dsrpub_request_answer.init(participant_handle, "DSR_GRAPH_ANSWER", dsrparticipant.getAnswerTopicName());

    // RTPS Initialize comms threads
    if (dsr_input_file != std::string()) {
        try {
            utils->read_from_json_file(dsr_input_file, insert_node_read_file);
            qDebug() << __FUNCTION__ << "Warning, graph read from file " << QString::fromStdString(dsr_input_file);
        }
        catch (const DSR::DSRException &e) {
            std::cout << e.what() << '\n';
            qFatal("Aborting program. Cannot continue without intial file");
        }
        start_fullgraph_server_thread();
        start_subscription_threads(true);
    } else {
        start_subscription_threads(true);     // regular subscription to deltas
        bool res = start_fullgraph_request_thread();    // for agents that want to request the graph for other agent
        if (res == false) {
            eprosima::fastrtps::Domain::removeParticipant(
                    participant_handle); // Remove a Participant and all associated publishers and subscribers.
            qFatal("DSRGraph aborting: could not get DSR from the network after timeout");  //JC ¿se pueden limpiar aquí cosas antes de salir?

        }
    }
    qDebug() << __FUNCTION__ << "Constructor finished OK";
}

DSRGraph::~DSRGraph() {
    qDebug() << "Removing rtps participant";
    eprosima::fastrtps::Domain::removeParticipant(
            dsrparticipant.getParticipant()); // Remove a Participant and all associated publishers and subscribers.
    if (fullgraph_thread.joinable()) fullgraph_thread.join();
    if (delta_node_thread.joinable()) delta_node_thread.join();
    if (delta_edge_thread.joinable()) delta_edge_thread.join();
    if (delta_node_attrs_thread.joinable()) delta_node_attrs_thread.join();
    if (delta_edge_attrs_thread.joinable()) delta_edge_attrs_thread.join();
}

//////////////////////////////////////
/// NODE METHODS
/////////////////////////////////////

std::optional<DSR::Node> DSRGraph::get_node(const std::string &name) {
    std::shared_lock<std::shared_mutex> lock(_mutex);
    if (name.empty()) return {};
    int id = get_id_from_name(name).value_or(-1);
    std::optional<CRDTNode> n = get_(id);
    if (n.has_value()) return Node(n.value());
    return {};
}

std::optional<DSR::Node> DSRGraph::get_node(int id) {
    std::shared_lock<std::shared_mutex> lock(_mutex);
    std::optional<CRDTNode> n = get_(id);
    if (n.has_value()) return Node(n.value());
    return {};
}


std::tuple<bool, std::optional<IDL::Mvreg>> DSRGraph::insert_node_(const CRDTNode &node) {
    if (deleted.find(node.id()) == deleted.end()) {
        if (!nodes[node.id()].read().empty() and *nodes[node.id()].read().begin() == node) {
            return {true, {}};
        }

        mvreg<CRDTNode, uint32_t> delta = nodes[node.id()].write(node);
        update_maps_node_insert(node.id(), node);

        return {true, translateNodeMvCRDTtoIDL(agent_id, node.id(), delta)};
    }
    return {false, {}};
}


std::optional<uint32_t> DSRGraph::insert_node(Node &node) {
    if (node.id() == -1u) return {};
    std::optional<IDL::Mvreg> aw;

    bool r = false;
    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        //if (id_map.find(node.id()) == id_map.end() and name_map.find(node.name()) == name_map.end()) {
            try{
                if (dsr_getid_proxy != nullptr)
                {
                    int new_node_id = dsr_getid_proxy->getID();
                    node.id(new_node_id);
                    node.name(node.type() + "_" + std::to_string(new_node_id));
                } else
                {
                    qWarning() << __FILE__ << __FUNCTION__ << "Cannot connect to idserver. Aborting";
                    std::terminate();
                }
            }
            catch(const std::exception& e)
            {
                throw std::runtime_error((std::string("Cannot get new id from idserver, check config file ")
                                          + __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__)).data());
            }

            std::tie(r, aw) = insert_node_(user_node_to_crdt(std::move(node)));
        }// else
        //    throw std::runtime_error((std::string("Cannot insert node in G, a node with the same id already exists ")
        //                              + __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__)).data());
    //}
    if (r) {
        if (!copy) {
            if (aw.has_value()) {
                dsrpub_node.write(&aw.value());
                std::cout << "[INSERT NODE] : " << aw->id()<< std::endl;
                emit update_node_signal(node.id(), node.type());
                for (const auto &[k, v]: node.fano()) {
                    std::cout << "  [INSERT EDGE] : " << node.id() << " " << k.first << " type: " << k.second << std::endl;
                    emit update_edge_signal(node.id(), k.first, k.second);
                }
            }
        }
        return node.id();
    }
    return {};
}


std::tuple<bool, std::optional<std::vector<IDL::MvregNodeAttr>>> DSRGraph::update_node_(const CRDTNode &node) {

    if (deleted.find(node.id()) == deleted.end()) {
        if (!nodes[node.id()].read().empty()) {

            vector<IDL::MvregNodeAttr> atts_deltas;
            auto &iter = nodes[node.id()].read_reg().attrs();
            //New attributes and updates.
            for (auto &[k, att]: node.attrs()) {
                //Insertar si inigualdad o inexistencia
                if (iter.find(k) == iter.end()) {
                    mvreg<CRDTAttribute, uint32_t> mv;
                    iter.insert(make_pair(k, mv));
                }
                if (iter[k].read().empty() or *att.read().begin() != *iter.at(k).read().begin()) {
                    auto delta = iter[k].write(*att.read().begin());
                    atts_deltas.emplace_back(translateNodeAttrMvCRDTtoIDL(agent_id, node.id(), node.id(), k, delta));
                }
            }
            //Remove old attributes.
            auto it_a = iter.begin();
            while (it_a != iter.end()) {
                const std::string &k = it_a->first;
                if (node.attrs().find(k) == node.attrs().end()) {
                    auto delta = iter[k].reset();
                    it_a = iter.erase(it_a);
                    atts_deltas.emplace_back(
                            translateNodeAttrMvCRDTtoIDL(node.agent_id(), node.id(), node.id(), k, delta));
                } else {
                    it_a++;
                }
            }

            return {true, atts_deltas};
        }
    }

    return {false, {}};
}


bool DSRGraph::update_node(Node &node) {
    bool r = false;
    std::optional<std::vector<IDL::MvregNodeAttr>> vec_node_attr;

    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        if (deleted.find(node.id()) != deleted.end())
            throw std::runtime_error(
                    (std::string("Cannot update node in G, " + std::to_string(node.id()) + " is deleted")  + __FILE__ + " " +
                     __FUNCTION__ + " " + std::to_string(__LINE__)).data());
        else if ((id_map.find(node.id()) != id_map.end() and id_map[node.id()] != node.name()) or
            (name_map.find(node.name()) != name_map.end() and name_map[node.name()] != node.id()))
            throw std::runtime_error(
                    (std::string("Cannot update node in G, id and name must be unique") + __FILE__ + " " +
                     __FUNCTION__ + " " + std::to_string(__LINE__)).data());
        else if (nodes.getMapRef().find(node.id()) != nodes.getMapRef().end()) {
            //node.agent_id(agent_id);
            std::tie(r, vec_node_attr) = update_node_(user_node_to_crdt(std::move(node)));
        }
    }
    if (r) {
        if (!copy) {
            if (vec_node_attr.has_value()) {
                for (auto &v: vec_node_attr.value()) {
                    std::cout << "[ATTR CHANGE] " << v.attr_name() << std::endl;
                    dsrpub_node_attrs.write(&v);
                }
                emit update_node_signal(node.id(), node.type());
            }
        }
    }
    return r;
}


std::tuple<bool, vector<tuple<uint32_t, uint32_t, std::string>>, std::optional<IDL::Mvreg>, vector<IDL::MvregEdge>>
DSRGraph::delete_node_(uint32_t id) {

    vector<tuple<uint32_t, uint32_t, std::string>> edges_;
    vector<IDL::MvregEdge> aw;

    //1. Get and remove node.
    auto node = get_(id);
    if (!node.has_value()) return make_tuple(false, edges_, std::nullopt, aw);
    for (const auto &v : node.value().fano()) { // Delete all edges from this node.
        std::cout << id << " -> " << v.first.first << " " << v.first.second << std::endl;
        edges_.emplace_back(make_tuple(id, v.first.first, v.first.second));
    }
    // Get remove delta.
    auto delta = nodes[id].reset();
    IDL::Mvreg delta_remove = translateNodeMvCRDTtoIDL(agent_id, id, delta);
    update_maps_node_delete(id, node.value());

    //2. search and remove edges.
    //For each node check if there is an edge to remove.
    for (auto &[k, v] : nodes.getMapRef()) {
        if (edges.find({k, id}) == edges.end()) continue;
        // Remove all edges between them
        auto& visited_node = v.read_reg();
        for (const auto &key : edges[{k, id}]) {

            auto delta_fano = visited_node.fano()[{k, key}].reset();
            aw.emplace_back(translateEdgeMvCRDTtoIDL(agent_id, visited_node.id(), k, key, delta_fano));
            visited_node.fano().erase({k, key});
            edges_.emplace_back(make_tuple(visited_node.id(), id, key));

            edgeType[key].erase({visited_node.id(), id});
        }


        //Remove all from cache
        edges.erase({visited_node.id(), id});
    }
    return make_tuple(true, edges_, delta_remove, aw);

}

bool DSRGraph::delete_node(const std::string &name) {

    bool result = false;
    vector<tuple<uint32_t, uint32_t, std::string>> edges_;
    std::optional<IDL::Mvreg> deleted_node;
    vector<IDL::MvregEdge> aw_;

    std::optional<int> id = {};
    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        id = get_id_from_name(name);
        if (id.has_value()) {
            std::tie(result, edges_, deleted_node, aw_) = delete_node_(id.value());
        } else {
            return false;
        }
    }

    if (result) {
        if (!copy) {
            std::cout << "[DELETE NODE] : " << id.value()<< std::endl;
            emit del_node_signal(id.value());
            dsrpub_node.write(&deleted_node.value());

            for (auto &a  : aw_) {
                std::cout << "  [DELETE EDGE] : " << a.from() << " " << a.to() << " type: " << a.type() << std::endl;
                dsrpub_edge.write(&a);
            }
            for (auto &[id0, id1, label] : edges_)
                    emit del_edge_signal(id0, id1, label);
        }
        return true;
    }
    return false;

}

bool DSRGraph::delete_node(uint32_t id) {

    bool result;
    vector<tuple<uint32_t, uint32_t, std::string>> edges_;
    std::optional<IDL::Mvreg> deleted_node;
    vector<IDL::MvregEdge> aw_;
    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        if (in(id)) {
            std::tie(result, edges_, deleted_node, aw_) = delete_node_(id);
        } else { return false; }
    }

    if (result) {
        if (!copy) {

            std::cout << "[DELETE NODE] : " << id<< std::endl;
            emit del_node_signal(id);
            dsrpub_node.write(&deleted_node.value());

            for (auto &a  : aw_) {
                std::cout << "  [DELETE EDGE] : " << a.from() << " " << a.to() << " type: " << a.type() << std::endl;
                dsrpub_edge.write(&a);
            }
            for (auto &[id0, id1, label] : edges_)
                    emit del_edge_signal(id0, id1, label);
        }
        return true;
    }

    return false;
}


std::vector<DSR::Node> DSRGraph::get_nodes_by_type(const std::string &type) {
    std::shared_lock<std::shared_mutex> lock(_mutex);
    std::vector<Node> nodes_;
    if (nodeType.find(type) != nodeType.end()) {
        for (auto &id: nodeType[type]) {
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
std::optional<CRDTEdge> DSRGraph::get_edge_(uint32_t from, uint32_t to, const std::string &key) {
    std::shared_lock<std::shared_mutex> lock(_mutex);
    if (in(from) && in(to)) {
        auto n = get_(from);
        if (n.has_value()) {
            IDL::EdgeKey ek;
            ek.to(to);
            ek.type(key);
            auto edge = n.value().fano().find({to, key});
            if (edge != n.value().fano().end()) {
                return *edge->second.read().begin();
            }
        }
        std::cout << __FUNCTION__ << ":" << __LINE__ << " Error obteniedo edge from: " << from << " to: " << to
                  << " key " << key << endl;
    }
    return {};
}

std::optional<DSR::Edge> DSRGraph::get_edge(const std::string &from, const std::string &to, const std::string &key) {
    std::shared_lock<std::shared_mutex> lock(_mutex);
    std::optional<uint32_t> id_from = get_id_from_name(from);
    std::optional<uint32_t> id_to = get_id_from_name(to);
    if (id_from.has_value() and id_to.has_value()) {
        auto edge_opt = get_edge_(id_from.value(), id_to.value(), key);
        if (edge_opt.has_value()) return Edge(edge_opt.value());
    }
    return {};
}

std::optional<DSR::Edge> DSRGraph::get_edge(uint32_t from, uint32_t to, const std::string &key) {
    auto edge_opt = get_edge_(from, to, key);
    if (edge_opt.has_value()) return Edge(edge_opt.value());
    return {};
}

std::optional<Edge> DSRGraph::get_edge(const Node& n, const std::string& to, const std::string& key)
{
    std::optional<uint32_t> id_to = get_id_from_name(to);
    if (id_to.has_value()) {
        return (n.fano().find({id_to.value(), key}) != n.fano().end()) ?  std::make_optional(n.fano().find({id_to.value(), key})->second) : std::nullopt;
    }
    return {};
}

std::optional<Edge> DSRGraph::get_edge(const Node &n, uint32_t to, const std::string& key)
{
    EdgeKey ek; ek.to(to); ek.type(key);
    return (n.fano().find({to, key}) != n.fano().end()) ?  std::make_optional(n.fano().find({to, key})->second) : std::nullopt;
};

std::tuple<bool, std::optional<IDL::MvregEdge>, std::optional<std::vector<IDL::MvregEdgeAttr>>>
DSRGraph::insert_or_assign_edge_(const CRDTEdge &attrs, uint32_t from, uint32_t to) {

    std::optional<IDL::MvregEdge> delta_edge;
    std::optional<std::vector<IDL::MvregEdgeAttr>> delta_attrs;


    if (nodes.getMapRef().find(from) != nodes.getMapRef().end()) {
        auto &node = nodes[from].read_reg();
        //check if we are creating an edge or we are updating it.
        if (node.fano().find({to, attrs.type()}) != node.fano().end()) {//Update
            vector<IDL::MvregEdgeAttr> atts_deltas;
            auto iter = nodes[from].read_reg().fano().find({attrs.to(), attrs.type()});
            if (iter != nodes[from].read_reg().fano().end()) {
                auto &iter_edge = iter->second.read_reg().attrs();
                for (auto[k, att]: attrs.attrs()) {
                    //comparar igualdad o inexistencia
                    if (iter_edge.find(k) == iter_edge.end()) {
                        mvreg<CRDTAttribute, uint32_t> mv;
                        iter_edge.insert({k, mv});
                    }
                    if (iter_edge[k].read().empty() or
                        *att.read().begin() !=
                        *iter_edge.at(k).read().begin()) {
                        auto delta = iter_edge.at(k).write(att.read_reg());//*iter_edge.at(k).read().begin());
                        atts_deltas.emplace_back(
                                translateEdgeAttrMvCRDTtoIDL(agent_id, from, from, to, attrs.type(), k, delta));
                    }
                }
                auto it = iter_edge.begin();
                while (it != iter_edge.end()) {
                    if (attrs.attrs().find(it->first) == attrs.attrs().end()) {
                        it = iter_edge.erase(it);
                    } else {
                        it++;
                    }
                }

                return {true, {}, atts_deltas};
            }
        } else { // Insert
            mvreg<CRDTEdge, uint32_t> mv;
            node.fano().insert({{to, attrs.type()}, mv});
            auto delta = node.fano()[{to, attrs.type()}].write(attrs);
            update_maps_node_insert(from, node);
            return {true, translateEdgeMvCRDTtoIDL(agent_id, from, to, attrs.type(), delta), {}};
        }
    }
    return {false, {}, {}};
}


bool DSRGraph::insert_or_assign_edge(const Edge &attrs) {
    bool r = false;
    std::optional<IDL::MvregEdge> delta_edge;
    std::optional<std::vector<IDL::MvregEdgeAttr>> delta_attrs;


    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        int from = attrs.from();
        int to = attrs.to();
        if (in(from) && in(to)) {
            std::tie(r, delta_edge, delta_attrs) = insert_or_assign_edge_(user_edge_to_crdt(attrs), from, to);
        } else {
            std::cout << __FUNCTION__ << ":" << __LINE__ << " Error. ID:" << from << " or " << to
                      << " not found. Cant update. " << std::endl;
            return false;
        }
    }
    if (r) {
        if (!copy) {

            if (delta_edge.has_value()) { //Insert
                dsrpub_edge.write(&delta_edge.value());
            }
            if (delta_attrs.has_value()) { //Update
                for (auto &d : delta_attrs.value())
                    dsrpub_edge_attrs.write(&d);
            }
            std::cout << "[INSERT EDGE] : " << attrs.from() << " " << attrs.to() << " type: " << attrs.type() << std::endl;
            emit update_edge_signal(attrs.from(), attrs.to(), attrs.type());
        }
    }
    return true;
}


void DSRGraph::insert_or_assign_edge_RT(Node &n, uint32_t to, std::vector<float> &&trans, std::vector<float> &&rot_euler) {

    bool r1 = false;
    bool r2 = false;
    bool no_send = true;

    std::optional<IDL::MvregEdge> node1_insert;
    std::optional<vector<IDL::MvregEdgeAttr>> node1_update;
    std::optional<vector<IDL::MvregNodeAttr>> node2;
    std::optional<CRDTNode> to_n;
    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        if (in(to)) {
            CRDTEdge e; e.to(to);  e.from(n.id()); e.type("RT"); e.agent_id(agent_id);
            CRDTAttribute tr; tr.type(3); tr.val(CRDTValue(std::move(trans))); tr.timestamp(get_unix_timestamp());
            CRDTAttribute rot; rot.type(3); rot.val(CRDTValue(std::move(rot_euler))); rot.timestamp(get_unix_timestamp());
            auto [it, new_el] = e.attrs().emplace("rotation_euler_xyz", mvreg<CRDTAttribute, uint32_t> ());
            it->second.write(rot);
            auto [it2, new_el2] = e.attrs().emplace("translation", mvreg<CRDTAttribute, uint32_t> ());
            it2->second.write(tr);


            to_n = get_(to).value();
            if (auto x = get_crdt_attrib_by_name<parent_att>(to_n.value()); x.has_value()) {
                if ( x.value() != n.id()) {
                    no_send = !modify_attrib_local<parent_att>(to_n.value(), n.id());
                }
            } else {
                no_send = !add_attrib_local<parent_att>(to_n.value(), n.id());
            }

            if (auto x = get_crdt_attrib_by_name<level_att>(to_n.value()); x.has_value()) {
                if (x.value() != get_node_level(n).value() + 1) {
                    no_send = !modify_attrib_local<level_att>(to_n.value(),  get_node_level(n).value() + 1 );
                }
            } else {
                no_send = add_attrib_local<level_att>(to_n.value(),  get_node_level(n).value() + 1 );
            }

            //Check if RT edge exist.
            if (n.fano().find({to, "RT"}) == n.fano().end()) {
                //Create -> from: IDL::MvregEdge, to: vector<IDL::MvregNodeAttr>
                std::tie(r1, node1_insert, std::ignore) = insert_or_assign_edge_(e, n.id(), to);
                if (!no_send) std::tie(r2, node2) = update_node_(to_n.value());

            } else {
                //Update -> from: IDL::MvregEdgeAttr, to: vector<IDL::MvregNodeAttr>

                std::tie(r1, std::ignore, node1_update) = insert_or_assign_edge_(e, n.id(), to);
                if (!no_send) std::tie(r2, node2) = update_node_(to_n.value());

            }
            if (!r1) {
                throw std::runtime_error(
                        "Could not insert Node " + std::to_string(n.id()) + " in G in insert_or_assign_edge_RT() " +
                        __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__));
            }
            if (!r2 and !no_send) {
                throw std::runtime_error(
                        "Could not insert Node " + std::to_string(to_n->id()) + " in G in insert_or_assign_edge_RT() " +
                        __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__));
            }
        } else
            throw std::runtime_error(
                    "Destination node " + std::to_string(to) + " not found in G in insert_or_assign_edge_RT() " +
                    __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__));
    }
    if (!copy) {

        if (node1_insert.has_value())
            dsrpub_edge.write(&node1_insert.value());
        if (node1_update.has_value())
            for (auto &d : node1_update.value())
                dsrpub_edge_attrs.write(&d);
        if (!no_send and node2.has_value())
                for (auto &d : node2.value())
                    dsrpub_node_attrs.write(&d);

        emit update_edge_signal(n.id(), to, "RT");
        if (!no_send) emit update_node_signal(to_n->id(), to_n->type());
    }
}

void DSRGraph::insert_or_assign_edge_RT(Node &n, uint32_t to, const std::vector<float> &trans,
                                        const std::vector<float> &rot_euler) {


    bool r1 = false;
    bool r2 = false;
    bool no_send = true;

    std::optional<IDL::MvregEdge> node1_insert;
    std::optional<vector<IDL::MvregEdgeAttr>> node1_update;
    std::optional<vector<IDL::MvregNodeAttr>> node2;
    std::optional<CRDTNode> to_n;
    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        if (in(to)) {
            CRDTEdge e; e.to(to);  e.from(n.id()); e.type("RT"); e.agent_id(agent_id);
            CRDTAttribute tr; tr.type(3); tr.val(CRDTValue(trans)); tr.timestamp(get_unix_timestamp());
            CRDTAttribute rot; rot.type(3); rot.val(CRDTValue(rot_euler)); rot.timestamp(get_unix_timestamp());
            auto [it, new_el] = e.attrs().emplace("rotation_euler_xyz", mvreg<CRDTAttribute, uint32_t> ());
            it->second.write(rot);
            auto [it2, new_el2] = e.attrs().emplace("translation", mvreg<CRDTAttribute, uint32_t> ());
            it2->second.write(tr);


            to_n = get_(to).value();
            if (auto x = get_crdt_attrib_by_name<parent_att>(to_n.value()); x.has_value()) {
                if ( x.value() != n.id()) {
                    no_send = !modify_attrib_local<parent_att>(to_n.value(), n.id());
                }
            } else {
                no_send = !add_attrib_local<parent_att>(to_n.value(), n.id());
            }

            if (auto x = get_crdt_attrib_by_name<level_att>(to_n.value()); x.has_value()) {
                if (x.value() != get_node_level(n).value() + 1) {
                    no_send = !modify_attrib_local<level_att>(to_n.value(),  get_node_level(n).value() + 1 );
                }
            } else {
                no_send = add_attrib_local<level_att>(to_n.value(),  get_node_level(n).value() + 1 );
            }

            //Check if RT edge exist.
            if (n.fano().find({to, "RT"}) == n.fano().end()) {
                //Create -> from: IDL::MvregEdge, to: vector<IDL::MvregNodeAttr>
                std::tie(r1, node1_insert, std::ignore) = insert_or_assign_edge_(e, n.id(), to);
                if (!no_send) std::tie(r2, node2) = update_node_(to_n.value());

            } else {
                //Update -> from: IDL::MvregEdgeAttr, to: vector<IDL::MvregNodeAttr>

                std::tie(r1, std::ignore, node1_update) = insert_or_assign_edge_(e, n.id(), to);
                if (!no_send) std::tie(r2, node2) = update_node_(to_n.value());

            }
            if (!r1) {
                throw std::runtime_error(
                        "Could not insert Node " + std::to_string(n.id()) + " in G in insert_or_assign_edge_RT() " +
                        __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__));
            }
            if (!r2 and !no_send) {
                throw std::runtime_error(
                        "Could not insert Node " + std::to_string(to_n->id()) + " in G in insert_or_assign_edge_RT() " +
                        __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__));
            }
        } else
            throw std::runtime_error(
                    "Destination node " + std::to_string(to) + " not found in G in insert_or_assign_edge_RT() " +
                    __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__));
        }

    if (!copy) {

        if (node1_insert.has_value())
            dsrpub_edge.write(&node1_insert.value());
        if (node1_update.has_value())
            for (auto &d : node1_update.value())
                dsrpub_edge_attrs.write(&d);
        if (!no_send and node2.has_value())
            for (auto &d : node2.value())
                dsrpub_node_attrs.write(&d);

        emit update_edge_signal(n.id(), to, "RT");
        if (!no_send) emit update_node_signal(to_n->id(), to_n->type());
    }


}


std::optional<IDL::MvregEdge> DSRGraph::delete_edge_(uint32_t from, uint32_t to, const std::string &key) {
    if (nodes.getMapRef().find(from) != nodes.getMapRef().end()) {
        auto &node = nodes[from].read_reg();
        if (node.fano().find({to, key}) != node.fano().end()) {
            auto delta = node.fano().at({to, key}).reset();
            node.fano().erase({to, key});
            update_maps_edge_delete(from, to, key);
            std::cout << "DELETE EDGE: " << std::boolalpha <<(nodes[from].read_reg().fano().find({to, key}) == nodes[from].read_reg().fano().end())<<std::endl ;
            //node.value().agent_id(agent_id);
            return translateEdgeMvCRDTtoIDL(agent_id, from, to, key, delta);
        }
    }
    return {};
}

bool DSRGraph::delete_edge(uint32_t from, uint32_t to, const std::string &key) {

    std::optional<IDL::MvregEdge> delta;
    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        if (!in(from) || !in(to)) return false;
        delta = delete_edge_(from, to, key);
    }
    if (delta.has_value()) {
        if (!copy) {
            std::cout << "[DELETE EDGE] : " <<from << " " << to << " type: " << key << std::endl;
            emit del_edge_signal(from, to, key);
            dsrpub_edge.write(&delta.value());
        }
        return true;
    }
    return false;

}

bool DSRGraph::delete_edge(const std::string &from, const std::string &to, const std::string &key) {

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
        if (!copy) {
            std::cout << "[DELETE EDGE] : " <<id_from.value() << " " << id_to.value() << " type: " << key << std::endl;
            emit del_edge_signal(id_from.value(), id_to.value(), key);
            dsrpub_edge.write(&delta.value());
        }
        return true;
    }
    return false;
}


std::vector<DSR::Edge> DSRGraph::get_node_edges_by_type(const Node &node, const std::string &type) {
    std::vector<Edge> edges_;
    for (auto &[key, edge] : node.fano())
        if (key.second == type)
            edges_.emplace_back(Edge(edge));
    return edges_;
}

std::vector<DSR::Edge> DSRGraph::get_edges_by_type(const std::string &type) {
    std::shared_lock<std::shared_mutex> lock(_mutex);
    std::vector<Edge> edges_;
    if (edgeType.find(type) != edgeType.end()) {
        for (auto &[from, to] : edgeType[type]) {
            auto n = get_edge_(from, to, type);
            if (n.has_value())
                edges_.emplace_back(Edge(n.value()));
        }
    }
    return edges_;
}

std::vector<DSR::Edge> DSRGraph::get_edges_to_id(uint32_t id) {
    std::shared_lock<std::shared_mutex> lock(_mutex);
    std::vector<Edge> edges_;
    for (const auto &[key, types] : edges) {
        auto[from, to] = key;
        if (to == id) {
            for (const std::string &type : types) {
                auto n = get_edge_(from, to, type);
                if (n.has_value())
                    edges_.emplace_back(Edge(n.value()));
            }
        }
    }
    return edges_;
}

    std::optional<std::map<std::pair<uint32_t, std::string>, DSR::Edge>> DSRGraph::get_edges(uint32_t id) {
    //std::unordered_map<std::pair<uint32_t, std::string>, CRDTEdge, pair_hash> pa;
    std::shared_lock<std::shared_mutex> lock(_mutex);
    std::optional<Node> n = get_node(id);
    if (n.has_value()) {
        return n->fano();
    }
    return std::nullopt;

};

DSR::Edge DSRGraph::get_edge_RT(const Node &n, uint32_t to) {
    auto edges = n.fano();
    auto res = edges.find({to, "RT"});
    if (res != edges.end())
        return res->second;
    else
        throw std::runtime_error("Could not find edge " + std::to_string(to) + " in node " + std::to_string(n.id()) +
                                 " in edge_to_RTMat() " + __FILE__ + " " + __FUNCTION__ + " " +
                                 std::to_string(__LINE__));
}

RTMat DSRGraph::get_edge_RT_as_RTMat(const Edge &edge) {
    auto r = get_attrib_by_name<rotation_euler_xyz_att>(edge);
    auto t =  get_attrib_by_name<translation_att>(edge);
    if (r.has_value() and t.has_value())
        return RTMat{r->get()[0], r->get()[1], r->get()[2], t->get()[0], t->get()[1], t->get()[2]};
    else
        throw std::runtime_error(
                "Could not find required attributes in node " + edge.type() + " " + std::to_string(edge.to()) +
                " in get_edge_RT as_RTMat() " + __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__));

}

RTMat DSRGraph::get_edge_RT_as_RTMat(Edge &&edge) {
    auto r = get_attrib_by_name<rotation_euler_xyz_att>(edge);
    auto t =  get_attrib_by_name<translation_att>(edge);
    if (r.has_value() and t.has_value())
        return RTMat{r->get()[0], r->get()[1], r->get()[2], t->get()[0], t->get()[1], t->get()[2]};
    else
        throw std::runtime_error(
                "Could not find required attributes in node " + edge.type() + " " + std::to_string(edge.to()) +
                " in get_edge_RT as_RTMat() " + __FILE__ + " " + __FUNCTION__ + " " + std::to_string(__LINE__));
}


/////////////////////////////////////////////////
///// Utils
/////////////////////////////////////////////////

std::map<uint32_t, DSR::Node> DSRGraph::getCopy() const {
    std::map<uint32_t, Node> mymap;
    std::shared_lock<std::shared_mutex> lock(_mutex);
    for (auto &[key, val] : nodes.getMap())
        mymap[key] = *val.read().begin();

    return mymap;
}

std::vector<uint32_t> DSRGraph::getKeys() const {
    std::vector<uint32_t> keys;
    std::shared_lock<std::shared_mutex> lock(_mutex);

    for (auto &[key, val] : nodes.getMap())
        keys.emplace_back(key);

    return keys;
}


//////////////////////////////////////////////////////////////////////////////
/////  CORE
//////////////////////////////////////////////////////////////////////////////

std::optional<CRDTNode> DSRGraph::get(uint32_t id) {
    std::shared_lock<std::shared_mutex> lock(_mutex);
    return get_(id);
}

std::optional<CRDTNode> DSRGraph::get_(uint32_t id) {

    if (in(id)) {
        if (!nodes[id].read().empty()) {
            return make_optional(*nodes[id].read().begin());
        }
    }
    return {};
}

std::optional<std::int32_t> DSRGraph::get_node_level(const Node &n) {
    return get_attrib_by_name<level_att>(n);
}

std::optional<std::uint32_t> DSRGraph::get_parent_id(const Node &n) {
    return get_attrib_by_name<parent_att>(n);
}

std::optional<DSR::Node> DSRGraph::get_parent_node(const Node &n) {
    auto p = get_attrib_by_name<parent_att>(n);
    if (p.has_value()) {
        std::shared_lock<std::shared_mutex> lock(_mutex);
        return get_(p.value());
    }
    return {};
}


std::string DSRGraph::get_node_type(Node &n) {
    return n.type();
}

inline void DSRGraph::update_maps_node_delete(uint32_t id, const CRDTNode &n) {
    nodes.erase(id);
    name_map.erase(id_map[id]);
    id_map.erase(id);
    deleted.insert(id);

    if (nodeType.find(n.type()) != nodeType.end())
        nodeType[n.type()].erase(id);

    for (const auto &[k, v] : n.fano()) {
        edges[{id, v.read().begin()->to()}].erase(k.second);
        if (edges[{id, k.first}].empty()) edges.erase({id, k.first});
        edgeType[k.second].erase({id, k.first});
    }
}

inline void DSRGraph::update_maps_node_insert(uint32_t id, const CRDTNode &n) {
    name_map[n.name()] = id;
    id_map[id] = n.name();
    nodeType[n.type()].emplace(id);

    for (const auto &[k, v] : n.fano()) {
        edges[{id, k.first}].insert(k.second);
        edgeType[k.second].insert({id, k.first});
    }
}


inline void DSRGraph::update_maps_edge_delete(uint32_t from, uint32_t to, const std::string &key) {
    edges[{from, to}].erase(key);
    if (edges[{from, to}].empty()) edges.erase({from, to});
    edgeType[key].erase({from, to});
}

inline void DSRGraph::update_maps_edge_insert(uint32_t from, uint32_t to, const std::string &key) {
    edges[{from, to}].insert(key);
    edgeType[key].insert({from, to});
}


inline std::optional<uint32_t> DSRGraph::get_id_from_name(const std::string &name) {
    auto v = name_map.find(name);
    if (v != name_map.end()) return v->second;
    return {};
}

std::optional<std::string> DSRGraph::get_name_from_id(uint32_t id) {
    auto v = id_map.find(id);
    if (v != id_map.end()) return v->second;
    return {};
}

size_t DSRGraph::size() {
    std::shared_lock<std::shared_mutex> lock(_mutex);
    return nodes.getMapRef().size();
};

bool DSRGraph::in(uint32_t id) const {
    return nodes.in(id);
}

bool DSRGraph::empty(const uint32_t &id) {
    if (nodes.in(id)) {
        return nodes[id].read().empty();
    } else
        return false;
}

void DSRGraph::join_delta_node(IDL::Mvreg &mvreg) {
    CRDTNode nd;
    try {

        bool signal = false, ok = false;
        auto d = translateNodeMvIDLtoCRDT(mvreg);
        {
            std::unique_lock<std::shared_mutex> lock(_mutex);
            if (deleted.find(mvreg.id()) == deleted.end()) {
                ok = true;
                nd = (nodes[mvreg.id()].read().empty()) ?
                              CRDTNode() : *nodes[mvreg.id()].read().begin();

                nodes[mvreg.id()].join(d);
                if (nodes[mvreg.id()].read().empty() or mvreg.dk().ds().empty()) {
                    nodes.erase(mvreg.id());
                    std::cout << "JOIN REMOVE" << std::endl;
                    //Delete all nodes and attributes received out of order
                    //temp_edge.erase(mvreg.id());
                    //temp_node_attr.erase(mvreg.id());
                    //Update Maps
                    update_maps_node_delete(mvreg.id(), nd);
                } else {
                    std::cout << "JOIN INSERT" << std::endl;
                    signal = true;
                    //We have to consume all unordered delta edges for this node. Normally there won't be any.
                    /*for (auto &[k, v] : temp_edge[mvreg.id()]) {
                        nodes[mvreg.id()].read_reg().fano()[{std::get<1>(k), std::get<2>(k)}].join(v);
                        if (nodes[mvreg.id()].read_reg().fano()[{std::get<1>(k), std::get<2>(k)}].read().empty()) {
                            nodes[mvreg.id()].read_reg().fano().erase({std::get<1>(k), std::get<2>(k)});
                        }
                        temp_edge[mvreg.id()].erase(k);
                    }

                    //We have to consume all unordered delta attributes for this node. Normally there won't be any.
                    for (auto &[k, v] : temp_node_attr[mvreg.id()]) {
                        nodes[mvreg.id()].read_reg().attrs()[k].join(v);
                        if (nodes[mvreg.id()].read_reg().attrs()[k].read().empty()) {
                            nodes[mvreg.id()].read_reg().attrs().erase(k);
                        }
                        temp_node_attr[mvreg.id()].erase(k);
                    }*/

                    update_maps_node_insert(mvreg.id(), *nodes[mvreg.id()].read().begin());
                }
            }
        }

        if (ok) {
            if (signal) {
                std::cout << "[INSERT NODE] : " << mvreg.id() << " type: " << nodes[mvreg.id()].read().begin()->type() << std::endl;
                emit update_node_signal(mvreg.id(), nodes[mvreg.id()].read().begin()->type());
                for (auto &[k,v] : nodes[mvreg.id()].read().begin()->fano()) {
                    std::cout << "[INSERT EDGE] : " << mvreg.id() << "" <<  k.first << " type: " << k.second <<  std::endl;
                    emit update_edge_signal(mvreg.id(), k.first, k.second);
                }
            } else {
                std::cout << "[DELETE NODE] : " << mvreg.id() << std::endl;
                emit del_node_signal(mvreg.id());
                for (auto &node: nd.fano()) {
                    std::cout << "  [DELETE EDGE] : " << node.second.read_reg().from() << " " <<  node.second.read_reg().to()<<  " type: " << node.second.read_reg().type()<<  std::endl;
                    emit del_edge_signal(node.second.read_reg().from(), node.second.read_reg().to(), node.second.read_reg().type());
                }

                for (const auto &[key, types] : edges)
                {
                    auto [from, to] = key;
                    if (to == nd.id()) {
                        for (const std::string& type : types) {
                            std::cout << "  [DELETE EDGE] : " << from << "" <<  to << " type: " << type <<  std::endl;
                            emit del_edge_signal(from, to, type);
                        }
                    }
                }
            }
        }

    } catch (const std::exception &e) {
        std::cout << "EXCEPTION: " << __FILE__ << " " << __FUNCTION__ << ":" << __LINE__ << " " << e.what()
                  << std::endl;
    };

}


void DSRGraph::join_delta_edge(IDL::MvregEdge &mvreg) {
    try {
        bool signal = false, ok = false;
        auto d = translateEdgeMvIDLtoCRDT(mvreg);
        {
            std::unique_lock<std::shared_mutex> lock(_mutex);

            //Check if the node where we are joining the edge exist.
            if (nodes.getMapRef().find(mvreg.id()) != nodes.getMapRef().end()) {
                ok = true;
                auto &n = nodes[mvreg.id()].read_reg();
                n.fano()[{mvreg.to(), mvreg.type()}].join(d);

                //Check if we are inserting or deleting.
                if (mvreg.dk().ds().empty() or n.fano().find({mvreg.to(), mvreg.type()}) == n.fano().end()) { //Remove
                    n.fano().erase({mvreg.to(), mvreg.type()});
                    //Delete received items out of order
                    //Update maps
                    update_maps_edge_delete(mvreg.from(), mvreg.to(), mvreg.type());
                } else { //Insert
                    signal = true;

                    /*
                    //We have to consume all unordered delta attributes for this edge. Normally there won't be any.
                    for (auto &[k, v] : temp_edge_attr[{mvreg.from(), mvreg.to(), mvreg.type()}]) {
                        n.fano()[{mvreg.to(), mvreg.type()}].read_reg().attrs()[k].join(v);
                        if (n.fano()[{mvreg.to(), mvreg.type()}].read_reg().attrs()[k].read().empty()) {
                            n.fano()[{mvreg.to(), mvreg.type()}].read_reg().attrs().erase(k);
                        }
                        temp_edge_attr[{mvreg.from(), mvreg.to(), mvreg.type()}].erase(k);
                    }
                     */
                    //Update maps
                    update_maps_edge_insert(mvreg.from(), mvreg.to(), mvreg.type());
                }
                //temp_edge_attr.erase({mvreg.from(), mvreg.to(), mvreg.type()});
                //temp_edge.erase(mvreg.from());

            } else if (deleted.find(mvreg.id()) == deleted.end()) {
                //If the node is not found but it is not deleted, we save de delta for later.
                //We use a CRDT because we can receive multiple deltas for the same edge unordered before we get the node.
                /*temp_edge[mvreg.from()][{mvreg.from(), mvreg.to(), mvreg.type()}].join(d);

                //If we are deleting the edge
                if (temp_edge[mvreg.from()].find({mvreg.from(), mvreg.to(), mvreg.type()}) ==
                    temp_edge[mvreg.from()].end()) {
                    temp_edge[mvreg.from()].erase(
                            {mvreg.from(), mvreg.to(), mvreg.type()}); //Delete the mvreg if the edge is deleted.
                    if (temp_edge[mvreg.id()].empty()) { temp_edge.erase(mvreg.id()); }
                    //temp_edge_attr.erase({mvreg.from(), mvreg.to(), mvreg.type()});
                } else {*/
                    /*
                    //Consume al attributes
                    for (auto &[k, v] : temp_edge_attr[{mvreg.from(), mvreg.to(), mvreg.type()}]) {
                        temp_edge[mvreg.from()][{mvreg.from(), mvreg.to(), mvreg.type()}].read_reg().attrs()[k].join(v);
                        if (temp_edge[mvreg.from()][{mvreg.from(), mvreg.to(),
                                                     mvreg.type()}].read_reg().attrs()[k].read().empty()) {
                            temp_edge[mvreg.from()][{mvreg.from(), mvreg.to(), mvreg.type()}].read_reg().attrs().erase(
                                    k);
                        }
                        temp_edge_attr[{mvreg.from(), mvreg.to(), mvreg.type()}].erase(k);
                    }

                }*/
            }
        }

        if (ok) {
            if (signal) {
                std::cout << "[INSERT EDGE] : " << mvreg.from() << " " <<  mvreg.to() <<  " type: " << mvreg.type()<< std::endl;
                emit update_edge_signal(mvreg.from(), mvreg.to(), mvreg.type());
            } else {
                std::cout << "DELETE EDGE: " << std::boolalpha <<(nodes[mvreg.from()].read_reg().fano().find({mvreg.to(), mvreg.type()}) == nodes[mvreg.from()].read_reg().fano().end())<<std::endl ;
                std::cout << "[DELETE EDGE] : " << mvreg.from() << " " <<  mvreg.to() << " type: " << mvreg.type()<< std::endl;
                emit del_edge_signal(mvreg.from(), mvreg.to(), mvreg.type());
            }
        }

    } catch (const std::exception &e) {
        std::cout << "EXCEPTION: " << __FILE__ << " " << __FUNCTION__ << ":" << __LINE__ << " " << e.what()
                  << std::endl;
    };


}


void DSRGraph::join_delta_node_attr(IDL::MvregNodeAttr &mvreg) {

    try {
        bool ok = false;
        auto d = translateNodeAttrMvIDLtoCRDT(mvreg);
        {
            std::unique_lock<std::shared_mutex> lock(_mutex);
            //Check if the node where we are joining the edge exist.
            if (deleted.find(mvreg.id()) == deleted.end() and nodes.getMapRef().find(mvreg.id()) != nodes.getMapRef().end()) {
                ok = true;
                auto &n = nodes[mvreg.id()].read_reg();
                if (n.attrs().find(mvreg.attr_name()) == n.attrs().end()) {
                    ::mvreg<CRDTAttribute, uint32_t> new_mv;
                    n.attrs().insert({mvreg.attr_name(), new_mv});
                }
                std::cout << "JOINING NODE ATTRIBUTE: " << mvreg.attr_name() << std::endl;
                n.attrs()[mvreg.attr_name()].join(d);

                //Check if we are inserting or deleting.
                if (mvreg.dk().ds().empty() or n.attrs().find(mvreg.attr_name()) == n.attrs().end()) { //Remove
                    n.attrs().erase(mvreg.attr_name());
                    //Update maps
                    update_maps_node_insert(mvreg.id(), n);
                } else { //Insert
                    //Update maps
                    update_maps_node_insert(mvreg.id(), n);
                }
                //temp_node_attr.erase(mvreg.id());
            } /*else if (deleted.find(mvreg.id()) == deleted.end()) {
                std::cout << "!!!!!!!!!!!!!!!!!Saving for later" << std::endl;
                //If the node is not found but it is not deleted, we save de delta for later.
                //We use a CRDT because we can receive multiple deltas for the same edge unordered before we get the node.
                if (temp_node_attr[mvreg.id()].find(mvreg.attr_name()) == temp_node_attr[mvreg.id()].end()) {
                    ::mvreg<CRDTAttribute, uint32_t> new_mv;
                    temp_node_attr[mvreg.id()].insert({mvreg.attr_name(), new_mv});
                }
                temp_node_attr[mvreg.id()][mvreg.attr_name()].join(d);

                //If we are deleting the edge
                if (temp_node_attr[mvreg.id()].find(mvreg.attr_name()) == temp_node_attr[mvreg.id()].end()) {
                    temp_node_attr[mvreg.id()].erase(mvreg.attr_name()); //Delete the mvreg if the edge is deleted.
                    if (temp_node_attr[mvreg.id()].empty()) { temp_node_attr.erase(mvreg.id()); }
                }
            }*/
        }

        if (ok) {
            //print_node(mvreg.id());
            std::cout << "[UPDATE NODE] : " << mvreg.id() << " type: " <<  nodes[mvreg.id()].read().begin()->type() << std::endl;
            emit update_node_signal(mvreg.id(), nodes[mvreg.id()].read().begin()->type());
        }

    } catch (const std::exception &e) {
        std::cout << "EXCEPTION: " << __FILE__ << " " << __FUNCTION__ << ":" << __LINE__ << " " << e.what()
                  << std::endl;
    };


}


void DSRGraph::join_delta_edge_attr(IDL::MvregEdgeAttr &mvreg) {
    try {
        bool  ok = false;
        auto d = translateEdgeAttrMvIDLtoCRDT(mvreg);
        {
            std::unique_lock<std::shared_mutex> lock(_mutex);
            //Check if the node where we are joining the edge exist.
            if (nodes.getMapRef().find(mvreg.id()) != nodes.getMapRef().end() and
                !(nodes[mvreg.id()].read_reg().fano().find({mvreg.to(), mvreg.type()}) == nodes[mvreg.id()].read_reg().fano().end())) {
                ok = true;
                auto &n = nodes[mvreg.id()].read_reg().fano()[{mvreg.to(), mvreg.type()}].read_reg();

                std::cout << "JOINING EDGE ATTRIBUTE: " << mvreg.attr_name() << std::endl;

                n.attrs()[mvreg.attr_name()].join(d);

                //Check if we are inserting or deleting.
                if (mvreg.dk().ds().empty() or n.attrs().find(mvreg.attr_name()) == n.attrs().end()) { //Remove
                    n.attrs().erase(mvreg.attr_name());
                    //Update maps
                    update_maps_node_insert(mvreg.id(), nodes[mvreg.id()].read_reg());
                } else { //Insert
                    //Update maps
                    update_maps_node_insert(mvreg.id(), nodes[mvreg.id()].read_reg());
                }
            } /*else if (deleted.find(mvreg.id()) == deleted.end()) {
                //If the node is not found but it is not deleted, we save de delta for later.
                //We use a CRDT because we can receive multiple deltas for the same edge unordered before we get the node.
                temp_edge_attr[{mvreg.from(), mvreg.to(), mvreg.type()}][mvreg.attr_name()].join(d);

                //If we are deleting the edge
                if (temp_edge_attr[{mvreg.from(), mvreg.to(), mvreg.type()}].find(mvreg.attr_name()) ==
                    temp_edge_attr[{mvreg.from(), mvreg.to(), mvreg.type()}].end()) {
                    temp_edge_attr[{mvreg.from(), mvreg.to(), mvreg.type()}].erase(
                            mvreg.attr_name()); //Delete the mvreg if the edge is deleted.
                    if (temp_edge_attr[{mvreg.from(), mvreg.to(), mvreg.type()}].empty()) {
                        temp_edge_attr.erase({mvreg.from(), mvreg.to(), mvreg.type()});
                    }
                }
            }*/
        }

        if (ok) {
            std::cout << "[UPDATE EDGE] : " << mvreg.id()  << " " << mvreg.from() << " type: "<< mvreg.type() <<std::endl;
            emit update_edge_signal(mvreg.id(),  mvreg.from(), mvreg.type());
        }

    } catch (const std::exception &e) {
        std::cout << "EXCEPTION: " << __FILE__ << " " << __FUNCTION__ << ":" << __LINE__ << " " << e.what()
                  << std::endl;
    };


}

void DSRGraph::join_full_graph(IDL::OrMap &full_graph) {

    vector<tuple<bool, int, std::string, CRDTNode>> updates;
    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        /*
        auto m = static_cast<map<int, int>>(full_graph.cbase().cc());
        std::set<pair<int, int>> s;
        for (auto &v : full_graph.cbase().dc())
            s.emplace(std::make_pair(v.first(), v.second()));
        */
        for (auto &[k, val] : full_graph.m()) {
            auto mv = translateNodeMvIDLtoCRDT(val);
            CRDTNode nd = (nodes[k].read().empty()) ? CRDTNode() : *nodes[k].read().begin();

            if (deleted.find(k) == deleted.end()) {
                nodes[k].join(mv);
                if (mv.read().empty() or nodes[k].read().empty()) {
                    update_maps_node_delete(k, nd);
                    updates.emplace_back(make_tuple(false, k, "", nd));
                } else {
                    /*
                    for (auto &[kk, v] : temp_edge[k]) {
                        nodes[k].read_reg().fano()[{std::get<1>(kk), std::get<2>(kk)}].join(v);
                        if (nodes[k].read_reg().fano()[{std::get<1>(kk), std::get<2>(kk)}].read().empty()) {
                            nodes[k].read_reg().fano().erase({std::get<1>(kk), std::get<2>(kk)});
                        }
                        temp_edge[k].erase(kk);
                    }

                    //We have to consume all unordered delta attributes for this node. Normally there won't be any.
                    for (auto &[kk, v] : temp_node_attr[k]) {
                        nodes[k].read_reg().attrs()[kk].join(v);
                        if (nodes[k].read_reg().attrs()[kk].read().empty()) {
                            nodes[k].read_reg().attrs().erase(kk);
                        }
                        temp_node_attr[k].erase(kk);
                    }
                     */
                    print_node(*nodes[k].read().begin());

                    update_maps_node_insert(k, *mv.read().begin());
                    updates.emplace_back(make_tuple(true, k, nodes[k].read().begin()->type(), nd));
                }
            } else {
                //Delete all temporary information.
                //temp_edge.erase(k);
                //temp_node_attr.erase(k);
                //for (auto &[kk, v] : temp_edge_attr)
                //    if (std::get<0>(kk) == k) temp_edge_attr.erase(kk);
            }
        }


    }
    for (auto &[signal, id, type, nd] : updates)
        if (signal) {
            //check what change is joined
            if (nd.attrs() != nodes[id].read().begin()->attrs()) {
                emit update_node_signal(id, nodes[id].read().begin()->type());
            } else if (nd != *nodes[id].read().begin()) {
                auto iter =  nodes[id].read().begin()->fano();
                for (const auto &[k,v] : nd.fano()) {
                    if (iter.find(k) == iter.end())
                            emit del_edge_signal(id, k.first, k.second);
                }
                for (const auto &[k,v] : iter) {
                    if (nd.fano().find(k) == nd.fano().end() or nd.fano()[k] != v)
                            emit update_edge_signal(id, k.first, k.second);
                }
            }
        } else {
            emit del_node_signal(id);
        }

}

bool DSRGraph::start_fullgraph_request_thread() {
    return fullgraph_request_thread();
}

void DSRGraph::start_fullgraph_server_thread() {
    fullgraph_thread = std::thread(&DSRGraph::fullgraph_server_thread, this);
}

void DSRGraph::start_subscription_threads(bool showReceived) {
    delta_node_thread = std::thread(&DSRGraph::node_subscription_thread, this, showReceived);
    delta_edge_thread = std::thread(&DSRGraph::edge_subscription_thread, this, showReceived);
    delta_node_attrs_thread = std::thread(&DSRGraph::node_attrs_subscription_thread, this, showReceived);
    delta_edge_attrs_thread = std::thread(&DSRGraph::edge_attrs_subscription_thread, this, showReceived);

}

uint32_t DSRGraph::id() {
    return nodes.getId();
}

IDL::DotContext DSRGraph::context() {
    IDL::DotContext om_dotcontext;
    for (auto &kv_cc : nodes.context().getCcDc().first) {
        om_dotcontext.cc().emplace(make_pair(kv_cc.first, kv_cc.second));
    }
    for (auto &kv_dc : nodes.context().getCcDc().second) {
        IDL::PairInt p_i;
        p_i.first(kv_dc.first);
        p_i.second(kv_dc.second);
        om_dotcontext.dc().push_back(p_i);
    }
    return om_dotcontext;
}

std::map<uint32_t, IDL::Mvreg> DSRGraph::Map() {
    std::shared_lock<std::shared_mutex> lock(_mutex);
    std::map<uint32_t, IDL::Mvreg> m;
    for (auto kv : nodes.getMapRef()) {
        //std::cout << "P " << kv.second.dk.ds.begin()->second.attrs().find("parent")->second.dk.ds.begin()->second << std::endl;
        m[kv.first] = translateNodeMvCRDTtoIDL(agent_id, kv.first, kv.second);
        //std::cout << "P " << m[kv.first].dk().ds().begin()->second.attrs().find("parent")->second.dk().ds().begin()->second.value().uint() << std::endl;
    }
    return m;
}

void DSRGraph::node_subscription_thread(bool showReceived) {
    // RTPS Initialize subscriptor
    auto name = __FUNCTION__;
    auto lambda_general_topic = [&, name = name](eprosima::fastrtps::Subscriber *sub, bool *work,
                                                 DSR::DSRGraph *graph) {
        if (*work) {
            try {
                eprosima::fastrtps::SampleInfo_t m_info;
                IDL::Mvreg sample;
                if (sub->takeNextData(&sample, &m_info)) { // Get sample
                    if (m_info.sampleKind == eprosima::fastrtps::rtps::ALIVE) {
                        //if( m_info.sample_identity.writer_guid().is_on_same_process_as(sub->getGuid()) == false) {
                        if (sample.agent_id() != agent_id) {
                            if (showReceived)
                                std::cout << name << " Received:" << sample.id() << " node from: "
                                          << m_info.sample_identity.writer_guid() << std::endl;
                            graph->join_delta_node(sample);
                        }
                    }
                }
            }
            catch (const std::exception &ex) { cerr << ex.what() << endl; }
        }
    };
    dsrpub_call_node = NewMessageFunctor(this, &work, lambda_general_topic);
    dsrsub_node.init(dsrparticipant.getParticipant(), "DSR_NODE", dsrparticipant.getNodeTopicName(), dsrpub_call_node);
}

void DSRGraph::edge_subscription_thread(bool showReceived) {
    // RTPS Initialize subscriptor
    auto name = __FUNCTION__;
    auto lambda_general_topic = [&, name = name](eprosima::fastrtps::Subscriber *sub, bool *work,
                                                 DSR::DSRGraph *graph) {
        if (*work) {
            try {
                eprosima::fastrtps::SampleInfo_t m_info;
                IDL::MvregEdge sample;
                if (sub->takeNextData(&sample, &m_info)) { // Get sample
                    if (m_info.sampleKind == eprosima::fastrtps::rtps::ALIVE) {
                        //if( m_info.sample_identity.writer_guid().is_on_same_process_as(sub->getGuid()) == false) {
                        if (sample.agent_id() != agent_id) {
                            if (showReceived)
                                std::cout << name << " Received:" << sample.id() << " node from: "
                                          << m_info.sample_identity.writer_guid() << std::endl;
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

void DSRGraph::edge_attrs_subscription_thread(bool showReceived) {
    // RTPS Initialize subscriptor
    auto name = __FUNCTION__;
    auto lambda_general_topic = [&, name = name](eprosima::fastrtps::Subscriber *sub, bool *work,
                                                 DSR::DSRGraph *graph) {
        if (*work) {
            try {
                eprosima::fastrtps::SampleInfo_t m_info;
                IDL::MvregEdgeAttr sample;
                if (sub->takeNextData(&sample, &m_info)) { // Get sample
                    if (m_info.sampleKind == eprosima::fastrtps::rtps::ALIVE) {
                        //if( m_info.sample_identity.writer_guid().is_on_same_process_as(sub->getGuid()) == false) {
                        if (sample.agent_id() != agent_id) {
                            if (showReceived)
                                std::cout << name << " Received:" << sample.id() << " node from: "
                                          << m_info.sample_identity.writer_guid() << std::endl;
                            graph->join_delta_edge_attr(sample);
                        }
                    }
                }
            }
            catch (const std::exception &ex) { cerr << ex.what() << endl; }
        }
    };
    dsrpub_call_edge_attrs = NewMessageFunctor(this, &work, lambda_general_topic);
    dsrsub_edge_attrs.init(dsrparticipant.getParticipant(), "DSR_EDGE_ATTRS", dsrparticipant.getEdgeAttrTopicName(),
                           dsrpub_call_edge_attrs);
}

void DSRGraph::node_attrs_subscription_thread(bool showReceived) {
    // RTPS Initialize subscriptor
    auto name = __FUNCTION__;
    auto lambda_general_topic = [&, name = name](eprosima::fastrtps::Subscriber *sub, bool *work,
                                                 DSR::DSRGraph *graph) {
        if (*work) {
            try {
                eprosima::fastrtps::SampleInfo_t m_info;
                IDL::MvregNodeAttr sample;
                if (sub->takeNextData(&sample, &m_info)) { // Get sample
                    if (m_info.sampleKind == eprosima::fastrtps::rtps::ALIVE) {
                        //if( m_info.sample_identity.writer_guid().is_on_same_process_as(sub->getGuid()) == false) {
                        if (sample.agent_id() != agent_id) {
                            if (showReceived)
                                std::cout << name << " Received:" << sample.id() << " node from: "
                                          << m_info.sample_identity.writer_guid() << std::endl;
                            graph->join_delta_node_attr(sample);
                        }
                    }
                }
            }
            catch (const std::exception &ex) { cerr << ex.what() << endl; }
        }
    };
    dsrpub_call_node_attrs = NewMessageFunctor(this, &work, lambda_general_topic);
    dsrsub_node_attrs.init(dsrparticipant.getParticipant(), "DSR_NODE_ATTRS", dsrparticipant.getNodeAttrTopicName(),
                           dsrpub_call_node_attrs);
}

void DSRGraph::fullgraph_server_thread() {
    std::cout << __FUNCTION__ << "->Entering thread to attend full graph requests" << std::endl;
    // Request Topic
    auto lambda_graph_request = [&](eprosima::fastrtps::Subscriber *sub, bool *work, DSR::DSRGraph *graph) {

        eprosima::fastrtps::SampleInfo_t m_info;
        IDL::GraphRequest sample;
        //readNextData o takeNextData
        if (sub->takeNextData(&sample, &m_info)) { // Get sample
            if (m_info.sampleKind == eprosima::fastrtps::rtps::ALIVE) {
                if (/*m_info.sample_identity.writer_guid().is_on_same_process_as(sub->getGuid()) == false*/static_cast<uint32_t>(std::stoi(sample.from())) != agent_id) {
                    std::cout << " Received Full Graph request: from " << m_info.sample_identity.writer_guid()
                              << std::endl;
                    *work = false;
                    IDL::OrMap mp;
                    mp.id(graph->get_agent_id());
                    mp.m(graph->Map());
                    mp.cbase(graph->context());
                    std::cout << "nodos enviados: " << mp.m().size() << std::endl;

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
    dsrsub_graph_request.init(dsrparticipant.getParticipant(), "DSR_GRAPH_REQUEST",
                              dsrparticipant.getRequestTopicName(), dsrpub_graph_request_call);
};

bool DSRGraph::fullgraph_request_thread() {
    bool sync = false;
    // Answer Topic
    auto lambda_request_answer = [&sync](eprosima::fastrtps::Subscriber *sub, bool *work, DSR::DSRGraph *graph) {

        eprosima::fastrtps::SampleInfo_t m_info;
        IDL::OrMap sample;
        std::cout << "Grafo completo - Mensajes sin leer " << sub->get_unread_count() << std::endl;
        if (sub->takeNextData(&sample, &m_info)) { // Get sample
            if (m_info.sampleKind == eprosima::fastrtps::rtps::ALIVE) {
                if (/*m_info.sample_identity.writer_guid().is_on_same_process_as(sub->getGuid()) == false*/ sample.id() != graph->get_agent_id()) {
                    std::cout << " Received Full Graph from " << m_info.sample_identity.writer_guid() << " whith "
                              << sample.m().size() << " elements" << std::endl;
                    graph->join_full_graph(sample);
                    std::cout << "Synchronized." << std::endl;
                    sync = true;
                }
            }
        }
    };

    dsrpub_request_answer_call = NewMessageFunctor(this, &work, lambda_request_answer);
    dsrsub_request_answer.init(dsrparticipant.getParticipant(), "DSR_GRAPH_ANSWER", dsrparticipant.getAnswerTopicName(),
                               dsrpub_request_answer_call);

    std::this_thread::sleep_for(300ms);   // NEEDED ?

    std::cout << " Requesting the complete graph " << std::endl;
    IDL::GraphRequest gr;
    gr.from(std::to_string(agent_id));
    dsrpub_graph_request.write(&gr);

    bool timeout = false;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    while (!sync and !timeout) {
        std::this_thread::sleep_for(1000ms);
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        timeout = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() > TIMEOUT*3;
        std::cout  << " Waiting for the graph ... seconds to timeout [" << std::ceil(std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()/10)/100.0  << "/"<< TIMEOUT/1000*3<<"] " << std::endl;
        dsrpub_graph_request.write(&gr);
    }
    eprosima::fastrtps::Domain::removeSubscriber(dsrsub_request_answer.getSubscriber());
    return sync;
}


//////////////////////////////////////////////////
///// PRIVATE COPY
/////////////////////////////////////////////////

DSRGraph::DSRGraph(const DSRGraph& G) : agent_id(G.agent_id), copy(true)
{
    nodes = G.nodes;
    graph_root = G.graph_root;
    utils = std::make_unique<Utilities>(this);
    id_map = G.id_map;
    deleted = G.deleted;
    name_map = G.name_map;
    edges = G.edges;
    edgeType = G.edgeType;
    nodeType = G.nodeType;
}

DSRGraph DSRGraph::G_copy() {
    return DSRGraph(*this);
};

bool DSRGraph::is_copy() {
    return copy;
};