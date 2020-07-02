//
// Created by juancarlos on 1/7/20.
//

#ifndef CONVERTER_H
#define CONVERTER_H

#include "user_types.h"

namespace CRDT {

    // Translators
    inline static IDL::Mvreg translateNodeMvCRDTtoIDL(int agent_id, int id, mvreg<CRDTNode, int> &data) {
        IDL::Mvreg delta_crdt;
        for (auto &kv_dots : data.dk.ds) {
            IDL::PairInt pi;
            pi.first(kv_dots.first.first);
            pi.second(kv_dots.first.second);

            delta_crdt.dk().ds().emplace(make_pair(pi, kv_dots.second.toIDLNode(id)));
            delta_crdt.dk().cbase().cc().emplace(kv_dots.first);
        }
        //for (auto &kv_cc : data.context().getCcDc().first){
        //    delta_crdt.dk().cbase().cc().emplace(make_pair(kv_cc.first, kv_cc.second));
        //}
        for (auto &kv_dc : data.context().getCcDc().second) {
            IDL::PairInt pi;
            pi.first(kv_dc.first);
            pi.second(kv_dc.second);

            delta_crdt.dk().cbase().dc().push_back(pi);
        }
        delta_crdt.id(id);
        delta_crdt.agent_id(agent_id);
        return delta_crdt;
    }

    inline static mvreg<CRDTNode, int> translateNodeMvIDLtoCRDT(IDL::Mvreg &data) {
        // Context
        dotcontext<int> dotcontext_aux;
        std::map<int, int> m;
        for (auto &v : data.dk().cbase().cc())
            m.insert(std::make_pair(v.first, v.second));
        std::set<pair<int, int>> s;
        for (auto &v : data.dk().cbase().dc())
            s.insert(std::make_pair(v.first(), v.second()));
        dotcontext_aux.setContext(m, s);
        // Dots
        std::map<pair<int, int>, CRDTNode> ds_aux;
        for (auto &[k, v] : data.dk().ds())
            ds_aux[pair<int, int>(k.first(), k.second())] = std::move(v);
        // Join
        mvreg<CRDTNode, int> aw;
        aw.dk.c = dotcontext_aux;
        aw.dk.set(ds_aux);
        return aw;
    }

    inline static IDL::MvregEdgeAttr
    translateEdgeAttrMvCRDTtoIDL(int agent_id, int id, int from, int to, const std::string &type,
                                 const std::string &attr, mvreg<CRDTAttribute, int> &data) {
        IDL::MvregEdgeAttr delta_crdt;

        for (auto &kv_dots : data.dk.ds) {
            IDL::PairInt pi;
            pi.first(kv_dots.first.first);
            pi.second(kv_dots.first.second);

            delta_crdt.dk().ds().emplace(make_pair(pi, kv_dots.second.toIDLAttrib()));
            delta_crdt.dk().cbase().cc().emplace(kv_dots.first);
        }

        //pair<map<int, int>, set<pair<int, int>>> d = data.context().getCcDc();
        //for (auto &kv_cc : data.context().getCcDc().first){
        //    delta_crdt.dk().cbase().cc().emplace(make_pair(kv_cc.first, kv_cc.second));
        //}
        for (auto &kv_dc : data.context().dc) {
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

    inline static mvreg<CRDTAttribute, int> translateEdgeAttrMvIDLtoCRDT(IDL::MvregEdgeAttr &data) {
        // Context
        dotcontext<int> dotcontext_aux;
        std::map<int, int> m;
        for (auto &v : data.dk().cbase().cc())
            m.insert(std::make_pair(v.first, v.second));
        std::set<pair<int, int>> s;
        for (auto &v : data.dk().cbase().dc())
            s.insert(std::make_pair(v.first(), v.second()));
        dotcontext_aux.setContext(m, s);
        // Dots
        std::map<pair<int, int>, CRDTAttribute> ds_aux;
        for (auto &[k, v] : data.dk().ds())
            ds_aux[pair<int, int>(k.first(), k.second())] = std::move(v);
        // Join
        mvreg<CRDTAttribute, int> aw;
        aw.dk.c = dotcontext_aux;
        aw.dk.set(ds_aux);
        return aw;
    }

    inline static IDL::MvregNodeAttr
    translateNodeAttrMvCRDTtoIDL(int agent_id, int id, int node, const std::string &attr, mvreg<CRDTAttribute, int> &data) {
        IDL::MvregNodeAttr delta_crdt;


        for (auto &kv_dots : data.dk.ds) {
            IDL::PairInt pi;
            pi.first(kv_dots.first.first);
            pi.second(kv_dots.first.second);

            delta_crdt.dk().ds().emplace(make_pair(pi, kv_dots.second.toIDLAttrib()));
            delta_crdt.dk().cbase().cc().emplace(kv_dots.first);
        }
        //pair<map<int, int>, set<pair<int, int>>> d = data.context().getCcDc();
        //for (auto &kv_cc : d.first) {
        //    delta_crdt.dk().cbase().cc().emplace(make_pair(kv_cc.first, kv_cc.second));
        //}
        for (auto &kv_dc : data.context().dc) {
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

    inline static mvreg<CRDTAttribute, int> translateNodeAttrMvIDLtoCRDT(IDL::MvregNodeAttr &data) {
        // Context
        dotcontext<int> dotcontext_aux;
        std::map<int, int> m;
        for (auto &v : data.dk().cbase().cc())
            m.insert(std::make_pair(v.first, v.second));
        std::set<pair<int, int>> s;
        for (auto &v : data.dk().cbase().dc())
            s.insert(std::make_pair(v.first(), v.second()));
        dotcontext_aux.setContext(m, s);
        // Dots
        std::map<pair<int, int>, CRDTAttribute> ds_aux;
        for (auto &[k, v] : data.dk().ds())
            ds_aux[pair<int, int>(k.first(), k.second())] = std::move(v);
        // Join
        mvreg<CRDTAttribute, int> aw;
        aw.dk.c = dotcontext_aux;
        aw.dk.set(ds_aux);
        return aw;
    }

    inline static mvreg<CRDTEdge, int> translateEdgeMvIDLtoCRDT(IDL::MvregEdge &data) {
        // Context
        dotcontext<int> dotcontext_aux;
        std::map<int, int> m;
        for (auto &v : data.dk().cbase().cc())
            m.insert(std::make_pair(v.first, v.second));
        std::set<pair<int, int>> s;
        for (auto &v : data.dk().cbase().dc())
            s.insert(std::make_pair(v.first(), v.second()));
        dotcontext_aux.setContext(m, s);
        // Dots
        std::map<pair<int, int>, CRDTEdge> ds_aux;
        for (auto &[k, v] : data.dk().ds())
            ds_aux[pair<int, int>(k.first(), k.second())] = std::move(v);
        // Join
        mvreg<CRDTEdge, int> aw;
        aw.dk.c = dotcontext_aux;
        aw.dk.set(ds_aux);
        return aw;
    }

    inline static IDL::MvregEdge translateEdgeMvCRDTtoIDL(int agent_id, int id, mvreg<CRDTEdge, int> &data) {
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
            delta_crdt.dk().cbase().cc().emplace(kv_dots.first);

        }
        //pair<map<int, int>, set<pair<int, int>>> d = data.context().getCcDc();
        //for (auto &kv_cc : data.context().getCcDc().first){
        //    delta_crdt.dk().cbase().cc().emplace(make_pair(kv_cc.first, kv_cc.second));
        //}
        for (auto &kv_dc : data.context().dc) {
            IDL::PairInt pi;
            pi.first(kv_dc.first);
            pi.second(kv_dc.second);

            delta_crdt.dk().cbase().dc().push_back(pi);
        }
        delta_crdt.id(id);
        delta_crdt.agent_id(agent_id);
        return delta_crdt;
    }

    inline static CRDTAttribute user_attribute_to_crdt(Attribute&& attr) {

        CRDTAttribute attribute;
        CRDTValue val;
        attribute.type(attr.value().index());
        val.variant(std::move(attr.value()));
        attribute.val(std::move(val));
        attribute.agent_id(attr.agent_id());
        attribute.timestamp(attr.timestamp());

        return attribute;
    }

    inline static CRDTAttribute user_attribute_to_crdt(const Attribute& attr) {

        CRDTAttribute attribute;
        CRDTValue val;
        attribute.type(attr.value().index());
        val.variant(attr.value());
        attribute.val(std::move(val));
        attribute.agent_id(attr.agent_id());
        attribute.timestamp(attr.timestamp());

        return attribute;
    }

    inline static CRDTEdge user_edge_to_crdt(Edge&& edge) {
        CRDTEdge crdt_edge;

        crdt_edge.agent_id(edge.agent_id());
        crdt_edge.from(edge.from());
        crdt_edge.to(edge.to());
        crdt_edge.type(std::move(edge.type()));
        for (auto &&[k,v] : edge.attrs()) {
            mvreg<CRDTAttribute, int> mv;
            mv.write(user_attribute_to_crdt(std::move(v)));
            crdt_edge.attrs().emplace(k, std::move(mv));
        }

        return crdt_edge;
    }

    inline static CRDTEdge user_edge_to_crdt(const Edge& edge) {
        CRDTEdge crdt_edge;

        crdt_edge.agent_id(edge.agent_id());
        crdt_edge.from(edge.from());
        crdt_edge.to(edge.to());
        crdt_edge.type(edge.type());
        for (auto &&[k,v] : edge.attrs()) {
            mvreg<CRDTAttribute, int> mv;
            mv.write(user_attribute_to_crdt(v));
            crdt_edge.attrs().emplace(k, mv);
        }

        return crdt_edge;
    }


    inline static CRDTNode user_node_to_crdt(Node&& node) {
        CRDTNode crdt_node;

        crdt_node.agent_id(node.agent_id());
        crdt_node.id(node.id());
        crdt_node.type(std::move(node.type()));
        crdt_node.name(std::move(node.name()));
        for (auto &&[k,v] : node.attrs()) {
            mvreg<CRDTAttribute, int> mv;
            mv.write(user_attribute_to_crdt(std::move(v)));
            crdt_node.attrs().emplace(k, std::move(mv));
        }

        for (auto &&[k,v] : node.fano()) {
            mvreg<CRDTEdge, int> mv;
            mv.write(user_edge_to_crdt(std::move(v)));
            crdt_node.fano().emplace(k, std::move(mv));
        }
        return crdt_node;
    }


    inline static CRDTNode user_node_to_crdt(const Node& node) {
        CRDTNode crdt_node;

        crdt_node.agent_id(node.agent_id());
        crdt_node.id(node.id());
        crdt_node.type(node.type());
        crdt_node.name(node.name());
        for (auto &&[k,v] : node.attrs()) {
            mvreg<CRDTAttribute, int> mv;
            mv.write(user_attribute_to_crdt(v));
            crdt_node.attrs().emplace(k, mv);
        }

        for (auto &&[k,v] : node.fano()) {
            mvreg<CRDTEdge, int> mv;
            mv.write(user_edge_to_crdt(v));
            crdt_node.fano().emplace(k, mv);
        }
        return crdt_node;
    }
}


#endif //CONVERTER_H
