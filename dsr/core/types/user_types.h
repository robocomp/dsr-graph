//
// Created by juancarlos on 1/7/20.
//

#ifndef USER_TYPES_H
#define USER_TYPES_H

#include <cstdint>
#include "crdt_types.h"

namespace CRDT {




    class Attribute {
    public:
        Attribute(const CRDT::ValType& mvalue, uint64_t mtimestamp, uint32_t magent_id );

        Attribute (const CRDTAttribute& attr ) {
            m_timestamp = attr.timestamp();
            m_agent_id = attr.agent_id();
            m_value = ValType(attr.val().variant());
        }
        ~Attribute() = default;

        Attribute& operator= (const CRDTAttribute& attr ) {
            m_timestamp = attr.timestamp();
            m_agent_id = attr.agent_id();
            m_value = ValType(attr.val().variant());

            return *this;

        }

        const ValType &value() const;

        ValType& value();

        uint64_t timestamp() const;

        void timestamp(uint64_t t);

        uint32_t agent_id() const;

    private:
        ValType m_value;
        uint64_t m_timestamp;
        uint32_t m_agent_id;
    };

    class Edge {
    public:
        Edge(uint32_t mTo, uint32_t mFrom, const string &mType, const unordered_map<std::string, Attribute>& mAttrs,
                 uint32_t mAgentId);


        Edge (const CRDTEdge& edge ) {
            m_agent_id = edge.agent_id();
            m_from = edge.from();
            m_to = edge.to();
            m_type = edge.type();
            for (const auto &[k,v] : edge.attrs()) {
                m_attrs[k] = *v.read().begin();
            }

        }
        ~Edge() = default;

        Edge& operator= (const CRDTEdge& attr ) {
            m_agent_id = attr.agent_id();
            m_from = attr.from();
            m_to = attr.to();
            m_type = attr.type();
            for (const auto &[k,v] : attr.attrs()) {
                m_attrs[k] = *v.read().begin();
            }
            return *this;

        }

        uint32_t to() const;

        uint32_t from() const;

        const string &type() const;

        string &type();

        const unordered_map<std::string, Attribute> &attrs() const;

        unordered_map<std::string, Attribute> &attrs();

        uint32_t agent_id() const;

    private:
        uint32_t m_to;
        uint32_t m_from;
        std::string m_type;
        std::unordered_map<std::string, Attribute> m_attrs;
        uint32_t m_agent_id;
    };

    class Node {
    public:
        Node(uint32_t mId, const string &mType, const string &mName,
                 const unordered_map<std::string, Attribute> &mAttrs,
                 const unordered_map<std::pair<int32_t, std::string>, Edge, pair_hash> &mFano, uint32_t mAgentId);

        Node (const CRDTNode& node) {
            m_agent_id = node.agent_id();
            m_id = node.id();
            m_name = node.name();
            m_type = node.type();
            for (const auto &[k,v] : node.attrs()) {
                m_attrs[k] = *v.read().begin();
            }
            for (const auto &[k,v] : node.fano()) {
                m_fano[k] = *v.read().begin();
            }
        }

        ~Node() = default;

        Node& operator= (const CRDTNode& node ) {
            m_agent_id = node.agent_id();
            m_id = node.id();
            m_name = node.name();
            m_type = node.type();
            for (const auto &[k,v] : node.attrs()) {
                m_attrs[k] = *v.read().begin();
            }
            for (const auto &[k,v] : node.fano()) {
                m_fano[k] = *v.read().begin();
            }

            return *this;
        }

        uint32_t id() const;

        const string &type() const;

        string &type();

        const string &name() const;

        string &name();

        const unordered_map<std::string, Attribute> &attrs() const;

        unordered_map<std::string, Attribute> &attrs();

        const unordered_map<std::pair<int32_t, std::string>, Edge, pair_hash> &fano() const;

        unordered_map<std::pair<int32_t, std::string>, Edge, pair_hash> &fano();

        uint32_t agent_id() const;


    private:
        uint32_t m_id;
        std::string m_type;
        std::string m_name;
        std::unordered_map<std::string, Attribute> m_attrs;
        std::unordered_map<std::pair<int32_t, std::string>, Edge, pair_hash> m_fano;
        uint32_t m_agent_id;
    };

}

#endif //USER_TYPES_H
