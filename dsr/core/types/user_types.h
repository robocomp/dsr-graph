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

        Attribute() = default;

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

        friend std::ostream &operator<<(std::ostream &os, const Attribute &type) {

            switch (type.m_value.index()) {
                //case 0:
                //    os << "UNINITIALIZED ";
                //    break;
                case 0:
                    os << " str: " << get<string>(type.m_value);
                    break;
                case 1:
                    os << " dec: " << get<int32_t>(type.m_value);
                    break;
                case 2:
                    os << " float: " << get<float>(type.m_value);
                    break;
                case 3:
                    os << " float_vec: [ ";
                    for (const auto &k: get<vector<float>>(type.m_value))
                        os << k << ", ";
                    os << "] ";
                    break;
                case 4:
                    os << "bool: " << (get<bool>(type.m_value) ? " TRUE" : " FALSE");
                    break;
                case 5:
                    os << " byte_vec: [ ";
                    for (const auto &k: get<vector<uint8_t>>(type.m_value))
                        os << k << ", ";
                    os << "] ";
                    break;
                default:
                    os << "OTRO TIPO";
                    break;
            }
            return os;
        }

        void value(const ValType &mValue);

        void setMTimestamp(uint64_t mTimestamp);

        void agent_id(uint32_t mAgentId);

        bool operator==(const Attribute &rhs) const {
            return m_value == rhs.m_value;
        }

        bool operator!=(const Attribute &rhs) const {
            return !(rhs == *this);
        }

        bool operator<(const Attribute &rhs) const {
            return m_value < rhs.m_value;
        }

        bool operator>(const Attribute &rhs) const {
            return rhs < *this;
        }

        bool operator<=(const Attribute &rhs) const {
            return !(rhs < *this);
        }

        bool operator>=(const Attribute &rhs) const {
            return !(*this < rhs);
        }

    private:
        ValType m_value;
        uint64_t m_timestamp;
        uint32_t m_agent_id;
    };

    class Edge {
    public:

        Edge() = default;

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

        void to(uint32_t mTo);

        void from(uint32_t mFrom);

        void type(const string &mType);

        void attrs(const unordered_map<std::string, Attribute> &mAttrs);

        void agent_id(uint32_t mAgentId);

        bool operator==(const Edge &rhs) const {
            return m_to == rhs.m_to &&
                   m_from == rhs.m_from &&
                   m_type == rhs.m_type &&
                   m_attrs == rhs.m_attrs;
        }

        bool operator!=(const Edge &rhs) const {
            return !(rhs == *this);
        }

        bool operator<(const Edge &rhs) const {
            if (m_to < rhs.m_to)
                return true;
            if (rhs.m_to < m_to)
                return false;
            if (m_from < rhs.m_from)
                return true;
            if (rhs.m_from < m_from)
                return false;
            if (m_type < rhs.m_type)
                return true;
            if (rhs.m_type < m_type)
                return false;
            return true;
        }

        bool operator>(const Edge &rhs) const {
            return rhs < *this;
        }

        bool operator<=(const Edge &rhs) const {
            return !(rhs < *this);
        }

        bool operator>=(const Edge &rhs) const {
            return !(*this < rhs);
        }

    private:
        uint32_t m_to;
        uint32_t m_from;
        std::string m_type;
        std::unordered_map<std::string, Attribute> m_attrs;
        uint32_t m_agent_id;
    };

    class Node {
    public:

        Node() = default;

        Node(uint32_t mId, const string &mType, const string &mName,
                 const unordered_map<std::string, Attribute> &mAttrs,
                 const unordered_map<std::pair<uint32_t, std::string>, Edge, pair_hash> &mFano, uint32_t mAgentId);

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

        const unordered_map<std::pair<uint32_t, std::string>, Edge, pair_hash> &fano() const;

        unordered_map<std::pair<uint32_t, std::string>, Edge, pair_hash> &fano();

        uint32_t agent_id() const;

        void id(uint32_t mId);

        void type(const string &mType);

        void name(const string &mName);

        void attrs(const unordered_map<std::string, Attribute> &mAttrs);

        void fano(const unordered_map<std::pair<uint32_t, std::string>, Edge, pair_hash> &mFano);

        void agent_id(uint32_t mAgentId);

        bool operator==(const Node &rhs) const {
            return m_id == rhs.m_id &&
                   m_type == rhs.m_type &&
                   m_name == rhs.m_name &&
                   m_attrs == rhs.m_attrs &&
                   m_fano == rhs.m_fano;
        }

        bool operator!=(const Node &rhs) const {
            return !(rhs == *this);
        }

        bool operator<(const Node &rhs) const {
            if (m_id < rhs.m_id)
                return true;
            if (rhs.m_id < m_id)
                return false;
            if (m_type < rhs.m_type)
                return true;
            if (rhs.m_type < m_type)
                return false;
            if (m_name < rhs.m_name)
                return true;
            if (rhs.m_name < m_name)
                return false;
            return true;
        }

        bool operator>(const Node &rhs) const {
            return rhs < *this;
        }

        bool operator<=(const Node &rhs) const {
            return !(rhs < *this);
        }

        bool operator>=(const Node &rhs) const {
            return !(*this < rhs);
        }

    private:
        uint32_t m_id;
        std::string m_type;
        std::string m_name;
        std::unordered_map<std::string, Attribute> m_attrs;
        std::unordered_map<std::pair<uint32_t, std::string>, Edge, pair_hash> m_fano;
        uint32_t m_agent_id;
    };

}

#endif //USER_TYPES_H
