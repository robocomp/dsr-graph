//
// Created by juancarlos on 1/7/20.
//

#include "user_types.h"

namespace CRDT {
    Attribute::Attribute(const ValType &value_, uint64_t timestamp_, uint32_t agent_id_)
            : m_value(ValType(value_)), m_timestamp(timestamp_), m_agent_id(agent_id_) {}

    const ValType &Attribute::value() const {
        return m_value;
    }

    ValType& Attribute::value()  {
        return m_value;
    }

    uint64_t Attribute::timestamp() const {
        return m_timestamp;
    }

    uint32_t Attribute::agent_id() const {
        return m_agent_id;
    }

    void Attribute::timestamp(uint64_t t) {
        m_timestamp = t;
    }

    void Attribute::value(const ValType &mValue) {
        m_value = mValue;
    }


    void Attribute::agent_id(uint32_t mAgentId) {
        m_agent_id = mAgentId;
    }

    Edge::Edge(uint32_t mTo, uint32_t mFrom, const string &mType,
                   const unordered_map<std::string, Attribute> &mAttrs,
                   uint32_t mAgentId) : m_to(mTo), m_from(mFrom), m_type(mType), m_attrs{mAttrs},
                                          m_agent_id(mAgentId) {}

    uint32_t Edge::to() const {
        return m_to;
    }

    uint32_t Edge::from() const {
        return m_from;
    }

    const string &Edge::type() const {
        return m_type;
    }

    string &Edge::type() {
        return m_type;
    }

    const unordered_map<std::string, Attribute> &Edge::attrs() const {
        return m_attrs;
    }

    unordered_map<std::string, Attribute> &Edge::attrs()  {
        return m_attrs;
    }

    uint32_t Edge::agent_id() const {
        return m_agent_id;
    }

    void Edge::to(uint32_t mTo) {
        m_to = mTo;
    }

    void Edge::from(uint32_t mFrom) {
        m_from = mFrom;
    }

    void Edge::type(const string &mType) {
        m_type = mType;
    }

    void Edge::attrs(const unordered_map<std::string, Attribute> &mAttrs) {
        m_attrs = mAttrs;
    }

    void Edge::agent_id(uint32_t mAgentId) {
        m_agent_id = mAgentId;
    }

    Node::Node(uint32_t mId, const string &mType, const string &mName,
               const unordered_map<std::string, Attribute> &mAttrs,
               const unordered_map<std::pair<uint32_t, std::string>, Edge, pair_hash> &mFano, uint32_t mAgentId)
            : m_id(
            mId), m_type(mType), m_name(mName), m_attrs{mAttrs}, m_fano{mFano}, m_agent_id(mAgentId) {}

    uint32_t Node::id() const {
        return m_id;
    }

    const string &Node::type() const {
        return m_type;
    }

    string &Node::type()  {
        return m_type;
    }

    const string &Node::name() const {
        return m_name;
    }

    string &Node::name()  {
        return m_name;
    }

    const unordered_map<std::string, Attribute> &Node::attrs() const {
        return m_attrs;
    }

    unordered_map<std::string, Attribute>& Node::attrs()  {
        return m_attrs;
    }

    const unordered_map<std::pair<uint32_t, std::string>, Edge, pair_hash> &Node::fano() const {
        return m_fano;
    }

     unordered_map<std::pair<uint32_t, std::string>, Edge, pair_hash> &Node::fano()  {
        return m_fano;
    }

    uint32_t Node::agent_id() const {
        return m_agent_id;
    }

    void Node::id(uint32_t mId) {
        m_id = mId;
    }

    void Node::type (const string &mType) {
        m_type = mType;
    }

    void Node::name(const string &mName) {
        m_name = mName;
    }

    void Node::attrs(const unordered_map<std::string, Attribute> &mAttrs) {
        m_attrs = mAttrs;
    }

    void Node::fano(const unordered_map<std::pair<uint32_t, std::string>, Edge, pair_hash> &mFano) {
        m_fano = mFano;
    }

    void Node::agent_id(uint32_t mAgentId) {
        m_agent_id = mAgentId;
    }
}