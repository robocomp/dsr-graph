//
// Created by juancarlos on 8/6/20.
//

#ifndef CRDT_RTPS_DSR_CRDT_TYPES_H
#define CRDT_RTPS_DSR_CRDT_TYPES_H

#include "libs/delta-crdts.cc"
#include <iostream>
#include "unordered_map"
#include "variant"
#include "map"

namespace CRDT {


    struct pair_hash {
        template<class T1, class T2>
        std::size_t operator()(const std::pair<T1, T2> &pair) const {
            return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
        }
    };

    static constexpr std::array<std::string_view, 7> TYPENAMES_UNION = {"UNINITIALIZED", "STRING", "INT", "FLOAT",
                                                                        "FLOAT_VEC", "BOOL", "BYTE_VEC"};
    using ValType = std::variant<std::string, int32_t, float, std::vector<float>, bool, std::vector<byte>>;
    enum Types : uint32_t {
        STRING,
        INT,
        FLOAT,
        FLOAT_VEC,
        BOOL,
        BYTE_VEC
    };


    class Value {
    public:

        Value() = default;

        ~Value() = default;



        Value(IDL::Val &&x) {

            switch (x._d()) {
                case 0:
                    val = std::move(x.str());
                    break;
                case 1:
                    val = x.dec();
                    break;
                case 2:
                    val = x.fl();
                    break;
                case 3:
                    val = std::move(x.float_vec());
                    break;
                case 4:
                    val = x.bl();
                    break;
                case 5: {
                    std::vector<std::byte> b = std::vector<std::byte>(reinterpret_cast<std::byte *>(x.byte_vec().data()),
                                           reinterpret_cast<std::byte *>(x.byte_vec().data()) + x.byte_vec().size());
                    val = b;
                    break;
                }
                default:
                    break;
            }
        }

        Value(std::vector<float> &&float_vec) {
            val = std::move(float_vec);
        }

        Value(const std::vector<float> &float_vec) {
            val = float_vec;
        }

        Value &operator=(const Value &x) {

            if (this == &x) return *this;
            val = x.val;
            return *this;
        }

        Value &operator=(Value &&x) {
            if (this == &x) return *this;
            val = std::move(x.val);
            return *this;
        }

        bool operator<(const Value &rhs) const {

            if (static_cast<int32_t>(val.index()) != rhs.selected()) return false;

            switch (val.index()) {
                case 0:
                    return str() < rhs.str();
                case 1:
                    return dec() < rhs.dec();
                case 2:
                    return fl() < rhs.fl();
                case 3:
                    return float_vec() < rhs.float_vec();
                case 4:
                    return bl() < rhs.bl();
                case 5:
                    return byte_vec() < rhs.byte_vec();
                default:
                    return false;
            }
        }

        bool operator>(const Value &rhs) const {
            return rhs < *this;
        }

        bool operator<=(const Value &rhs) const {
            return !(rhs < *this);
        }

        bool operator>=(const Value &rhs) const {
            return !(*this < rhs);
        }

        bool operator==(const Value &rhs) const {

            if (static_cast<int32_t>(val.index()) != rhs.selected()) return false;
            return val == rhs.val;
        }

        bool operator!=(const Value &rhs) const {
            return !(rhs == *this);
        }



        friend std::ostream &operator<<(std::ostream &os, const Value &type) {

            switch (type.selected()) {
                //case 0:
                //    os << "UNINITIALIZED ";
                //    break;
                case 0:
                    os << " str: " << type.str();
                    break;
                case 1:
                    os << " dec: " << type.dec();
                    break;
                case 2:
                    os << " float: " << type.fl();
                    break;
                case 3:
                    os << " float_vec: [ ";
                    for (const auto &k: type.float_vec())
                        os << k << ", ";
                    os << "] ";
                    break;
                case 4:
                    os << "bool: " << (type.bl() ? " TRUE" : " FALSE");
                    break;
                case 5:
                    os << " byte_vec: [ ";
                    for (const auto &k: type.byte_vec())
                        os << static_cast<uint8_t >(k) << ", ";
                    os << "] ";
                    break;
                default:
                    os << "OTRO TIPO";
                    break;
            }
            return os;
        }

        [[nodiscard]] int32_t selected() const ;

        std::string &str() ;

        [[nodiscard]] const std::string &str() const ;

        void str(const std::string &_str) ;

        void str(std::string &&_str) ;

        void dec(int32_t _dec) ;

        [[nodiscard]] int32_t dec() const ;

        void fl(float _fl) ;

        [[nodiscard]] float fl() const ;

        void float_vec(const std::vector<float> &_float_vec) ;

        void float_vec(std::vector<float> &&_float_vec) ;

        [[nodiscard]] const std::vector<float> &float_vec() const ;

        std::vector<float> &float_vec() ;

        void bl(bool _bl) ;

        [[nodiscard]] bool bl() const ;

        void byte_vec(const std::vector<byte> &_float_vec) ;

        void byte_vec(std::vector<byte> &&_float_vec) ;

        [[nodiscard]] const std::vector<byte> &byte_vec() const ;

        std::vector<byte> &byte_vec() ;

        [[nodiscard]] IDL::Val toIDLVal() ;

    private:
        ValType val;
    };


    class Attribute {
    public:

        Attribute() {
            m_type = 0;
            m_timestamp = 0;
            m_agent_id = 0;
        }

        ~Attribute() = default;

        Attribute(const Attribute &x)
        {

            m_type = x.type();
            m_Value = x.val();
            m_timestamp = x.timestamp();
            m_agent_id = x.agent_id();

        }

        Attribute &operator=(IDL::Attrib &&x) {

            m_type = x.type();
            m_timestamp = x.timestamp();
            m_Value = Value(std::move(x.value()));
            m_agent_id = x.agent_id();
            return *this;
        }

        Attribute &operator=(Attribute &&x) {

            m_type = x.type();
            m_timestamp = x.timestamp();
            m_Value = std::move(x.val());
            m_agent_id = x.agent_id();
            return *this;
        }

        Attribute &operator=(const Attribute &x) {

            m_type = x.type();
            m_timestamp = x.timestamp();
            m_Value = x.val();
            m_agent_id = x.agent_id();
            return *this;
        }

        bool operator==(const Attribute &av_) const {
            if (this == &av_) {
                return true;
            }
            if (type() != av_.type() || val() != av_.val() || timestamp() != av_.timestamp()) {
                return false;
            }
            return true;
        }

        bool operator<(const Attribute &av_) const {
            if (this == &av_) {
                return false;
            }
            if (Value() < av_.val()) {
                return true;
            } else if (av_.val() < Value()) {
                return false;
            }
            return false;
        }

        bool operator!=(const Attribute &av_) const {
            return !operator==(av_);
        }

        bool operator<=(const Attribute &av_) const {
            return operator<(av_) || operator==(av_);
        }

        bool operator>(const Attribute &av_) const {
            return !operator<(av_) && !operator==(av_);
        }

        bool operator>=(const Attribute &av_) const {
            return !operator<(av_);
        }

        friend std::ostream &operator<<(std::ostream &output, const Attribute &av_) {
            output << "Type: " << av_.type() << ", Value[" << av_.val() << "]: " << av_.val() << ", ";
            return output;
        };
        void type(int32_t _type) ;


        [[nodiscard]] int32_t type() const ;

        void timestamp(uint64_t _time) ;

        [[nodiscard]] uint64_t timestamp() const ;


        void val(IDL::Val &&_Value) ;

        void val(Value &&_Value) ;

        [[nodiscard]] const Value &val() const ;

        Value &val() ;

        void agent_id(int32_t _agent_id) ;

        [[nodiscard]] int32_t agent_id() const ;

        [[nodiscard]] IDL::Attrib toIDLAttrib() ;

    private:
        int32_t m_type;
        Value m_Value;
        uint64_t m_timestamp;
        uint32_t m_agent_id;
    };


    class Edge {
    public:


        Edge() {
            m_to = 0;
            m_type = "";
            m_from = 0;
            m_agent_id = 0;
        }

        ~Edge() = default;


        Edge &operator=(const Edge &x) = default;

        Edge &operator=(IDL::Edge &&x) ;

        bool operator==(const Edge &eA_) const {
            if (this == &eA_) {
                return true;
            }
            if (m_type != eA_.m_type || from() != eA_.from() || to() != eA_.to() || attrs() != eA_.attrs()) {
                return false;
            }
            return true;
        }

        bool operator<(const Edge &eA_) const {
            if (this == &eA_) {
                return false;
            }
            if (m_type < eA_.m_type) {
                return true;
            } else if (eA_.m_type < m_type) {
                return false;
            }
            return false;
        }

        bool operator!=(const Edge &eA_) const {
            return !operator==(eA_);
        }

        bool operator<=(const Edge &eA_) const {
            return operator<(eA_) || operator==(eA_);
        }

        bool operator>(const Edge &eA_) const {
            return !operator<(eA_) && !operator==(eA_);
        }

        bool operator>=(const Edge &eA_) const {
            return !operator<(eA_);
        }

        friend std::ostream &operator<<(std::ostream &output,const Edge &ea_) {
            output << "IDL::EdgeAttribs[" << ea_.m_type << ", from:" << ea_.from() << "-> to:" << ea_.to()
                   << " Attribs:[";
            for (const auto &v : ea_.attrs())
                output << v.first << ":" << v.second << " - ";
            output << "]]";
            return output;
        };

        void to(int32_t _to) ;

        [[nodiscard]] int32_t to() const ;


        void type(const std::string &_type) ;

        void type(std::string &&_type) ;

        [[nodiscard]] const std::string &type() const ;

        std::string &type() ;

        void from(int32_t _from) ;

        [[nodiscard]] int32_t from() const ;

        void attrs(const std::unordered_map<std::string, mvreg<Attribute, int>> &_attrs) ;

        void attrs(std::unordered_map<std::string, mvreg<Attribute, int>> &&_attrs) ;

        const std::unordered_map<std::string, mvreg<Attribute, int>> &attrs() const ;

        std::unordered_map<std::string, mvreg<Attribute, int>> &attrs() ;

        void agent_id(int32_t _agent_id) ;

        [[nodiscard]] int32_t agent_id() const ;

        [[nodiscard]] IDL::Edge toIDLEdge(int id) ;

    private:
        int32_t m_to;
        std::string m_type;
        int32_t m_from;
        std::unordered_map<std::string, mvreg<Attribute, int>> m_attrs;
        int32_t m_agent_id{};
    };

    class Node {

    public:

        Node() {
            m_type = "";
            m_name = "";
            m_id = 0;
            m_agent_id = 0;
        }

        ~Node() = default;

        Node(const Node &x) {
            *this = x;
        }

        Node(Node &&x) {
            m_type = std::move(x.type());
            m_name = std::move(x.name());
            m_id = x.id();
            m_agent_id = x.agent_id();
            m_attrs = std::move(x.m_attrs);
            m_fano = std::move(x.m_fano);

        }
        Node(IDL::Node &&x) ;

        Node &operator=(const Node &x) = default;

        Node &operator=(Node &x) = default;

        Node &operator=(Node &&x) noexcept {

            m_type = std::move(x.m_type);
            m_name = std::move(x.m_name);
            m_id = x.m_id;
            m_agent_id = x.m_agent_id;
            m_attrs = std::move(x.m_attrs);
            m_fano = std::move(x.m_fano);

            return *this;
        }


        bool operator==(const Node &n_) const {
            if (this == &n_) {
                return true;
            }
            if (id() != n_.id() || type() != n_.type() || attrs() != n_.attrs() || fano() != n_.fano()) {
                return false;
            }
            return true;
        }


        bool operator<(const Node &n_) const {
            if (this == &n_) {
                return false;
            }
            if (id() < n_.id()) {
                return true;
            } else if (n_.id() < id()) {
                return false;
            }
            return false;
        }

        bool operator!=(const Node &n_) const {
            return !operator==(n_);
        }

        bool operator<=(const Node &n_) const {
            return operator<(n_) || operator==(n_);
        }

        bool operator>(const Node &n_) const {
            return !operator<(n_) && !operator==(n_);
        }

        bool operator>=(const Node &n_) const {
            return !operator<(n_);
        }

        friend std::ostream &operator<<(std::ostream &output, Node &n_) {
            output << "IDL::Node:[" << n_.id() << "," << n_.name() << "," << n_.type() << "], Attribs:[";
            for (const auto &v : n_.attrs())
                output << v.first << ":(" << v.second << ");";
            output << "], FanOut:[";
            for (auto &v : n_.fano())
                output << "[ " << v.first.first << " " << v.first.second << "] " << ":(" << v.second << ");";
            output << "]";
            return output;
        }
        void type(const std::string &_type) ;

        void type(std::string &&_type) ;

        [[nodiscard]] const std::string &type() const ;

        std::string &type() ;

        void name(const std::string &_name) ;

        void name(std::string &&_name) ;

        [[nodiscard]] const std::string &name() const ;

        std::string &name() ;

        void id(int32_t _id) ;

        [[nodiscard]] int32_t id() const ;

        void agent_id(int32_t _agent_id) ;

        [[nodiscard]] int32_t agent_id() const ;

        void attrs(const std::unordered_map<std::string, mvreg<Attribute, int>> &_attrs) ;

        void attrs(std::unordered_map<std::string, mvreg<Attribute, int>> &&_attrs) ;


        std::unordered_map<std::string, mvreg<Attribute, int>> &attrs() & ;

        const std::unordered_map<std::string, mvreg<Attribute, int>> &attrs() const& ;

        void fano(const std::unordered_map<std::pair<int32_t, std::string>, mvreg<Edge, int>, CRDT::pair_hash> &_fano) ;

        void fano(std::unordered_map<std::pair<int32_t, std::string>, mvreg<Edge, int>, CRDT::pair_hash> &&_fano) ;

        std::unordered_map<std::pair<int32_t, std::string>, mvreg<Edge, int>, CRDT::pair_hash> &fano() ;

        const std::unordered_map<std::pair<int32_t, std::string>, mvreg<Edge, int>, CRDT::pair_hash> &fano() const ;


        [[nodiscard]] IDL::Node toIDLNode(int id) ;

    private:
        std::string m_type;
        std::string m_name;
        int32_t m_id;
        int32_t m_agent_id;
        std::unordered_map<std::string, mvreg<Attribute, int>> m_attrs;
        std::unordered_map<std::pair<int32_t, std::string>, mvreg<Edge, int>, CRDT::pair_hash> m_fano;
    };



    // Translators
    inline static IDL::Mvreg translateNodeMvCRDTtoIDL(int agent_id, int id, mvreg<Node, int> &data)
    {
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

    inline static mvreg<Node, int> translateNodeMvIDLtoCRDT(IDL::Mvreg &data)
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
        mvreg<Node, int> aw = mvreg<Node, int>(data.id());
        aw.dk.c = dotcontext_aux;
        aw.dk.set(ds_aux);
        return aw;
    }

    inline static IDL::MvregEdgeAttr translateEdgeAttrMvCRDTtoIDL(int agent_id, int id, int from, int to, const std::string& type, const std::string& attr, mvreg<Attribute, int> &data)
    {
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
        for (auto &kv_dc : data.context().dc){
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

    inline static mvreg<Attribute, int> translateEdgeAttrMvIDLtoCRDT(IDL::MvregEdgeAttr &data)
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
        std::map <pair<int, int>, Attribute> ds_aux;
        for (auto &[k,v] : data.dk().ds())
            ds_aux[pair<int, int>(k.first(), k.second())] = std::move(v);
        // Join
        mvreg<Attribute, int> aw = mvreg<Attribute, int>(data.id());
        aw.dk.c = dotcontext_aux;
        aw.dk.set(ds_aux);
        return aw;
    }

    inline static IDL::MvregNodeAttr translateNodeAttrMvCRDTtoIDL(int agent_id, int id, int node, const std::string& attr, mvreg<Attribute, int> &data)
    {
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

    inline static mvreg<Attribute, int> translateNodeAttrMvIDLtoCRDT(IDL::MvregNodeAttr &data)
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
        std::map <pair<int, int>, Attribute> ds_aux;
        for (auto &[k,v] : data.dk().ds())
            ds_aux[pair<int, int>(k.first(), k.second())] = std::move(v);
        // Join
        mvreg<Attribute, int> aw = mvreg<Attribute, int>(data.id());
        aw.dk.c = dotcontext_aux;
        aw.dk.set(ds_aux);
        return aw;
    }

    inline static mvreg<Edge, int> translateEdgeMvIDLtoCRDT(IDL::MvregEdge &data)
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
        std::map <pair<int, int>, Edge> ds_aux;
        for (auto &[k,v] : data.dk().ds())
            ds_aux[pair<int, int>(k.first(), k.second())] = std::move(v);
        // Join
        mvreg<Edge, int> aw = mvreg<Edge, int>(data.id());
        aw.dk.c = dotcontext_aux;
        aw.dk.set(ds_aux);
        return aw;
    }

    inline static IDL::MvregEdge translateEdgeMvCRDTtoIDL(int agent_id, int id, mvreg<Edge, int> &data)
    {
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
        for (auto &kv_dc : data.context().dc){
            IDL::PairInt pi;
            pi.first(kv_dc.first);
            pi.second(kv_dc.second);

            delta_crdt.dk().cbase().dc().push_back(pi);
        }
        delta_crdt.id(id);
        delta_crdt.agent_id(agent_id);
        return delta_crdt;
    }

}

#endif //CRDT_RTPS_DSR_CRDT_TYPES_H
