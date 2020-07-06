
#ifndef DSR_UTILS
#define DSR_UTILS

#include <string>
#include <optional>
#include "dsr_exceptions.h"
#include "../core/topics/DSRGraphPubSubTypes.h"
#include "../core/types/crdt_types.h"
#include "../core/types/user_types.h"

namespace CRDT {
    class CRDTGraph;

    class Utilities {
    public:
        Utilities(CRDTGraph *G_);

        void read_from_json_file(const std::string &json_file_path);

        void write_to_json_file(const std::string &json_file_path);

        void print();

        void print_edge(const CRDT::Edge &edge);

        void print_node(const CRDT::Node &node);

        void print_node(uint32_t id);

        void print_RT(uint32_t root);

    private:
        CRDT::CRDTGraph *G;

        void print_RT(const CRDT::Node &node);

    };
};

#endif