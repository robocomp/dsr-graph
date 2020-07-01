
#ifndef DSR_UTILS
#define DSR_UTILS

#include <string>
#include <optional>
#include "dsr_exceptions.h"
#include "../core/topics/DSRGraphPubSubTypes.h"
#include "../core/types/crdt_types.h"

namespace CRDT {
    class CRDTGraph;

    class Utilities {
    public:
        Utilities(CRDTGraph *G_);

        void read_from_json_file(const std::string &json_file_path);

        void write_to_json_file(const std::string &json_file_path);

        void print();

        void print_edge(const CRDT::CRDTEdge &edge);

        void print_node(const CRDT::CRDTNode &node);

        void print_node(int id);

        void print_RT(std::int32_t root);

    private:
        CRDT::CRDTGraph *G;

        void print_RT(const CRDT::CRDTNode &node);

    };
};

#endif