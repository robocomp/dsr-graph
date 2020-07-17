
#ifndef DSR_UTILS
#define DSR_UTILS

#include <string>
#include <optional>
#include "dsr_exceptions.h"
#include "../core/topics/IDLGraphPubSubTypes.h"
#include "../core/types/crdt_types.h"
#include "../core/types/user_types.h"

namespace DSR {

    class DSRGraph;

    class Utilities {
    public:
        Utilities(DSRGraph *G_);

        void read_from_json_file(const std::string &json_file_path, std::function<std::optional<int>(const Node&)> insert_node);

        void write_to_json_file(const std::string &json_file_path);

        void print();

        void print_edge(const DSR::Edge &edge);

        void print_node(const DSR::Node &node);

        void print_node(uint32_t id);

        void print_RT(uint32_t root);

    private:
        DSR::DSRGraph *G;

        void print_RT(const DSR::Node &node);

    };
};

#endif