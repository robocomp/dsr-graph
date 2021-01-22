//
// Created by juancarlos on 7/5/20.
//

#ifndef CRDT_RTPS_DSR_STRINGS_CRDT_insert_remove_edge_H
#define CRDT_RTPS_DSR_STRINGS_CRDT_insert_remove_edge_H

#include "DSR_test.h"


class CRDT_insert_remove_edge : DSR_test {
public:
    CRDT_insert_remove_edge () = default;
    CRDT_insert_remove_edge( std::shared_ptr<DSR::DSRGraph> G_, const std::string& output_, const std::string& output_result_ , int num_ops_)
        : DSR_test( std::move(G_), output_,output_result_), num_ops(num_ops_) {times.resize(num_ops);};

    CRDT_insert_remove_edge& operator=(CRDT_insert_remove_edge&& t) {
        G = t.G;
        output = std::move(t.output);
        output_result = std::move(t.output_result);
        num_ops = t.num_ops;
        times.resize(num_ops);
        return *this;
    }

    ~CRDT_insert_remove_edge () = default;

    void save_json_result() override;
    void run_test() override;

private:

    std::chrono::steady_clock::time_point start, end;
    int num_ops{};


    void create_or_remove_edges(int i, const std::shared_ptr<DSR::DSRGraph>& G);

    int delay = 5; //ms

    double mean{}, ops_second{};
    std::string result;
};


#endif //CRDT_RTPS_DSR_STRINGS_CRDT_CONCURRENT_TEST_H
