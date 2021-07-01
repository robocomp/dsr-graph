//
// Created by juancarlos on 7/5/20.
//

#ifndef DSR_CONCURRENT_TEST_EDGE_OPS_H
#define DSR_CONCURRENT_TEST_EDGE_OPS_H

#include "DSR_test.h"


class CRDT_insert_remove_edge : DSR_test {
public:
    CRDT_insert_remove_edge () = delete;
    ~CRDT_insert_remove_edge () = default;

    CRDT_insert_remove_edge( std::shared_ptr<DSR::DSRGraph> G_, const std::string& output_, const std::string& output_result_ , int num_ops_, int delay_)
        : DSR_test( std::move(G_), output_,output_result_), num_ops(num_ops_), delay(delay_) {times.resize(num_ops);};


    void save_json_result() override;
    void run_test() override;

private:

    void create_or_remove_edges(const std::shared_ptr<DSR::DSRGraph>& G);


    std::chrono::steady_clock::time_point start, end;
    int num_ops{};
    int delay{}; //ms
    double mean{}, ops_second{};
    std::string result;
};


#endif //
