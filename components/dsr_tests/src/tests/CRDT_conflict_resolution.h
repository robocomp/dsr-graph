//
// Created by juancarlos on 15/5/20.
//

#ifndef DSR_CONCURRENT_TEST_CONFLICTS_OPS_H
#define DSR_CONCURRENT_TEST_CONFLICTS_OPS_H

#include "DSR_test.h"


class CRDT_conflict_resolution : DSR_test {
public:
    CRDT_conflict_resolution () = delete;
    ~CRDT_conflict_resolution () = default;
    CRDT_conflict_resolution( std::shared_ptr<DSR::DSRGraph> G_, const std::string& output_, const std::string& output_result_ ,int num_ops_, uint32_t agent_id_, int delay_)
        : DSR_test( std::move(G_), output_, output_result_), num_ops(num_ops_), agent_id(agent_id_), delay(delay_) {times.resize(num_ops);};

    void save_json_result() override;
    void run_test() override;

private:


    void insert_or_assign_attributes(const std::shared_ptr<DSR::DSRGraph>& G);


    std::chrono::steady_clock::time_point start, end;
    int num_ops{};
    uint32_t agent_id{};
    int delay{}; //ms
    std::string result;
    double mean{}, ops_second{};
};


#endif //
