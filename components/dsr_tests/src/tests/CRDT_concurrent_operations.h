//
// Created by juancarlos on 7/5/20.
//

#ifndef DSR_CONCURRENT_TEST_OPS_H
#define DSR_CONCURRENT_TEST_OPS_H

#include "DSR_test.h"
#include "dsr/api/dsr_api.h"


class CRDT_concurrent_operations : DSR_test {
public:
    CRDT_concurrent_operations () = delete;
    ~CRDT_concurrent_operations() = default;

    CRDT_concurrent_operations( std::shared_ptr<DSR::DSRGraph> G_, const std::string& output_, const std::string& output_result_ , int num_ops_, int num_threads_, uint32_t agent_id_, int delay_)
        : DSR_test( std::move(G_), output_, output_result_), num_ops(num_ops_), num_threads(num_threads_), agent_id(agent_id_), delay(delay_) {times.resize(num_ops*num_threads); };

    void save_json_result() override;
    void run_test() override;

private:
    void concurrent_ops(int i, int no, const std::shared_ptr<DSR::DSRGraph>& G);


    std::chrono::steady_clock::time_point start, end;
    std::vector<std::thread> threads;
    int num_ops{0};
    int num_threads{0};
    uint32_t agent_id{0};
    int delay{}; //ms
    double mean{0.0}, ops_second{0.0};
    std::string result;
    std::mutex mtx;
};


#endif
