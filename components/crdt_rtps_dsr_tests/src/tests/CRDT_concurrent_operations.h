//
// Created by juancarlos on 7/5/20.
//

#ifndef CRDT_RTPS_DSR_TEST_concurrent_insert_remove_node_H
#define CRDT_RTPS_DSR_TEST_concurrent_insert_remove_node_H

#include "DSR_test.h"
#include "dsr/api/dsr_api.h"


class CRDT_concurrent_operations : DSR_test {
public:
    CRDT_concurrent_operations () = default;
    CRDT_concurrent_operations( std::shared_ptr<DSR::DSRGraph> G_, const std::string& output_, const std::string& output_result_ , int num_ops_, int num_threads_, int agent_id_)
        : DSR_test( std::move(G_), output_, output_result_), num_ops(num_ops_), num_threads(num_threads_), agent_id(agent_id_) {times.resize(num_ops*num_threads); };

    CRDT_concurrent_operations& operator=(CRDT_concurrent_operations&& t) {
        output_result = std::move(t.output_result);
        G = t.G;
        num_threads = t.num_threads;
        agent_id = t.agent_id;
        output = std::move(t.output);
        num_ops = t.num_ops;
        times.resize(num_ops*num_threads, 0);
        return *this;
    }

    ~CRDT_concurrent_operations() = default;

    void save_json_result() override;
    void run_test() override;

private:

    std::chrono::steady_clock::time_point start, end;
    int num_ops{0};
    int num_threads{0};
    int agent_id{0};
    std::vector<std::thread> threads;
    void concurrent_ops(int i, int no, const std::shared_ptr<DSR::DSRGraph>& G);
    int delay = 5; //ms

    double mean{0.0}, ops_second{0.0};
    std::string result;

    std::mutex mtx;
};


#endif //CRDT_RTPS_DSR_STRINGS_CRDT_CONCURRENT_TEST_H
