//
// Created by juancarlos on 7/5/20.
//

#ifndef DSR_RTPS_DSR_TEST_concurrent_insert_remove_node_H
#define DSR_RTPS_DSR_TEST_concurrent_insert_remove_node_H

#include "DSR_test.h"


class DSR_concurrent_operations : DSR_test {
public:
    DSR_concurrent_operations () {};
    DSR_concurrent_operations(RoboCompDSRGetID::DSRGetIDPrxPtr id_prx, shared_ptr<DSR::DSRGraph> G_, const std::string& output_, const std::string& output_result_ , int num_ops_, int num_threads_, int agent_id_)
        : DSR_test(id_prx, G_, output_, output_result_), num_ops(num_ops_), num_threads(num_threads_), agent_id(agent_id_) {times.resize(num_ops*num_threads); };

    DSR_concurrent_operations& operator=(DSR_concurrent_operations&& t) {
        dsrgetid_proxy = std::move(t.dsrgetid_proxy);
        output_result = std::move(t.output_result);
        G = t.G;
        num_threads = t.num_threads;
        agent_id = t.agent_id;
        output = std::move(t.output);
        num_ops = t.num_ops;
        times.resize(num_ops*num_threads, 0);
        return *this;
    }

    ~DSR_concurrent_operations() {};

    void save_json_result();
    void run_test();

private:

    std::chrono::steady_clock::time_point start, end;
    int num_ops;
    int num_threads;
    int agent_id;
    std::vector<std::thread> threads;
    void concurrent_ops(int i, int no, const shared_ptr<DSR::DSRGraph>& G);
    int delay = 5; //ms

    double mean, ops_second;
    std::string result;

    std::mutex mtx;
};


#endif //DSR_RTPS_DSR_STRINGS_DSR_CONCURRENT_TEST_H
