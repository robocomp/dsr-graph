//
// Created by juancarlos on 7/5/20.
//

#ifndef DSR_RTPS_DSR_STRINGS_DSR_CONCURRENT_TEST_H
#define DSR_RTPS_DSR_STRINGS_DSR_CONCURRENT_TEST_H

#include "DSR_test.h"


class DSR_insert_remove_node : DSR_test {
public:
    DSR_insert_remove_node () {};
    DSR_insert_remove_node(RoboCompDSRGetID::DSRGetIDPrxPtr id_prx, shared_ptr<DSR::DSRGraph> G_, const std::string& output_, const std::string& output_result_ , int num_ops_)
        : DSR_test(id_prx, G_, output_, output_result_), num_ops(num_ops_) {times.resize(num_ops);};

    DSR_insert_remove_node& operator=(DSR_insert_remove_node&& t) {
        dsrgetid_proxy = std::move(t.dsrgetid_proxy);
        G = t.G;
        output = std::move(t.output);
        output_result = std::move(t.output_result);
        num_ops = t.num_ops;
        times.resize(num_ops);
        return *this;
    }

    ~DSR_insert_remove_node () {};

    void save_json_result();
    void run_test();

private:

    std::chrono::steady_clock::time_point start, end;
    int num_ops;


    void create_or_remove_nodes(int i, const shared_ptr<DSR::DSRGraph>& G);
    int delay = 5; //ms

    double mean, ops_second;
    std::string result;
};


#endif //DSR_RTPS_DSR_STRINGS_DSR_CONCURRENT_TEST_H
