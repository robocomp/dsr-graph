//
// Created by juancarlos on 7/5/20.
//

#ifndef CRDT_RTPS_DSR_TEST_concurrent_insert_remove_node_H
#define CRDT_RTPS_DSR_TEST_concurrent_insert_remove_node_H

#include "../../../../graph-related-classes/CRDT.h"
#include "DSR_test.h"


class CRDT_concurrent_operations : DSR_test {
public:
    CRDT_concurrent_operations () {};
    CRDT_concurrent_operations(RoboCompDSRGetID::DSRGetIDPrxPtr id_prx, shared_ptr<CRDT::CRDTGraph> G_, const std::string& output_, int num_ops_, int num_threads_)
        : DSR_test(id_prx, G_, output_), num_ops(num_ops_), num_threads(num_threads_) {};

    CRDT_concurrent_operations& operator=(CRDT_concurrent_operations&& t) {
        dsrgetid_proxy = std::move(t.dsrgetid_proxy);
        G = t.G;
        num_threads = t.num_threads;
        output = std::move(t.output);
        num_ops = t.num_ops;
        return *this;
    }

    ~CRDT_concurrent_operations() {};

    void save_json_result();
    void run_test();

private:

    std::chrono::steady_clock::time_point start, end;
    int num_ops;
    int num_threads;
    std::vector<std::thread> threads;
    void create_or_remove_nodes(int i, const shared_ptr<CRDT::CRDTGraph>& G);
    int delay = 5; //ms

};


#endif //CRDT_RTPS_DSR_STRINGS_CRDT_CONCURRENT_TEST_H
