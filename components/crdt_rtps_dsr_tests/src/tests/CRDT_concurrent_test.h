//
// Created by juancarlos on 7/5/20.
//

#ifndef CRDT_RTPS_DSR_STRINGS_CRDT_CONCURRENT_TEST_H
#define CRDT_RTPS_DSR_STRINGS_CRDT_CONCURRENT_TEST_H

#include "../../../../graph-related-classes/CRDT.h"
#include "DSR_test.h"


class CRDT_concurrent_test : DSR_test {
public:
    CRDT_concurrent_test () {};
    CRDT_concurrent_test(RoboCompDSRGetID::DSRGetIDPrxPtr id_prx,  shared_ptr<CRDT::CRDTGraph> G_, const std::string& output_,int num_ops_, int num_threads_)
        : DSR_test(id_prx, G_, output_), num_threads(num_threads_), num_ops(num_ops_) {};

    CRDT_concurrent_test& operator=(CRDT_concurrent_test&& t) {
        dsrgetid_proxy = std::move(t.dsrgetid_proxy);
        G = t.G;
        output = std::move(t.output);
        num_threads = t.num_threads;
        num_ops = t.num_ops;
        return *this;
    }

    ~CRDT_concurrent_test () {};

    void save_json_result();
    void run_test();

private:

    std::chrono::steady_clock::time_point start, end;
    int num_threads;
    int num_ops;
    std::vector<std::thread> threads;


    void create_or_remove_nodes(int i, const shared_ptr<CRDT::CRDTGraph>& G);
    void create_or_remove_edges(int i, const shared_ptr<CRDT::CRDTGraph>& G);
    void insert_or_assign_attributes(int i, const shared_ptr<CRDT::CRDTGraph>& G);

    int delay = 5; //ms

};


#endif //CRDT_RTPS_DSR_STRINGS_CRDT_CONCURRENT_TEST_H
