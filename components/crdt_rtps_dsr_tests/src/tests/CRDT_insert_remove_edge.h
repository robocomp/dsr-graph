//
// Created by juancarlos on 7/5/20.
//

#ifndef CRDT_RTPS_DSR_STRINGS_CRDT_insert_remove_edge_H
#define CRDT_RTPS_DSR_STRINGS_CRDT_insert_remove_edge_H

#include "../../../../graph-related-classes/CRDT.h"
#include "DSR_test.h"


class CRDT_insert_remove_edge : DSR_test {
public:
    CRDT_insert_remove_edge () {};
    CRDT_insert_remove_edge(RoboCompDSRGetID::DSRGetIDPrxPtr id_prx, shared_ptr<CRDT::CRDTGraph> G_, const std::string& output_, int num_ops_)
        : DSR_test(id_prx, G_, output_), num_ops(num_ops_) {};

    CRDT_insert_remove_edge& operator=(CRDT_insert_remove_edge&& t) {
        dsrgetid_proxy = std::move(t.dsrgetid_proxy);
        G = t.G;
        output = std::move(t.output);
        num_ops = t.num_ops;
        return *this;
    }

    ~CRDT_insert_remove_edge () {};

    void save_json_result();
    void run_test();

private:

    std::chrono::steady_clock::time_point start, end;
    int num_ops;


    void create_or_remove_edges(int i, const shared_ptr<CRDT::CRDTGraph>& G);

    int delay = 5; //ms

};


#endif //CRDT_RTPS_DSR_STRINGS_CRDT_CONCURRENT_TEST_H
