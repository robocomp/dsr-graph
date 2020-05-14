//
// Created by juancarlos on 7/5/20.
//

#ifndef CRDT_RTPS_DSR_STRINGS_CRDT_change_attribute_H
#define CRDT_RTPS_DSR_STRINGS_CRDT_change_attribute_H

#include "../../../../graph-related-classes/CRDT.h"
#include "DSR_test.h"


class CRDT_change_attribute : DSR_test {
public:
    CRDT_change_attribute () {};
    CRDT_change_attribute(RoboCompDSRGetID::DSRGetIDPrxPtr id_prx,  shared_ptr<CRDT::CRDTGraph> G_, const std::string& output_,int num_ops_, int agent_id_)
        : DSR_test(id_prx, G_, output_), num_ops(num_ops_), agent_id(agent_id_) {};

    CRDT_change_attribute& operator=(CRDT_change_attribute&& t) {
        dsrgetid_proxy = std::move(t.dsrgetid_proxy);
        G = t.G;
        output = std::move(t.output);
        num_ops = t.num_ops;
        return *this;
    }

    ~CRDT_change_attribute () {};

    void save_json_result();
    void run_test();

private:

    std::chrono::steady_clock::time_point start, end;
    int num_ops;

    void insert_or_assign_attributes(int i, const shared_ptr<CRDT::CRDTGraph>& G);

    int delay = 5; //ms
    int agent_id;
};


#endif //CRDT_RTPS_DSR_STRINGS_CRDT_CONCURRENT_TEST_H
