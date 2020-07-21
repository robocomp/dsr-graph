//
// Created by juancarlos on 7/5/20.
//

#ifndef DSR_RTPS_DSR_STRINGS_DSR_change_attribute_H
#define DSR_RTPS_DSR_STRINGS_DSR_change_attribute_H

#include "DSR_test.h"


class DSR_change_attribute : DSR_test {
public:
    DSR_change_attribute () {};
    DSR_change_attribute(RoboCompDSRGetID::DSRGetIDPrxPtr id_prx,  shared_ptr<DSR::DSRGraph> G_, const std::string& output_, const std::string& output_result_, int num_ops_, int agent_id_)
        : DSR_test(id_prx, G_, output_, output_result_), num_ops(num_ops_), agent_id(agent_id_) { times.resize(num_ops);};

    DSR_change_attribute& operator=(DSR_change_attribute&& t) {
        dsrgetid_proxy = std::move(t.dsrgetid_proxy);
        output_result = std::move(t.output_result);
        G = t.G;
        output = std::move(t.output);
        num_ops = t.num_ops;
        times.resize(num_ops);
        return *this;
    }

    ~DSR_change_attribute () {};

    void save_json_result();
    void run_test();

private:

    int num_ops;

    void insert_or_assign_attributes(int i, const shared_ptr<DSR::DSRGraph>& G);

    int delay = 5; //ms
    int agent_id;

    double mean, ops_second;
    std::string result;
};


#endif //DSR_RTPS_DSR_STRINGS_DSR_CONCURRENT_TEST_H
