//
// Created by juancarlos on 15/5/20.
//

#ifndef DSR_RTPS_DSR_DSR_conflict_resolution_H
#define DSR_RTPS_DSR_DSR_conflict_resolution_H

#include "DSR_test.h"


class DSR_conflict_resolution : DSR_test {
public:
    DSR_conflict_resolution () {};
    DSR_conflict_resolution(RoboCompDSRGetID::DSRGetIDPrxPtr id_prx,  shared_ptr<DSR::DSRGraph> G_, const std::string& output_, const std::string& output_result_ ,int num_ops_, int agent_id_)
        : DSR_test(id_prx, G_, output_, output_result_), num_ops(num_ops_), agent_id(agent_id_) {times.resize(num_ops);};

    DSR_conflict_resolution& operator=(DSR_conflict_resolution&& t) {
        dsrgetid_proxy = std::move(t.dsrgetid_proxy);
        output_result = std::move(t.output_result);
        G = t.G;
        output = std::move(t.output);
        num_ops = t.num_ops;
        times.resize(num_ops);
        return *this;
    }

    ~DSR_conflict_resolution () {};

    void save_json_result();
    void run_test();

private:

    std::chrono::steady_clock::time_point start, end;
    int num_ops;

    void insert_or_assign_attributes(int i, const shared_ptr<DSR::DSRGraph>& G);

    int delay = 5; //ms
    int agent_id;

    double mean, ops_second;
    std::string result;
};


#endif //DSR_RTPS_DSR_DSR_conflict_resolution_H
