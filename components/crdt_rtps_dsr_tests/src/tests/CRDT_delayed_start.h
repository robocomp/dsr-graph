//
// Created by juancarlos on 7/5/20.
//

#ifndef DSR_RTPS_DSR_delayed_start_H
#define DSR_RTPS_DSR_delayed_start_H

#include "DSR_test.h"


class DSR_delayed_start : DSR_test {
public:
    DSR_delayed_start () {};
    DSR_delayed_start(RoboCompDSRGetID::DSRGetIDPrxPtr id_prx, shared_ptr<DSR::DSRGraph> G_, const std::string& output_, const std::string& output_result_ ,int num_ops_, int agent_id_)
        : DSR_test(id_prx, G_, output_, output_result_), num_ops(num_ops_), agent_id(agent_id_) {times.resize(num_ops);};

    DSR_delayed_start& operator=(DSR_delayed_start&& t) {
        dsrgetid_proxy = std::move(t.dsrgetid_proxy);
        output_result = std::move(t.output_result);
        G = t.G;
        agent_id = t.agent_id;
        output = std::move(t.output);
        num_ops = t.num_ops;
        times.resize(num_ops);
        return *this;
    }

    ~DSR_delayed_start () {};

    void save_json_result();
    void run_test();

private:

    std::chrono::steady_clock::time_point start, end;
    int num_ops;
    int agent_id;

    void create_or_remove_nodes(int i, const shared_ptr<DSR::DSRGraph>& G);
    int delay = 5; //ms


    double mean, ops_second;
    std::string result;
};


#endif //DSR_RTPS_DSR_delayed_start_H
