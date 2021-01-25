//
// Created by juancarlos on 7/5/20.
//

#ifndef CRDT_RTPS_DSR_delayed_start_H
#define CRDT_RTPS_DSR_delayed_start_H

#include "DSR_test.h"


class CRDT_delayed_start : DSR_test {
public:
    CRDT_delayed_start () = default;
    CRDT_delayed_start( std::shared_ptr<DSR::DSRGraph> G_, const std::string& output_, const std::string& output_result_ ,int num_ops_, int agent_id_)
        : DSR_test( std::move(G_), output_, output_result_), num_ops(num_ops_), agent_id(agent_id_) {times.resize(num_ops);};

    CRDT_delayed_start& operator=(CRDT_delayed_start&& t) {
        output_result = std::move(t.output_result);
        G = t.G;
        agent_id = t.agent_id;
        output = std::move(t.output);
        num_ops = t.num_ops;
        times.resize(num_ops);
        return *this;
    }

    ~CRDT_delayed_start () = default;

    void save_json_result() override;
    void run_test() override;

private:

    std::chrono::steady_clock::time_point start, end;
    int num_ops{0};
    int agent_id{0};

    void create_or_remove_nodes(int i, const std::shared_ptr<DSR::DSRGraph>& G);
    int delay = 5; //ms


    double mean{}, ops_second{};
    std::string result;
};


#endif //CRDT_RTPS_DSR_delayed_start_H
