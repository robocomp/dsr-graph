//
// Created by juancarlos on 15/5/20.
//

#ifndef CRDT_RTPS_DSR_CRDT_conflict_resolution_H
#define CRDT_RTPS_DSR_CRDT_conflict_resolution_H

#include "DSR_test.h"


class CRDT_conflict_resolution : DSR_test {
public:
    CRDT_conflict_resolution () = default;
    CRDT_conflict_resolution( std::shared_ptr<DSR::DSRGraph> G_, const std::string& output_, const std::string& output_result_ ,int num_ops_, int agent_id_)
        : DSR_test( std::move(G_), output_, output_result_), num_ops(num_ops_), agent_id(agent_id_) {times.resize(num_ops);};

    CRDT_conflict_resolution& operator=(CRDT_conflict_resolution&& t) {
        output_result = std::move(t.output_result);
        G = t.G;
        output = std::move(t.output);
        num_ops = t.num_ops;
        times.resize(num_ops);
        return *this;
    }

    ~CRDT_conflict_resolution () = default;

    void save_json_result() override;
    void run_test() override;

private:

    std::chrono::steady_clock::time_point start, end;
    int num_ops{0};

    void insert_or_assign_attributes(int i, const std::shared_ptr<DSR::DSRGraph>& G);

    int delay = 5; //ms
    int agent_id;

    double mean, ops_second;
    std::string result;
};


#endif //CRDT_RTPS_DSR_CRDT_conflict_resolution_H
