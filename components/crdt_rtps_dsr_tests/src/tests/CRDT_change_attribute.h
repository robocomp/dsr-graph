//
// Created by juancarlos on 7/5/20.
//

#ifndef CRDT_RTPS_DSR_STRINGS_CRDT_change_attribute_H
#define CRDT_RTPS_DSR_STRINGS_CRDT_change_attribute_H

#include "DSR_test.h"


class CRDT_change_attribute : DSR_test {
public:
    CRDT_change_attribute () = default;
    CRDT_change_attribute(  std::shared_ptr<DSR::DSRGraph> G_, const std::string& output_, const std::string& output_result_, int num_ops_, int agent_id_)
        : DSR_test(std::move(G_), output_, output_result_), num_ops(num_ops_), agent_id(agent_id_) { times.resize(num_ops);};

    CRDT_change_attribute& operator=(CRDT_change_attribute&& t) {
        output_result = std::move(t.output_result);
        G = t.G;
        output = std::move(t.output);
        num_ops = t.num_ops;
        times.resize(num_ops);
        return *this;
    }

    ~CRDT_change_attribute () = default;

    void save_json_result() override;
    void run_test() override;

private:

    int num_ops{0};

    void insert_or_assign_attributes(int i, const std::shared_ptr<DSR::DSRGraph>& G);

    int delay = 5; //ms
    int agent_id{0};

    double mean{0.0}, ops_second{0.0};
    std::string result;
};


#endif //CRDT_RTPS_DSR_STRINGS_CRDT_CONCURRENT_TEST_H
