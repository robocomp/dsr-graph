//
// Created by juancarlos on 7/5/20.
//

#ifndef DSR_TEST_BASE_H
#define DSR_TEST_BASE_H

#include <random>
#include <utility>
#include "dsr/api/dsr_api.h"

static const std::string MARKER = ";";

struct Operation {
    uint32_t agent_id;
    uint64_t timestamp;
    std::string operation;
    bool result;

    std::string operator()() const {
        return std::to_string(agent_id)+ ";" + std::to_string(timestamp) +";"+ operation+ ";" + std::to_string(result);
    }
};

class DSR_test {
public:

    DSR_test() {
        mt = std::mt19937(rd());
        dist = std::uniform_real_distribution((float)-40.0, (float)40.0);
        random_pos = std::uniform_int_distribution((int)-200, (int)200);
        random_selector = std::uniform_int_distribution(0,1);
    };

    DSR_test( std::shared_ptr<DSR::DSRGraph> G_, std::string  output_,std::string  output_result_) :
     G(std::move(G_)), output(std::move(output_)), output_result(std::move(output_result_))
    {
            mt = std::mt19937(rd());
            dist = std::uniform_real_distribution((float)-40.0, (float)40.0);
            random_pos = std::uniform_int_distribution((int)-200, (int)200);
            random_selector = std::uniform_int_distribution(0,1);
    };

    //virtual ~DSR_test() {};

    virtual void save_json_result() = 0;
    virtual void run_test() = 0;

protected:
    void addEdgeIDs(uint64_t from, uint64_t to);
    std::pair<uint64_t , uint64_t> removeEdgeIDs();
    std::pair<uint64_t, uint64_t> getEdgeIDs();

    uint64_t removeID();
    uint64_t getID();

    int rnd_selector() { return random_selector(mt); };
    float rnd_float() { return random_pos(mt); };

    std::vector<int> times;
    std::chrono::steady_clock::time_point start, end, start_global, end_global;
    std::shared_ptr<DSR::DSRGraph> G;

    std::string output;
    std::string output_result;
    std::mt19937 mt;

private:

    std::random_device rd;
    std::uniform_real_distribution<float> dist;
    std::uniform_int_distribution<int> random_selector, random_pos;

    std::vector<std::pair<uint64_t, uint64_t>> created_edges;
    std::shared_mutex mut;
protected:
    std::vector<uint64_t> created_nodes;
    std::vector<Operation> operations;
};


#endif //DSR_TEST_BASE_H
