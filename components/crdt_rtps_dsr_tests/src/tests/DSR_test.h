//
// Created by juancarlos on 7/5/20.
//

#ifndef CRDT_RTPS_DSR_STRINGS_DSR_TEST_H
#define CRDT_RTPS_DSR_STRINGS_DSR_TEST_H

#include <DSRGetID.h>
#include <random>
#include "../../../../graph-related-classes/CRDT.h"

static const std::string MARKER = ";";

class DSR_test {
public:

    DSR_test() {
        mt = std::mt19937(rd());
        dist = std::uniform_real_distribution((float)-40.0, (float)40.0);
        random_pos = std::uniform_int_distribution((int)-200, (int)200);
        random_selector = std::uniform_int_distribution(0,1);
    };
    DSR_test(RoboCompDSRGetID::DSRGetIDPrxPtr id_prx,  shared_ptr<CRDT::CRDTGraph> G_, const std::string& output_,const std::string& output_result_) :
    dsrgetid_proxy(id_prx), G(G_), output(output_), output_result(output_result_)
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
    void addEdgeIDs(int from, int to);
    std::pair<int, int> removeEdgeIDs();
    std::pair<int, int> getEdgeIDs();

    int newID();
    int removeID();
    int getID();

    int rnd_selector() { return random_selector(mt); };
    float rnd_float() { return random_pos(mt); };

    std::vector<int> times;
    std::chrono::steady_clock::time_point start, end, start_global, end_global;
    std::shared_ptr<RoboCompDSRGetID::DSRGetIDPrx>  dsrgetid_proxy;
    std::shared_ptr<CRDT::CRDTGraph> G;

    std::string output;
    std::string output_result;
    std::mt19937 mt;




private:

    std::random_device rd;
    //std::mt19937 mt;
    std::uniform_real_distribution<float> dist;
    std::uniform_int_distribution<int> random_selector, random_pos;

    std::vector<int> created_nodos;
    std::vector<std::pair<int, int>> created_edges;
    std::mutex mut;

};


#endif //CRDT_RTPS_DSR_STRINGS_DSR_TEST_H
