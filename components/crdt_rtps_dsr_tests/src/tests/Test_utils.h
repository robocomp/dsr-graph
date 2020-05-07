//
// Created by juancarlos on 7/5/20.
//

#ifndef CRDT_RTPS_DSR_STRINGS_TEST_UTILS_H
#define CRDT_RTPS_DSR_STRINGS_TEST_UTILS_H

#include <vector>
#include <iostream>
#include <utility>
#include <random>
#include <mutex>
#include <DSRGetID.h>

static std::string MARKER = ";";

class Test_utils {

    std::shared_ptr<RoboCompDSRGetID::DSRGetIDPrx>  dsrgetid_proxy;

    public:
        Test_utils() {};
        Test_utils(RoboCompDSRGetID::DSRGetIDPrxPtr id_prx)
        : dsrgetid_proxy(id_prx)  {};


        Test_utils& operator=(Test_utils&& t) {
            dsrgetid_proxy = t.dsrgetid_proxy;
            return *this;
        }

        void addEdgeIDs(int from, int to);
        std::pair<int, int> removeEdgeIDs();
        std::pair<int, int> getEdgeIDs();

        int newID();
        int removeID();
        int getID();

    private:
        std::random_device rd;
        std::mt19937 mt;
        std::uniform_real_distribution<float> dist;
        std::uniform_int_distribution<int> randomNode, random_selector, node_selector, random_pos;

        std::vector<int> created_nodos;
        std::vector<std::pair<int, int>> created_edges;
        std::mutex mut;



};


#endif //CRDT_RTPS_DSR_STRINGS_TEST_UTILS_H
