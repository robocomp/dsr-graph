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
        Test_utils(RoboCompDSRGetID::DSRGetIDPrxPtr id_prx) : dsrgetid_proxy(id_prx)  
        {
            mt = std::mt19937(rd());
            dist = std::uniform_real_distribution((float)-40.0, (float)40.0);
            randomNode = std::uniform_int_distribution((int)100, (int)140.0);
            random_pos = std::uniform_int_distribution((int)-200, (int)200);
            random_selector = std::uniform_int_distribution(0,1);
        };

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

        int rnd_selector() { return random_selector(mt); };
        int rnd_float() { return random_pos(mt); };

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
