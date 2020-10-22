//
// Created by juancarlos on 25/8/20.
//

#ifndef BENCH_H
#define BENCH_H

#include "dsr/api/dsr_api.h"
#include <iostream>
#include <string>

// ICE include
#include <Ice/Ice.h>

class Graph {
public:
    static Graph& get() {
        static Graph instance;
        return instance;
    }

    Graph(Graph const&)           = delete;
    void operator=(Graph const&)  = delete;

    std::shared_ptr<DSR::DSRGraph> get_G() { return G;}

private:
    Graph () {
        std::thread([](){

            [[maybe_unused]] auto _ = std::system("/home/robocomp/robocomp/components/dsr-graph/components/idserver/bin/idserver --Ice.Config=/home/robocomp/robocomp/components/dsr-graph/components/crdt_rtps_dsr_tests/src/benchmark/config_idserver");
        }).detach();

        std::this_thread::sleep_for(300ms);
        auto c = Ice::initialize();
        auto pr = c->stringToProxy("dsrgetid:tcp -h localhost -p 11000");
        dsrgetid_proxy = Ice::uncheckedCast<RoboCompDSRGetID::DSRGetIDPrx>( pr );
        G = std::make_shared<DSR::DSRGraph>(0, "test", 1551, "", dsrgetid_proxy);
        std::atexit([](){
            [[maybe_unused]] auto _ = std::system("kill -9 $(ps -xa  | grep /home/robocomp/robocomp/components/dsr-graph/components/idserver/bin/idserver | awk 'NR==2{print $1}')");
        });
    }
    RoboCompDSRGetID::DSRGetIDPrxPtr dsrgetid_proxy;
    std::shared_ptr<DSR::DSRGraph> G;
    std::shared_ptr<DSR::DSRGraph> G_2;

};


template<int len>
std::string random_string()
{
    std::string chars = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz_-*?!{}&";
    std::string str;
    int pos;
    while(str.size() != len) {
        pos = (rand() % chars.size());
        str += chars.substr(pos,1);
    }
    return str;
}

template<class T, std::size_t size = 150>
class ConstantCircularBuffer {
    std::array<T, size> container;
    int idx = -1;

public:
    explicit constexpr ConstantCircularBuffer(std::function<T()> fn) {
        std::generate(container.begin(), container.end(), fn);
    }

    constexpr ConstantCircularBuffer(std::initializer_list<T> l) {
        container = std::array<T, size>{l};
    }

    constexpr auto next() -> T& {
        idx = (idx + 1) % size;
        return container.at(idx);
    }


};
#endif //BENCH_H
