//
// Created by juancarlos on 25/8/20.
//

#include "bench.h"
#include <benchmark/benchmark.h>
#include <random>

REGISTER_TYPE(int_, int32_t, false)
REGISTER_TYPE(float_, float, false)
REGISTER_TYPE(bool_, bool, false)
REGISTER_TYPE(uint_, uint32_t, false)
REGISTER_TYPE(string_, std::reference_wrapper<const string>, false)
REGISTER_TYPE(vec_byte, std::reference_wrapper<const std::vector<uint8_t>>, false)
REGISTER_TYPE(vec_float, std::reference_wrapper<const std::vector<float>>, false)


////////////////////////////////////
//////// Get node
///////////////////////////////////

static void get_node(benchmark::State& state) {
    auto G = Graph::get().get_G();

    for (auto _ : state) {
        auto node = G->get_node("world");
       benchmark::DoNotOptimize(node);
    }
}

BENCHMARK(get_node);

////////////////////////////////////
//////// Create node
///////////////////////////////////
/*
static void insert_node(benchmark::State& state) {
    auto G = Graph::get().get_G();

    for (auto _ : state) {
        DSR::Node n(G->get_agent_id(), "testtype");
        auto id_n = G->insert_node(n);
        benchmark::DoNotOptimize(id_n);
    }
}
*/
//BENCHMARK(insert_node);


//////////////////////////////////////
//////// Attribute updates
//////////////////////////////////////
static ConstantCircularBuffer<int, 10> int_input( []() -> int { return rand()%100; });
static ConstantCircularBuffer<float, 10> float_input( []() -> float { return static_cast<float>(rand())/static_cast<float>(10); });
static ConstantCircularBuffer<uint32_t, 10> uint_input( []() -> uint32_t { return static_cast<uint32_t>(rand()%100); });
static ConstantCircularBuffer<bool, 10> bool_input( []() -> uint32_t { return static_cast<bool>(rand()%2); });
static ConstantCircularBuffer<std::string, 10> short_string_input( []() -> std::string { return random_string<20>(); });
static ConstantCircularBuffer<std::string, 10> long_string_input( []() -> std::string { return random_string<200>(); });
static ConstantCircularBuffer<std::vector<float>, 10> short_vecfloat_input(
[]() -> std::vector<float> {
            std::vector<float> vec(5);
            generate(vec.begin(), vec.end(), [](){ return static_cast<float>(rand())/static_cast<float>(10);});
            return vec;
});
static ConstantCircularBuffer<std::vector<float>, 3> long_vecfloat_input(
[]() -> std::vector<float> {
    std::vector<float> vec(10000);
    generate(vec.begin(), vec.end(), [](){ return static_cast<float>(rand())/static_cast<float>(10);});
    return vec;
});
static ConstantCircularBuffer<std::vector<uint8_t>, 10> short_vecbyte_input(
        []() -> std::vector<uint8_t> {
            std::vector<uint8_t> vec(200);
            generate(vec.begin(), vec.end(), [](){ return static_cast<uint32_t>(rand()%100);});
            return vec;
        });
static ConstantCircularBuffer<std::vector<uint8_t>, 3> long_vecbyte_input(
        []() -> std::vector<uint8_t> {
            std::vector<uint8_t> vec(5000000);
            generate(vec.begin(), vec.end(), [](){ return static_cast<uint32_t>(rand()%100);});
            return vec;
        });

/*
 * Attribute updates using "update_attrib_by_name" for each supported type without deltas from other agents.
 * */


static void single_attribute_insert_int(benchmark::State& state) {
    std::array<std::string, 4> keys {"world", "omnirobot", "wall0", "floor_plane"};
    thread_local std::string k = keys[state.thread_index];

    auto G = Graph::get().get_G();

    auto node = G->get_node(k);
    G->insert_attrib_by_name<int__att>(node.value(), int_input.next());

    // Perform setup here
    for (auto _ : state) {
        auto node = G->get_node(k);
        G->update_attrib_by_name<int__att>(node.value(), int_input.next());
    }
}

static void single_attribute_insert_float(benchmark::State& state) {
    std::array<std::string, 4> keys {"world", "omnirobot", "wall0", "floor_plane"};
    thread_local  std::string k = keys[state.thread_index];
    auto G = Graph::get().get_G();

    auto node = G->get_node(k);
    G->insert_attrib_by_name<float__att>(node.value(), float_input.next());

    // Perform setup here
    for (auto _ : state) {
        auto node = G->get_node(k);
        G->update_attrib_by_name<float__att>(node.value(), float_input.next());
    }
}

static void single_attribute_insert_uint(benchmark::State& state) {
    std::array<std::string, 4> keys {"world", "omnirobot", "wall0", "floor_plane"};
    thread_local std::string k = keys[state.thread_index];
    auto G = Graph::get().get_G();

    auto node = G->get_node(k);
    G->insert_attrib_by_name<uint__att>(node.value(), uint_input.next());

    // Perform setup here
    for (auto _ : state) {
        auto node = G->get_node(k);
        G->update_attrib_by_name<uint__att>(node.value(), uint_input.next());
    }
}

static void single_attribute_insert_bool(benchmark::State& state) {
    std::array<std::string, 4> keys {"world", "omnirobot", "wall0", "floor_plane"};
    thread_local std::string k = keys[state.thread_index];
    auto G = Graph::get().get_G();

    auto node = G->get_node(k);
    G->insert_attrib_by_name<bool__att>(node.value(), bool_input.next());

    // Perform setup here
    for (auto _ : state) {
        auto node = G->get_node(k);
        G->update_attrib_by_name<bool__att>(node.value(), bool_input.next());
    }
}


static void single_attribute_insert_string_len_20(benchmark::State& state) {
    std::array<std::string, 4> keys {"world", "omnirobot", "wall0", "floor_plane"};
    thread_local std::string k = keys[state.thread_index];
    auto G = Graph::get().get_G();

    auto node = G->get_node(k);
    G->insert_attrib_by_name<string__att>(node.value(), short_string_input.next());

    // Perform setup here
    for (auto _ : state) {
        auto node = G->get_node(k);
        G->update_attrib_by_name<string__att>(node.value(), short_string_input.next());
    }
}

static void single_attribute_insert_string_len_200(benchmark::State& state) {
    std::array<std::string, 4> keys {"world", "omnirobot", "wall0", "floor_plane"};
    thread_local std::string k = keys[state.thread_index];
    auto G = Graph::get().get_G();

    auto node = G->get_node("world");
    G->insert_attrib_by_name<string__att>(node.value(), long_string_input.next());

    // Perform setup here
    for (auto _ : state) {
        auto node = G->get_node(k);
        G->update_attrib_by_name<string__att>(node.value(), long_string_input.next());
    }
}


static void single_attribute_insert_vecu8_len_200(benchmark::State& state) {
    std::array<std::string, 4> keys {"world", "omnirobot", "wall0", "floor_plane"};
    thread_local std::string k = keys[state.thread_index];
    auto G = Graph::get().get_G();


    auto node = G->get_node("world");
    G->insert_attrib_by_name<vec_byte_att>(node.value(), short_vecbyte_input.next());

    // Perform setup here
    for (auto _ : state) {
        auto node = G->get_node(k);
        G->update_attrib_by_name<vec_byte_att>(node.value(),  short_vecbyte_input.next());
    }
}

static void single_attribute_insert_vecu8_len_5000000(benchmark::State& state) {
    std::array<std::string, 4> keys {"world", "omnirobot", "wall0", "floor_plane"};
    thread_local std::string k = keys[state.thread_index];
    auto G = Graph::get().get_G();

    auto node = G->get_node("world");
    G->insert_attrib_by_name<vec_byte_att>(node.value(), long_vecbyte_input.next());

    // Perform setup here
    for (auto _ : state) {
        auto node = G->get_node(k);
        G->update_attrib_by_name<vec_byte_att>(node.value(), long_vecbyte_input.next());
    }
}

static void single_attribute_insert_vec_float_len_5(benchmark::State& state) {
    std::array<std::string, 4> keys {"world", "omnirobot", "wall0", "floor_plane"};
    thread_local std::string k = keys[state.thread_index];
    auto G = Graph::get().get_G();


    auto node = G->get_node("world");
    G->insert_attrib_by_name<vec_float_att>(node.value(), short_vecfloat_input.next());

    // Perform setup here
    for (auto _ : state) {
        auto node = G->get_node(k);
        G->update_attrib_by_name<vec_float_att>(node.value(), short_vecfloat_input.next());
    }
}

static void single_attribute_insert_vec_float_len_10000(benchmark::State& state) {
    std::array<std::string, 4> keys {"world", "omnirobot", "wall0", "floor_plane"};
    thread_local std::string k = keys[state.thread_index];
    auto G = Graph::get().get_G();


    auto node = G->get_node(k);
    G->insert_attrib_by_name<vec_float_att>(node.value(), long_vecfloat_input.next());

    // Perform setup here
    for (auto _ : state) {
        auto node = G->get_node(k);
        G->update_attrib_by_name<vec_float_att>(node.value(), long_vecfloat_input.next());
    }
}

BENCHMARK(single_attribute_insert_int)->  Threads(1); //-> Threads(2)->  Threads(3) -> Threads(4);
BENCHMARK(single_attribute_insert_float)->  Threads(1); // -> Threads(2)->  Threads(3) -> Threads(4);
BENCHMARK(single_attribute_insert_uint)->  Threads(1); // -> Threads(2)->  Threads(3) -> Threads(4);
BENCHMARK(single_attribute_insert_bool)->  Threads(1); // -> Threads(2)->  Threads(3) -> Threads(4);
BENCHMARK(single_attribute_insert_string_len_20)->  Threads(1); // -> Threads(2)->  Threads(3) -> Threads(4);
BENCHMARK(single_attribute_insert_string_len_200)->  Threads(1); // -> Threads(2)->  Threads(3) -> Threads(4);
BENCHMARK(single_attribute_insert_vecu8_len_5000000)->  Threads(1); // -> Threads(2)->  Threads(3) -> Threads(4);
BENCHMARK(single_attribute_insert_vecu8_len_200)->  Threads(1); // -> Threads(2)->  Threads(3) -> Threads(4);
BENCHMARK(single_attribute_insert_vec_float_len_5)->  Threads(1); // -> Threads(2)->  Threads(3) -> Threads(4);
BENCHMARK(single_attribute_insert_vec_float_len_10000)->  Threads(1); // -> Threads(2)->  Threads(3) -> Threads(4);



////////////////////////////////////
//////// Attribute updates (local)
///////////////////////////////////


/*
 * Attribute updates using "update_attrib_by_name" and "update_node" for each supported type without deltas from other agents.
 * */


static void single_attribute_insert_int_local(benchmark::State& state) {
    std::array<std::string, 4> keys {"world", "omnirobot", "wall0", "floor_plane"};
    thread_local std::string k = keys[state.thread_index];
    auto G = Graph::get().get_G();

    
    auto node = G->get_node(k);
    G->add_or_modify_attrib_local<int__att>(node.value(), int_input.next());
    G->update_node(node.value());

    // Perform setup here
    for (auto _ : state) {
        auto node = G->get_node(k);
        G->modify_attrib_local<int__att>(node.value(), int_input.next());
        G->update_node(node.value());
    }
}

static void single_attribute_insert_float_local(benchmark::State& state) {
    std::array<std::string, 4> keys {"world", "omnirobot", "wall0", "floor_plane"};
    thread_local std::string k = keys[state.thread_index];
    auto G = Graph::get().get_G();

    auto node = G->get_node(k);
    G->add_or_modify_attrib_local<float__att>(node.value(), float_input.next());
    G->update_node(node.value());

    // Perform setup here
    for (auto _ : state) {
        auto node = G->get_node(k);
        G->modify_attrib_local<float__att>(node.value(), float_input.next());
        G->update_node(node.value());
    }
}

static void single_attribute_insert_uint_local(benchmark::State& state) {
    std::array<std::string, 4> keys {"world", "omnirobot", "wall0", "floor_plane"};
    thread_local std::string k = keys[state.thread_index];
    auto G = Graph::get().get_G();

    auto node = G->get_node(k);
    G->add_or_modify_attrib_local<uint__att>(node.value(), uint_input.next());
    G->update_node(node.value());

    // Perform setup here
    for (auto _ : state) {
        auto node = G->get_node(k);
        G->modify_attrib_local<uint__att>(node.value(), uint_input.next());
        G->update_node(node.value());
    }
}

static void single_attribute_insert_bool_local(benchmark::State& state) {
    std::array<std::string, 4> keys {"world", "omnirobot", "wall0", "floor_plane"};
    thread_local std::string k = keys[state.thread_index];
    auto G = Graph::get().get_G();

    auto node = G->get_node(k);
    G->add_or_modify_attrib_local<bool__att>(node.value(), bool_input.next());
    G->update_node(node.value());

    // Perform setup here
    for (auto _ : state) {
        auto node = G->get_node(k);
        G->modify_attrib_local<bool__att>(node.value(), bool_input.next());
        G->update_node(node.value());
    }
}


static void single_attribute_insert_string_len_20_local(benchmark::State& state) {
    std::array<std::string, 4> keys {"world", "omnirobot", "wall0", "floor_plane"};
    thread_local std::string k = keys[state.thread_index];
    auto G = Graph::get().get_G();

    auto node = G->get_node(k);
    G->add_or_modify_attrib_local<string__att>(node.value(), short_string_input.next());
    G->update_node(node.value());

    // Perform setup here
    for (auto _ : state) {
        auto node = G->get_node(k);
        G->modify_attrib_local<string__att>(node.value(), short_string_input.next());
        G->update_node(node.value());
    }
}

static void single_attribute_insert_string_len_200_local(benchmark::State& state) {
    std::array<std::string, 4> keys {"world", "omnirobot", "wall0", "floor_plane"};
    thread_local std::string k = keys[state.thread_index];
    auto G = Graph::get().get_G();

    auto node = G->get_node(k);
    G->add_or_modify_attrib_local<string__att>(node.value(), long_string_input.next());
    G->update_node(node.value());

    // Perform setup here
    for (auto _ : state) {
        auto node = G->get_node(k);
        G->modify_attrib_local<string__att>(node.value(), long_string_input.next());
        G->update_node(node.value());
    }
}


static void single_attribute_insert_vecu8_len_200_local(benchmark::State& state) {
    std::array<std::string, 4> keys {"world", "omnirobot", "wall0", "floor_plane"};
    thread_local std::string k = keys[state.thread_index];
    auto G = Graph::get().get_G();


    auto node = G->get_node(k);
    G->add_or_modify_attrib_local<vec_byte_att>(node.value(), short_vecbyte_input.next());
    G->update_node(node.value());

    // Perform setup here
    for (auto _ : state) {
        auto node = G->get_node(k);
        G->modify_attrib_local<vec_byte_att>(node.value(), short_vecbyte_input.next());
        G->update_node(node.value());
    }
}

static void single_attribute_insert_vecu8_len_5000000_local(benchmark::State& state) {
    std::array<std::string, 4> keys {"world", "omnirobot", "wall0", "floor_plane"};
    thread_local std::string k = keys[state.thread_index];

    auto G = Graph::get().get_G();


    auto node = G->get_node(k);
    G->add_or_modify_attrib_local<vec_byte_att>(node.value(), long_vecbyte_input.next());
    G->update_node(node.value());

    // Perform setup here
    for (auto _ : state) {
        auto node = G->get_node(k);
        G->modify_attrib_local<vec_byte_att>(node.value(), long_vecbyte_input.next());
        G->update_node(node.value());
    }
}

static void single_attribute_insert_vec_float_len_5_local(benchmark::State& state) {
    std::array<std::string, 4> keys {"world", "omnirobot", "wall0", "floor_plane"};
    thread_local std::string k = keys[state.thread_index];

    auto G = Graph::get().get_G();


    auto node = G->get_node(k);
    G->add_or_modify_attrib_local<vec_float_att>(node.value(), short_vecfloat_input.next());
    G->update_node(node.value());

    // Perform setup here
    for (auto _ : state) {
        auto node = G->get_node(k);
        G->modify_attrib_local<vec_float_att>(node.value(), short_vecfloat_input.next());
        G->update_node(node.value());
    }
}

static void single_attribute_insert_vec_float_len_10000_local(benchmark::State& state) {
    std::array<std::string, 4> keys {"world", "omnirobot", "wall0", "floor_plane"};
    thread_local std::string k = keys[state.thread_index];

    auto G = Graph::get().get_G();


    auto node = G->get_node(k);
    G->insert_attrib_by_name<vec_float_att>(node.value(), long_vecfloat_input.next());
    G->update_node(node.value());

    // Perform setup here
    for (auto _ : state) {
        auto node = G->get_node(k);
        G->modify_attrib_local<vec_float_att>(node.value(), long_vecfloat_input.next());
        G->update_node(node.value());
    }
}

BENCHMARK(single_attribute_insert_int_local)->  Threads(1); // -> Threads(2)->  Threads(3) -> Threads(4);
BENCHMARK(single_attribute_insert_float_local)->  Threads(1); // -> Threads(2)->  Threads(3) -> Threads(4);
BENCHMARK(single_attribute_insert_uint_local)->  Threads(1) ; //-> Threads(2)->  Threads(3) -> Threads(4);
BENCHMARK(single_attribute_insert_bool_local)->  Threads(1); // -> Threads(2)->  Threads(3) -> Threads(4);
BENCHMARK(single_attribute_insert_string_len_20_local)->  Threads(1); // -> Threads(2)->  Threads(3) -> Threads(4);
BENCHMARK(single_attribute_insert_string_len_200_local)->  Threads(1) ; //-> Threads(2)->  Threads(3) -> Threads(4);
BENCHMARK(single_attribute_insert_vecu8_len_5000000_local)->  Threads(1); // -> Threads(2)->  Threads(3) -> Threads(4);
BENCHMARK(single_attribute_insert_vecu8_len_200_local)->  Threads(1); // -> Threads(2)->  Threads(3) -> Threads(4);
BENCHMARK(single_attribute_insert_vec_float_len_5_local)->  Threads(1); // -> Threads(2)->  Threads(3) -> Threads(4);
BENCHMARK(single_attribute_insert_vec_float_len_10000_local)->  Threads(1); // -> Threads(2)->  Threads(3) -> Threads(4);



////////////////////////////////////
//////// Attribute 5 attributes
///////////////////////////////////

static void update_5_attr(benchmark::State& state) {
    auto G = Graph::get().get_G();
    auto node = G->get_node("world");

    for (auto _ : state) {
        auto node = G->get_node("world");
        G->update_attrib_by_name<vec_float_att>(node.value(), short_vecfloat_input.next());
        G->update_attrib_by_name<vec_byte_att>(node.value(), short_vecbyte_input.next());
        G->update_attrib_by_name<int__att>(node.value(), int_input.next());
        G->update_attrib_by_name<string__att>(node.value(), short_string_input.next());
        G->update_attrib_by_name<bool__att>(node.value(), bool_input.next());
    }
}

static void update_5_attr_local_and_update(benchmark::State& state) {
    auto G = Graph::get().get_G();
    auto node = G->get_node("world");


    for (auto _ : state) {
        auto node = G->get_node("world");
        G->modify_attrib_local<vec_float_att>(node.value(), short_vecfloat_input.next());
        G->modify_attrib_local<vec_byte_att>(node.value(), short_vecbyte_input.next());
        G->modify_attrib_local<int__att>(node.value(),  int_input.next());
        G->modify_attrib_local<string__att>(node.value(), short_string_input.next());
        G->modify_attrib_local<bool__att>(node.value(), bool_input.next());
        G->update_node(node.value());
    }
}

BENCHMARK(update_5_attr_local_and_update);
BENCHMARK(update_5_attr);

BENCHMARK_MAIN();