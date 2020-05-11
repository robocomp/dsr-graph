//
// Created by juancarlos on 7/5/20.
//

#include "DSR_test.h"

// This has to be a RPC call to the idserver component
// create and insert a new id in the list
int DSR_test::newID()
{
    int node_id;
    try{
        node_id = dsrgetid_proxy->getID();
        created_nodos.push_back(node_id);
        qDebug() <<"New nodeID: "<< node_id;
    }catch(...)
    {
        qDebug()<<"Error getting new nodeID from idserver, check connection";
    }
    return node_id;
}
// pick a random id from the list of new ones
int DSR_test::removeID()
{
    std::lock_guard<std::mutex>  lock(mut);
    if(created_nodos.size()==0)
        return -1;
    auto node_randomizer = std::uniform_int_distribution(0, (int)created_nodos.size()-1);
    int l = node_randomizer(mt);
    int val = created_nodos.at(l);
    created_nodos.erase(created_nodos.begin()+l);
    return val;
}

// pick a random id from the list of new ones without removing
int DSR_test::getID()
{
    std::lock_guard<std::mutex>  lock(mut);
    if(created_nodos.size()==0)
        return -1;
    auto node_randomizer = std::uniform_int_distribution(0, (int)created_nodos.size()-1);
    int l = node_randomizer(mt);
    int val = created_nodos.at(l);
    return val;
}

std::pair<int, int> DSR_test::removeEdgeIDs(){
    std::lock_guard<std::mutex>  lock(mut);
    if(created_edges.size()==0)
        return { -1, -1 };
    auto edge_randomizer = std::uniform_int_distribution(0, (int)created_edges.size()-1);
    int l = edge_randomizer(mt);
    auto val = created_edges.at(l);
    created_edges.erase(created_edges.begin()+l);
    return val;
}


std::pair<int, int> DSR_test::getEdgeIDs(){
    std::lock_guard<std::mutex>  lock(mut);
    if(created_edges.size()==0)
        return { -1, -1 };
    auto edge_randomizer = std::uniform_int_distribution(0, (int)created_edges.size()-1);
    int l = edge_randomizer(mt);
    auto val = created_edges.at(l);
    return val;
}


void DSR_test::addEdgeIDs(int from, int to){
    std::lock_guard<std::mutex>  lock(mut);
    created_edges.emplace_back(std::make_pair(from, to));
}