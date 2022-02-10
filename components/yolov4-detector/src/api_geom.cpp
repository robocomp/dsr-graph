//
// Created by alumno on 19/1/22.
//
#include "api_geom.h"

float api_geom::distancia(std::shared_ptr<DSR::DSRGraph> G, DSR::Node n1, DSR::Node n2)
{

    auto node = G->get_nodes_by_type("container");

    for(auto&& container : node)
        std::cout << container.name();
    
    std::cout << "Nº contenedores: " << node.size();

    float x1 = 1 ,y1 = 1,z1 = 0,x2 = 0,y2 = 0,z2 = 0;
    float distance = sqrt(pow(abs(x1-x2),2) + pow(abs(y1-y2),2) + pow(abs(z1-z2),2));
    return distance;
}