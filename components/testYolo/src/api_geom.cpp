#include "api_geom.h"

api_geom::api_geom(){};

float api_geom::distancia(Node n1, node n2, DSR::DSRGraph G)
{
    float x1 = 1 ,y1 = 1,z1 = 0,x2 = 0,y2 = 0,z2 = 0;

    return sqrt(pow(abs(x1-x2),2) + pow(abs(y1-y2),2) + pow(abs(z1-z2),2));

}