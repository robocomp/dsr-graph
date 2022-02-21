#include "api_geom.h"
#include <functional>

api_geom::api_geom(){};

float api_geom::distance_between_objects(std::shared_ptr<DSR::DSRGraph> G, std::unique_ptr<DSR::RT_API> rt_api, std::optional<DSR::Node> node1, std::optional<DSR::Node> node2)
{
    
    auto edge1 = rt_api->get_edge_RT(G->get_parent_node(node1.value()).value(), node1.value().id());
    auto edge2 = rt_api->get_edge_RT(G->get_parent_node(node2.value()).value(), node2.value().id());

    if (edge1.has_value() && edge2.has_value())
    {
        auto values_edge1 = G->get_attrib_by_name<rt_translation_att>(edge1.value());
        auto values_edge2 = G->get_attrib_by_name<rt_translation_att>(edge2.value());

        //Can be done in a line with if(;) but it does not complain the case they dont have value in the operation
        if( values_edge1.has_value() && values_edge2.has_value())
        {
            /*
            std::cout << values_edge2.value().get().at(0) << std::endl;
            std::cout << values_edge2.value().get().at(1) << std::endl;
            std::cout << values_edge2.value().get().at(2) << std::endl;
            */

            //More intuitive, it can be done without variables
            float x1 = values_edge1.value().get().at(0);
            float y1 = values_edge1.value().get().at(1);
            float z1 = values_edge1.value().get().at(2);

            float x2 = values_edge2.value().get().at(0);
            float y2 = values_edge2.value().get().at(1);
            float z2 = values_edge2.value().get().at(2);

            return sqrt(pow(abs(x1-x2),2) + pow(abs(y1-y2),2) + pow(abs(z1-z2),2));
        }
        else
        {
            std::cout << "Incorrect attributes values ";
            return 0;
        }
    }        
    else
        {
            std::cout << "Incorrect edges values ";
            return 0;
        }
}