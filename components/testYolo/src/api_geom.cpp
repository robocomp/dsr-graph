#include "api_geom.h"

float api_geom::distance_between_objects(std::shared_ptr<DSR::DSRGraph> G, std::shared_ptr<DSR::RT_API> rt_api, DSR::Node node1, DSR::Node node2)
{
    
    auto edge1 = rt_api->get_edge_RT(G->get_parent_node(node1).value(), node1.id());
    auto edge2 = rt_api->get_edge_RT(G->get_parent_node(node2).value(), node2.id());

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
            std::cout << ;
            qWarning() << __FUNCTION__ << "Incorrect attributes values "; //TODO:Pner nombre nodo
            return 0;
        }
    }        
    else
        {
            std::cout << "Incorrect edges values ";
            return 0;
        }
}

float api_geom::distance_object_parent(std::shared_ptr<DSR::DSRGraph> G, std::unique_ptr<DSR::RT_API> rt_api, DSR::Node node1, DSR::Node node2)
{
    
    auto edge1 = rt_api->get_edge_RT(G->get_parent_node(node1).value(), node1.id());
    auto edge2 = rt_api->get_edge_RT(G->get_parent_node(node2).value(), node2.id());
    
    if (edge1.has_value() && edge2.has_value())
    {
        auto values_edge1 = G->get_attrib_by_name<rt_translation_att>(edge1.value());
        auto values_edge2 = G->get_attrib_by_name<rt_translation_att>(edge2.value());

        if( values_edge1.has_value() && values_edge2.has_value())
        {
            auto littleObjectHeight = G->get_attrib_by_name<obj_height_att>(edge1.value());
            auto bigObjectHeight = G->get_attrib_by_name<obj_height_att>(edge2.value());

            float x1 = values_edge1.value().get().at(0);
            float y1 = values_edge1.value().get().at(1) - (littleObjectHeight.value()/2);
            float z1 = values_edge1.value().get().at(2);
            
            float x2 = values_edge2.value().get().at(0);
            float y2 = values_edge2.value().get().at(1) + (bigObjectHeight.value()/2);
            float z2 = values_edge2.value().get().at(2);

            auto little_object_point = QVector3D(x1, y1 , z1);
            auto big_object_point = QVector3D(x2, y2 , z2);

            little_object_point.distanceToPlane(big_object_point,big_object_point.normalized());
        }
    }

    return 0;

}
