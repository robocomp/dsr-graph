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

float api_geom::distance_object_parent(std::shared_ptr<DSR::DSRGraph> G, std::shared_ptr<DSR::RT_API> rt_api, DSR::Node node1, DSR::Node node2)
{
    
    auto edge1 = rt_api->get_edge_RT(G->get_parent_node(node1).value(), node1.id());
    auto edge2 = rt_api->get_edge_RT(G->get_parent_node(node2).value(), node2.id());
    
    if (edge1.has_value() && edge2.has_value())
    {
        auto values_edge1 = G->get_attrib_by_name<rt_translation_att>(edge1.value());
        auto values_edge2 = G->get_attrib_by_name<rt_translation_att>(edge2.value());

        if( values_edge1.has_value() && values_edge2.has_value())
        {
            auto littleObjectHeight = G->get_attrib_by_name<obj_height_att>(node1); //node 1 lgitle
            auto bigObjectHeight = G->get_attrib_by_name<obj_height_att>(node2);  //node 2 big

            float x1 = values_edge1.value().get().at(0);
            float y1 = values_edge1.value().get().at(1);
            float z1 = values_edge1.value().get().at(2) - (littleObjectHeight.value()/2);
            
            float x2 = values_edge2.value().get().at(0);
            float y2 = values_edge2.value().get().at(1) ;
            float z2 = values_edge2.value().get().at(2) + (bigObjectHeight.value()/2);

            auto little_object_point = QVector3D(x1, y1 , z1);
            auto big_object_point = QVector3D(x2, y2 , z2);

            auto distance = little_object_point.distanceToPlane(big_object_point,big_object_point.normalized());

            auto bigwidth = G->get_attrib_by_name<obj_width_att>(node2);
            auto bigdepth = G->get_attrib_by_name<obj_depth_att>(node2);

            std::cout << x1 << " " << y1  << std::endl << x2 << " " << y2 << std::endl;
            std::cout << "Esquina arriba-izq: " << x2 - bigwidth.value()/2 << " " << y2 + bigdepth.value()/2 << std::endl;
            
            if(bigwidth.has_value() && bigdepth.has_value())
            {
                auto plane2d = QRect(x2 - bigwidth.value()/2, y2 + bigdepth.value()/2 , bigwidth.value(),bigdepth.value());
                std::cout << "bigwidth and bigdepth" << std::endl;

                if(plane2d.contains(y1,x1))
                {
                    //if( y1 - bigObjectHeight.value() < 3000)
                        std::cout << "Esta dentro " << std::endl;
                }
                // auto aux = QVector<QPoint>(),
                //                             QPoint(x1 + bigwidth.value()/2,y2,z2 - bigdepth.value()),
                //                             QPoint(x1 - bigwidth.value()/2,y2,z2 - bigdepth.value()),
                //                             QPoint(x1 - bigwidth.value()/2,y2,z2 + bigdepth.value()));
                
                //QPolygon(aux);
            } 
        }
    }

    return 0;

}