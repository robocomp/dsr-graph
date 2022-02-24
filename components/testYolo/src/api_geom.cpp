#include "api_geom.h"


api_geom::api_geom(std::shared_ptr<DSR::DSRGraph> G, std::shared_ptr<DSR::RT_API> rt_api, std::shared_ptr<DSR::InnerEigenAPI> inner_eigen)
{
    this->G = G;
    this->rt_api = rt_api;
    this->inner_eigen = inner_eigen;
}

float api_geom::distance_between_objects(DSR::Node node1, DSR::Node node2)
{
    
    auto edge1 = rt_api->get_edge_RT(G->get_parent_node(node1).value(), node1.id());
    auto edge2 = rt_api->get_edge_RT(G->get_parent_node(node2).value(), node2.id());

    if (edge1.has_value() && edge2.has_value())
    {
        auto edge1translation = G->get_attrib_by_name<rt_translation_att>(edge1.value());
        auto edge2translation = G->get_attrib_by_name<rt_translation_att>(edge2.value());

        if( edge1translation.has_value() && edge2translation.has_value())
        {
            float x1 = edge1translation.value().get().at(0);
            float y1 = edge1translation.value().get().at(1);
            float z1 = edge1translation.value().get().at(2);

            float x2 = edge2translation.value().get().at(0);
            float y2 = edge2translation.value().get().at(1);
            float z2 = edge2translation.value().get().at(2);

            return sqrt(pow(abs(x1-x2),2) + pow(abs(y1-y2),2) + pow(abs(z1-z2),2));
        }
        else
        {
            qWarning() << __FUNCTION__ << "Incorrect edges attributes values ";
            return 0;
        }
    }        
    else
    {
        qWarning() << __FUNCTION__ << "Incorrect edges values ";
        return 0;
    }
}

float api_geom::distance_object_parent(DSR::Node little_object, DSR::Node big_object)
{
    
    auto little_object_edge = rt_api->get_edge_RT(G->get_parent_node(little_object).value(), little_object.id());
    auto big_object_edge = rt_api->get_edge_RT(G->get_parent_node(big_object).value(), big_object.id());
    
    if (little_object_edge.has_value() && big_object_edge.has_value())
    {
        auto little_object_translation = G->get_attrib_by_name<rt_translation_att>(little_object_edge.value());
        auto big_object_translation = G->get_attrib_by_name<rt_translation_att>(big_object_edge.value());

        if( little_object_translation.has_value() && big_object_translation.has_value())
        {
            auto little_object_height = G->get_attrib_by_name<obj_height_att>(little_object);

            auto big_object_width = G->get_attrib_by_name<obj_width_att>(big_object);
            auto big_object_depth = G->get_attrib_by_name<obj_depth_att>(big_object);
            auto big_object_height = G->get_attrib_by_name<obj_height_att>(big_object);

            // std::cout << "BIG OBJECT DIMENSIONS VALUES" << std::endl;
            // std::cout << big_object_width.value() << std::endl;
            // std::cout << big_object_depth.value() << std::endl;
            // std::cout << big_object_height.value() << std::endl;

            //Calculate big_object_corners
            auto top_left_point = Eigen::Vector3d(-(big_object_width.value()/2),big_object_depth.value()/2,big_object_height.value());
            auto top_right_point = Eigen::Vector3d(big_object_width.value()/2,big_object_depth.value()/2,big_object_height.value());
            auto bottom_left_point = Eigen::Vector3d(-(big_object_width.value()/2),-(big_object_depth.value()/2),big_object_height.value());
            auto bottom_right_point = Eigen::Vector3d(big_object_width.value()/2,-(big_object_depth.value()/2),big_object_height.value());

            // std::cout << "VECTOR3D TABLE VALUES" << std::endl;
            // std::cout << top_left_point << std::endl;
            // std::cout << top_right_point << std::endl;
            // std::cout << bottom_left_point << std::endl;
            // std::cout << bottom_left_point << std::endl;

            auto world_top_left_point = inner_eigen->transform(world_name,top_left_point,big_object.name());
            auto world_top_right_point = inner_eigen->transform(world_name,top_right_point,big_object.name());
            auto world_bottom_left_point = inner_eigen->transform(world_name,bottom_left_point,big_object.name());
            auto world_bottom_right_point = inner_eigen->transform(world_name,bottom_right_point,big_object.name());

            // std::cout << "VECTOR3D WORLD VALUES" << std::endl;
            // std::cout << (int)world_top_left_point.value().x()<< std::endl;
            // std::cout << (int)world_top_right_point.value() << std::endl;
            // std::cout << (int)world_bottom_left_point.value() << std::endl;
            // std::cout << (int)world_bottom_right_point.value() << std::endl;
            
            auto world_points_list = std::initializer_list{ QPoint(world_top_left_point.value().x(), world_top_left_point.value().y()),
                                                            QPoint(world_top_right_point.value().x(), world_top_right_point.value().y()),
                                                            QPoint(world_bottom_right_point.value().x(), world_bottom_right_point.value().y()),
                                                            QPoint(world_bottom_left_point.value().x(), world_bottom_left_point.value().y()) };

            auto world_big_object_polygon = QPolygon(QVector(world_points_list));

            // std::cout << world_big_object_polygon << std::endl;
            
            //world_big_object_polygon.contains(QPoint(little_object_translation.value().get().at(0), little_object_translation.value().get().at(1)));
            
            //Checks if big object contains in its plane the little object
            if(world_big_object_polygon.containsPoint(QPoint(2625,803),Qt::OddEvenFill))
            {
                std::cout << "Contiene el punto" << std::endl;
            }                                    
            else
            {
                std::cout << "No contiene el punto" << std::endl;
            }
                // float x1 = values_edge1.value().get().at(0);
                // float y1 = values_edge1.value().get().at(1) - (littleObjectHeight.value()/2);
                // float z1 = values_edge1.value().get().at(2);
                
                // float x2 = values_edge2.value().get().at(0);
                // float y2 = values_edge2.value().get().at(1) + (bigObjectHeight.value()/2);
                // float z2 = values_edge2.value().get().at(2);

                // auto little_object_point = QVector3D(x1, y1 , z1);
                // auto big_object_point = QVector3D(x2, y2 , z2);

                // little_object_point.distanceToPlane(big_object_point,big_object_point.normalized());
            
        }
    }

    return 0;

}
