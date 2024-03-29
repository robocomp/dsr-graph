#include "api_geom.h"


api_geom::api_geom(std::shared_ptr<DSR::DSRGraph> G, std::shared_ptr<DSR::RT_API> rt_api, std::shared_ptr<DSR::InnerEigenAPI> inner_eigen)
{
    this->G = G;
    this->rt_api = rt_api;
    this->inner_eigen = inner_eigen;
}

std::optional<float> api_geom::distance_between_objects(DSR::Node &node1, DSR::Node &node2)
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
            qWarning() << __FUNCTION__ << "Incorrect edges attributes values ";
    }        
    else
        qWarning() << __FUNCTION__ << "Incorrect edges values";
    
    return {};
}

// std::optional<DSR::Node> api_geom::get_closest_objects_by_type(const DSR::Node &node,const std::string &type)
// {
//     auto nodes_of_type = G->get_nodes_by_type(type);

//     if(not nodes_of_type.empty())
//     {
//         auto it = std::ranges::min_element(nodes_of_type,[node,G](auto a,auto b)
//                                 { return G->distance_between_objects(node,a) < G->distance_between_objects(node,b) });
//         return *it;
//     }
//     else                                   
//         return {};
// }

std::optional<float> api_geom::distance_of_over_object(DSR::Node &little_object, DSR::Node &big_object)
{
    auto little_object_edge = rt_api->get_edge_RT(G->get_parent_node(little_object).value(), little_object.id());
    auto big_object_edge = rt_api->get_edge_RT(G->get_parent_node(big_object).value(), big_object.id());
    
    if (little_object_edge.has_value() && big_object_edge.has_value())
    {
        auto little_object_translation = G->get_attrib_by_name<rt_translation_att>(little_object_edge.value());
        auto big_object_translation = G->get_attrib_by_name<rt_translation_att>(big_object_edge.value());

        if(little_object_translation.has_value() && big_object_translation.has_value())
        {
            auto big_object_width = G->get_attrib_by_name<obj_width_att>(big_object);
            auto big_object_depth = G->get_attrib_by_name<obj_depth_att>(big_object);
            auto big_object_height = G->get_attrib_by_name<obj_height_att>(big_object);

            if(big_object_width.has_value() && big_object_depth.has_value() && big_object_height.has_value())
            {
                //Transform the big object' corners in its reference system to the world reference system
                auto world_top_left_point = inner_eigen->transform(world_name,Eigen::Vector3d(-(big_object_width.value()/2),big_object_depth.value()/2,big_object_height.value()),big_object.name());
                auto world_top_right_point = inner_eigen->transform(world_name,Eigen::Vector3d(big_object_width.value()/2,big_object_depth.value()/2,big_object_height.value()),big_object.name());
                auto world_bottom_left_point = inner_eigen->transform(world_name,Eigen::Vector3d(-(big_object_width.value()/2),-(big_object_depth.value()/2),big_object_height.value()),big_object.name());
                auto world_bottom_right_point = inner_eigen->transform(world_name,Eigen::Vector3d(big_object_width.value()/2,-(big_object_depth.value()/2),big_object_height.value()),big_object.name());

                if(world_top_left_point.has_value() and world_top_right_point.has_value() && world_bottom_left_point.has_value() && world_bottom_right_point.has_value())
                {
                    //Create a polygon with the world reference system corner points
                    std::initializer_list world_points_list = { QPointF(world_top_left_point.value().x(), world_top_left_point.value().y()),
                                                                    QPointF(world_top_right_point.value().x(), world_top_right_point.value().y()),
                                                                    QPointF(world_bottom_right_point.value().x(), world_bottom_right_point.value().y()),
                                                                    QPointF(world_bottom_left_point.value().x(), world_bottom_left_point.value().y()) };

                    auto world_big_object_polygon = QPolygonF(QVector(world_points_list));

                    if(auto little_object_height = G->get_attrib_by_name<obj_height_att>(little_object); little_object_height.has_value())
                    {
                        auto little_object_bottom_middle_point = QVector3D( little_object_translation.value().get().at(0),
                                                                            little_object_translation.value().get().at(1) - (little_object_height.value()/2),
                                                                            little_object_translation.value().get().at(1));
                        
                        auto big_object_top_middle_point = QVector3D(   big_object_translation.value().get().at(0),
                                                                        big_object_translation.value().get().at(1) + (big_object_height.value()/2),
                                                                        big_object_translation.value().get().at(2));

                        little_object_bottom_middle_point.distanceToPlane(big_object_top_middle_point,big_object_top_middle_point.normalized());
                        
                        //Checks if big object contains in its plane the little object CHANGE QPointF VALUE FOR LITTLE OBJECT VALUES
                        if(world_big_object_polygon.containsPoint(QPointF(little_object_translation.value().get().at(0),little_object_translation.value().get().at(1)),Qt::OddEvenFill))
                            return little_object_bottom_middle_point.distanceToPlane(big_object_top_middle_point,big_object_top_middle_point.normalized());
                        else
                            return {};
                    }
                    else
                        qWarning() << __FUNCTION__ << "Incorrect little object height value";
                }
                else
                    qWarning() << __FUNCTION__ << "Incorrect transform values from big object references system to world";
            }
            else
                qWarning() << __FUNCTION__ << "Incorrect dimensions values of big object";
        }
        else
            qWarning() << __FUNCTION__ << "Incorrect translation values of big or little object";
    }
    else
        qWarning() << __FUNCTION__ << "Incorrect edges values of big or little object";

    return {};
}

bool api_geom::strictly_over_object(DSR::Node &little_object, DSR::Node &big_object)
{
    return distance_of_over_object(little_object, big_object) < 500;      
}
