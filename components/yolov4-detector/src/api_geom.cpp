#include "api_geom.h"

api_geom::api_geom(std::shared_ptr<DSR::DSRGraph> G, std::shared_ptr<DSR::RT_API> rt_api, std::shared_ptr<DSR::InnerEigenAPI> inner_eigen)
{
    this->G = G;
    this->rt_api = rt_api;
    this->inner_eigen = inner_eigen;
}

std::optional<float> api_geom::distance_between_objects(const DSR::Node &node1, const DSR::Node &node2)
{

    auto edge1 = rt_api->get_edge_RT(G->get_parent_node(node1).value(), node1.id());
    auto edge2 = rt_api->get_edge_RT(G->get_parent_node(node2).value(), node2.id());

    if (edge1.has_value() && edge2.has_value())
    {
        auto edge1translation = G->get_attrib_by_name<rt_translation_att>(edge1.value());
        auto edge2translation = G->get_attrib_by_name<rt_translation_att>(edge2.value());

        if (edge1translation.has_value() && edge2translation.has_value())
        {
            float x1 = edge1translation.value().get().at(0);
            float y1 = edge1translation.value().get().at(1);
            float z1 = edge1translation.value().get().at(2);

            float x2 = edge2translation.value().get().at(0);
            float y2 = edge2translation.value().get().at(1);
            float z2 = edge2translation.value().get().at(2);

            return sqrt(pow(abs(x1 - x2), 2) + pow(abs(y1 - y2), 2) + pow(abs(z1 - z2), 2));
        }
        else
            qWarning() << __FUNCTION__ << "Incorrect edges attributes values ";
    }
    else
        qWarning() << __FUNCTION__ << "Incorrect edges values";

    return {};
}

std::optional<DSR::Node> api_geom::get_closest_objects_by_type(const DSR::Node &node, const std::string &type)
{
    auto nodes_of_type = G->get_nodes_by_type(type);

    if (not nodes_of_type.empty())
    {
        auto it = std::ranges::min_element(nodes_of_type, [node, this](auto a, auto b)
                                           { return distance_between_objects(node, a) < distance_between_objects(node, b); });
        return *it;
    }
    else
        return {};
}

std::optional<float> api_geom::distance_of_over_object(const DSR::Node &little_object, const DSR::Node &big_object)
{
    auto little_object_edge = rt_api->get_edge_RT(G->get_parent_node(little_object).value(), little_object.id());
    auto big_object_edge = rt_api->get_edge_RT(G->get_parent_node(big_object).value(), big_object.id());

    if (little_object_edge.has_value() && big_object_edge.has_value())
    {
        auto little_object_translation = G->get_attrib_by_name<rt_translation_att>(little_object_edge.value());
        auto big_object_translation = G->get_attrib_by_name<rt_translation_att>(big_object_edge.value());

        if (little_object_translation.has_value() && big_object_translation.has_value())
        {
            auto big_object_width = G->get_attrib_by_name<obj_width_att>(big_object);
            auto big_object_depth = G->get_attrib_by_name<obj_depth_att>(big_object);
            auto big_object_height = G->get_attrib_by_name<obj_height_att>(big_object);

            if (big_object_width.has_value() && big_object_depth.has_value() && big_object_height.has_value())
            {
                // Transform the big object' corners in its reference system to the world reference system
                auto world_top_left_point = inner_eigen->transform(world_name, Eigen::Vector3d(-(big_object_width.value() / 2), big_object_depth.value() / 2, big_object_height.value()), big_object.name());
                auto world_top_right_point = inner_eigen->transform(world_name, Eigen::Vector3d(big_object_width.value() / 2, big_object_depth.value() / 2, big_object_height.value()), big_object.name());
                auto world_bottom_left_point = inner_eigen->transform(world_name, Eigen::Vector3d(-(big_object_width.value() / 2), -(big_object_depth.value() / 2), big_object_height.value()), big_object.name());
                auto world_bottom_right_point = inner_eigen->transform(world_name, Eigen::Vector3d(big_object_width.value() / 2, -(big_object_depth.value() / 2), big_object_height.value()), big_object.name());

                if (world_top_left_point.has_value() and world_top_right_point.has_value() && world_bottom_left_point.has_value() && world_bottom_right_point.has_value())
                {
                    // Create a polygon with the world reference system corner points
                    std::initializer_list world_points_list = {QPointF(world_top_left_point.value().x(), world_top_left_point.value().y()),
                                                               QPointF(world_top_right_point.value().x(), world_top_right_point.value().y()),
                                                               QPointF(world_bottom_right_point.value().x(), world_bottom_right_point.value().y()),
                                                               QPointF(world_bottom_left_point.value().x(), world_bottom_left_point.value().y())};

                    auto world_big_object_polygon = QPolygonF(QVector(world_points_list));

                    if (auto little_object_height = G->get_attrib_by_name<obj_height_att>(little_object); little_object_height.has_value())
                    {

                        if (world_big_object_polygon.containsPoint(QPointF(little_object_translation.value().get().at(0), little_object_translation.value().get().at(1)), Qt::OddEvenFill))
                            return (little_object_translation.value().get().at(2) - (little_object_height.value() / 2) - (big_object_translation.value().get().at(2) + (big_object_height.value() / 2)));
                        else
                            qWarning() << __FUNCTION__ << "Big object not contain little object";
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

std::optional<float> api_geom::height_difference(const DSR::Node &node1, const DSR::Node &node2)
{
    auto node1_edge = rt_api->get_edge_RT(G->get_parent_node(node1).value(), node1.id());
    auto node2_edge = rt_api->get_edge_RT(G->get_parent_node(node2).value(), node2.id());

    auto node1_height = G->get_attrib_by_name<obj_height_att>(node1);
    auto node2_height = G->get_attrib_by_name<obj_height_att>(node2);

    if (node1_height.has_value() and node2_height.has_value())
    {
        if (node1_edge.has_value() and node2_edge.has_value())
        {
            auto node1_translation = G->get_attrib_by_name<rt_translation_att>(node1_edge.value());
            auto node2_translation = G->get_attrib_by_name<rt_translation_att>(node2_edge.value());

            if (node1_translation.value().get().at(2) > node2_translation.value().get().at(2))
            {
                std::cout << "Else" << std::endl;
                return (node1_translation.value().get().at(2) - node1_height.value() / 2) - (node2_translation.value().get().at(2) + node2_height.value() / 2);
            }
            else
            {
                std::cout << "Else" << std::endl;
                return (node1_translation.value().get().at(2) + node1_height.value() / 2) - (node2_translation.value().get().at(2) - node2_height.value() / 2);
            }
        }
        else
            qWarning() << __FUNCTION__ << "Incorrect edges values of node1 or node2";
    }
    else
        qWarning() << __FUNCTION__ << "Incorrect height values of node1 or node2";

    return {};
}

bool api_geom::strictly_over_object(DSR::Node &little_object, DSR::Node &big_object)
{
    return distance_of_over_object(little_object, big_object) < 500;
}

bool api_geom::insert_node(DSR::Node &node, DSR::Node &parent)
{
    // G->add_or_modify_attrib_local<parent_att>(object_node, parent_node.value().id());
    // G->add_or_modify_attrib_local<level_att>(object_node, G->get_node_level(parent_node.value()).value() + 1);
    // G->add_or_modify_attrib_local<obj_width_att>(object_node, size[0]);
    // G->add_or_modify_attrib_local<obj_height_att>(object_node, size[1]);
    // G->add_or_modify_attrib_local<obj_depth_att>(object_node, size[2]);
    // const auto &[random_x, random_y] = get_random_position_to_draw_in_graph("object");
    // G->add_or_modify_attrib_local<pos_x_att>(object_node, random_x);
    // G->add_or_modify_attrib_local<pos_y_att>(object_node, random_y);
    // G->get_parent_node(node).value().id()
    // G->get_node(world_name).value() --- (float)parent_rt.value().y()

    if (G->get_parent_node(node).value().id() == 1)
    {
        std::cout << "IF" << std::endl;
        auto world = G->get_node(world_name).value();

        if (auto node_edge = rt_api->get_edge_RT(world, node.id()); node_edge.has_value())
        {
            auto node_translation = G->get_attrib_by_name<rt_translation_att>(node_edge.value());

            qInfo() << "world cup position" << node_translation.value().get();

            if (auto parent_rt = inner_eigen->transform(parent.name(),
                            Eigen::Vector3d(node_translation.value().get().at(0), node_translation.value().get().at(1), node_translation.value().get().at(2)), world_name);
                parent_rt.has_value())
            {
                auto transformwithoutpoints = inner_eigen->transform(parent.name(),node.name());

                std::cout << "tranformwithoutpoints" << transformwithoutpoints.value() << std::endl;
                std::cout << "table cup position " << parent_rt.value() << std::endl;

                float x = transformwithoutpoints.value().x();
                float y = transformwithoutpoints.value().y();
                float z = transformwithoutpoints.value().z();

                // Delete previous edge
                auto world = G->get_node(world_name).value();
                G->delete_edge(world.id(), node.id(), "RT");
                G->update_node(world);

                // Update parent node
                G->add_or_modify_attrib_local<parent_att>(node, parent.id());
                //Hardcode level value
                G->add_or_modify_attrib_local<level_att>(node, G->get_node_level(parent).value() + 1);
                G->update_node(node);
                G->update_node(parent);
                G->update_node(world);

                // Create new edge
                DSR::Edge edge = DSR::Edge::create<RT_edge_type>(parent.id(), node.id());
                
                // std::cout << "Primer transform" << parent_rt.value() << std::endl;

                rt_api->insert_or_assign_edge_RT(parent,node.id(),std::vector<float>{x, y, z},std::vector<float>{0., 0., 0.});
                G->update_node(parent);
                // // Cam problem
                // G->add_or_modify_attrib_local<rt_translation_att>(edge, std::vector<float>{x, y, z});

                // G->add_or_modify_attrib_local<rt_rotation_euler_xyz_att>(edge, std::vector<float>{0., 0., 0.});

                // Get and set rotation matrix
                //  auto RT = inner_eigen->get_rotation_matrix(parent.name(),node.name());
                //  std::cout << "ROTATION MATRIX" << RT.value() << std::endl;
                // std::cout << "RTVALUES" << RT.value().x() << (float)RT.value().y() << (float)RT.value().z() << std::endl;

                // Insert edges and updates
                // G->insert_or_assign_edge(edge);
                // G->update_node(node);
                // G->update_node(parent);
                // G->update_node(world);
            }

            // auto world_node_translation = G->get_attrib_by_name<rt_translation_att>(world_edge_node.value());

            // auto big_object_edge_node = inner_eigen->transform(parent.name(), Eigen::Vector3d
            //                             (world_node_translation.value().get().at(0),world_node_translation.value().get().at(1),world_node_translation.value().get().at(2)), world_name);

            // auto RT = inner_eigen->get_rotation_matrix(parent.name(), node.name());

            // std::cout << "Translation:" << big_object_edge_node.value() << std::endl;

            // std::cout << "Rotation Matrix:" << RT.value() << std::endl;

            // std::cout << "Borramos world-cup y updateamos world" << std::endl;

            // G->add_or_modify_attrib_local<parent_att>(node, parent.id());
            // G->update_node(node);

            // G->add_or_modify_attrib_local<rt_translation_att>(edge, std::vector<float>
            //                             {(float)big_object_edge_node.value().x(), (float)big_object_edge_node.value().y(), (float)big_object_edge_node.value().z()});

            // G->add_or_modify_attrib_local<rt_rotation_euler_xyz_att>(edge, std::vector<float>{0., 0., 0.});
            // G->insert_or_assign_edge(edge);
        }
    }
    return true;
}
