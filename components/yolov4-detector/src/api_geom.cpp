#include "api_geom.h"

using namespace DSR;

Api_Geom::Api_Geom(std::shared_ptr<DSR::DSRGraph> G)
{
    this->G = G;
    this->rt_api = G->get_rt_api();
    this->inner_eigen = G->get_inner_eigen_api();
}

std::optional<float> Api_Geom::distance_between_objects(const DSR::Node &node1, const DSR::Node &node2)
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

std::optional<DSR::Node> Api_Geom::get_closest_objects_by_type(const DSR::Node &node, const std::string &type)
{
    auto nodes_of_type = G->get_nodes_by_type(type);
    std::cout << nodes_of_type.size() << std::endl;
    if (not nodes_of_type.empty())
    {
        auto it = std::ranges::min_element(nodes_of_type, [node, this](auto a, auto b)
                                           { return distance_between_objects(node, a) < distance_between_objects(node, b); });
        return *it;
    }
    else
        return {};
}

std::optional<float> Api_Geom::distance_to_object_below(const DSR::Node &this_object, const DSR::Node &below_object)
{
    auto this_object_edge = rt_api->get_edge_RT(G->get_parent_node(this_object).value(), this_object.id());
    auto below_object_edge = rt_api->get_edge_RT(G->get_parent_node(below_object).value(), below_object.id());

    if (this_object_edge.has_value() && below_object_edge.has_value())
    {
        auto this_object_translation = inner_eigen->transform(world_name, this_object.name());
        auto below_object_translation = G->get_attrib_by_name<rt_translation_att>(below_object_edge.value());

        if (this_object_translation.has_value() && below_object_translation.has_value())
        {
            auto below_object_width = G->get_attrib_by_name<obj_width_att>(below_object);
            auto below_object_depth = G->get_attrib_by_name<obj_depth_att>(below_object);
            auto below_object_height = G->get_attrib_by_name<obj_height_att>(below_object);

            if (below_object_width.has_value() && below_object_depth.has_value() && below_object_height.has_value())
            {
                // Transform the big object' corners in its reference system to the world reference system
                auto world_top_left_point = inner_eigen->transform(world_name, Eigen::Vector3d(-(below_object_width.value() / 2), below_object_depth.value() / 2, below_object_height.value()), below_object.name());
                auto world_top_right_point = inner_eigen->transform(world_name, Eigen::Vector3d(below_object_width.value() / 2, below_object_depth.value() / 2, below_object_height.value()), below_object.name());
                auto world_bottom_left_point = inner_eigen->transform(world_name, Eigen::Vector3d(-(below_object_width.value() / 2), -(below_object_depth.value() / 2), below_object_height.value()), below_object.name());
                auto world_bottom_right_point = inner_eigen->transform(world_name, Eigen::Vector3d(below_object_width.value() / 2, -(below_object_depth.value() / 2), below_object_height.value()), below_object.name());

                if (world_top_left_point.has_value() and world_top_right_point.has_value() && world_bottom_left_point.has_value() && world_bottom_right_point.has_value())
                {
                    // Create a polygon with the world reference system corner points
                    std::initializer_list world_points_list = {QPointF(world_top_left_point.value().x(), world_top_left_point.value().y()),
                                                               QPointF(world_top_right_point.value().x(), world_top_right_point.value().y()),
                                                               QPointF(world_bottom_right_point.value().x(), world_bottom_right_point.value().y()),
                                                               QPointF(world_bottom_left_point.value().x(), world_bottom_left_point.value().y())};

                    auto world_below_object_polygon = QPolygonF(QVector(world_points_list));

                    if (auto this_object_height = G->get_attrib_by_name<obj_height_att>(this_object); this_object_height.has_value())
                    {
                        std::cout << "Taza:" << this_object_translation.value() << std::endl;

                        if (world_below_object_polygon.containsPoint(QPointF(this_object_translation.value().x(), this_object_translation.value().y()), Qt::OddEvenFill))
                        {
                            if (auto distance = height_difference(this_object, below_object); distance.has_value())
                                return distance.value() - (this_object_height.value() / 2) - (below_object_height.value() / 2);
                        }
                        else
                        {
                            std::cout << below_object.name() << " not contain little object" << std::endl;
                            // qWarning() << __FUNCTION__ << "Big object not contain little object";
                        }
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

std::optional<float> Api_Geom::height_difference(const DSR::Node &node1, const DSR::Node &node2)
{
    auto world_node = G->get_node(world_name);
    // auto node1_edge = rt_api->get_edge_RT(G->get_parent_node(node1).value(), node1.id());
    // auto node2_edge = rt_api->get_edge_RT(G->get_parent_node(node2).value(), node2.id());
    if (world_node.has_value())
    {
        // auto node1_edge = rt_api->get_edge_RT(world_node.value(), node1.id());
        auto node1_edge = inner_eigen->transform(world_name, node1.name());
        auto node2_edge = rt_api->get_edge_RT(G->get_parent_node(node2).value(), node2.id());

        auto node1_height = G->get_attrib_by_name<obj_height_att>(node1);
        auto node2_height = G->get_attrib_by_name<obj_height_att>(node2);

        if (node1_height.has_value() and node2_height.has_value())
        {
            if (node1_edge.has_value() and node2_edge.has_value())
            {
                // auto node1_translation = G->get_attrib_by_name<rt_translation_att>(node1_edge.value());
                auto node2_translation = G->get_attrib_by_name<rt_translation_att>(node2_edge.value());
                return node1_edge.value().z() - node2_translation.value().get().at(2);
            }
            else
                qWarning() << __FUNCTION__ << "Incorrect edges values of node1 or node2";
        }
        else
            qWarning() << __FUNCTION__ << "Incorrect height values of node1 or node2";
    }
    else
        qWarning() << __FUNCTION__ << "Incorrect world node value";
    return {};
}

// IN PROGRESS
bool Api_Geom::strictly_over_object(DSR::Node &little_object, DSR::Node &big_object)
{
    return distance_to_object_below(little_object, big_object) < 500;
}

bool Api_Geom::update_node(DSR::Node &node, DSR::Node &new_parent)
{
    if (auto current_parent = G->get_parent_node(node); current_parent.has_value()) // Obtain the current parent
    {
        if (current_parent.value().id() != new_parent.id())
        {
            if (auto node_edge = rt_api->get_edge_RT(current_parent.value(), node.id()); node_edge.has_value()) // Obtain current edge
            {
                if (auto translation_from_parent = inner_eigen->transform(new_parent.name(), node.name()); translation_from_parent.has_value()) // Obtain the translation from parent
                {
                    // Change for casting in insert or assign edge RT
                    float x = translation_from_parent.value().x();
                    float y = translation_from_parent.value().y();
                    float z = translation_from_parent.value().z();

                    std::cout << "PARENT NAME PREVIOUS TO ACTUALIZATION" << current_parent.value().name() << std::endl;

                    // Delete previous edge
                    G->delete_edge(current_parent.value().id(), node.id(), "RT");
                    G->update_node(current_parent.value());

                    // Update node attributes
                    G->add_or_modify_attrib_local<parent_att>(node, new_parent.id());
                    G->add_or_modify_attrib_local<level_att>(node, G->get_node_level(new_parent).value() + 1);
                    G->update_node(new_parent);

                    // Create new edge
                    rt_api->insert_or_assign_edge_RT(new_parent, node.id(), std::vector<float>{x, y, z}, std::vector<float>{0., 0., 0.});
                    return true;
                }
                else
                    qWarning() << __FUNCTION__ << "Transform result has not value";
            }
            else
                qWarning() << __FUNCTION__ << "Edge between current parent and node does not exist";
        }
    }
    else
        qWarning() << __FUNCTION__ << "Current parent of the node does not exist";

    return false;
}

bool Api_Geom::insert_node_in_container(DSR::Node &node)
{
    auto containers = G->get_nodes_by_type("container");
    if (not containers.empty())
        for (auto &&container : containers)
            if (auto distance = distance_to_object_below(node, container); distance.has_value())
            {
                std::cout << "Contenedor: " << container.name() << " Node:" << node.name() << " Distance:" << distance.value() << std::endl;
                if (0 < distance.value() and distance.value() < 200)
                {
                    if (not this->update_node(node, container))
                        qWarning() << __FUNCTION__ << "Does not insert properly";
                    else
                        return true;
                }
            } // change to over or above

    return false;
}