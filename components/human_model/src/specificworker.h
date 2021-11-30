/*
 *    Copyright (C) 2021 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"
#include <doublebuffer/DoubleBuffer.h>
#include  "../../../etc/graph_names.h"


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

public slots:
	void compute();
	int startup_check();
	void initialize(int period);

private:
	// DSR graph
	std::shared_ptr<DSR::DSRGraph> G;
    std::shared_ptr<DSR::InnerEigenAPI> inner_eigen;
    std::unique_ptr<DSR::AgentInfoAPI> agent_info_api;


    //DSR params
	std::string agent_name;
	int agent_id;

	bool tree_view;
	bool graph_view;
	bool qscene_2d_view;
	bool osg_3d_view;

	// DSR graph viewer
	std::unique_ptr<DSR::DSRViewer> graph_viewer;
	QHBoxLayout mainLayout;
	void modify_node_slot(std::uint64_t, const std::string &type){};
	void modify_attrs_slot(std::uint64_t id, const std::vector<std::string>& att_names){};
	void modify_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type){};

	void del_edge_slot(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){};
	void del_node_slot(std::uint64_t from){};     
	bool startup_check_flag;

    void create_person_model();

    void delete_person_model();

    tuple<float, float> get_random_position_to_draw_in_graph(const string &type);

    template <typename T>
    void add_node(const std::string &node_name, const std::string &parent_node_name)
    {
        DSR::Node new_node = DSR::Node::create<T>(node_name);
        auto parent_node = G->get_node(parent_node_name);
        G->add_or_modify_attrib_local<parent_att>(new_node, parent_node.value().id());
        G->add_or_modify_attrib_local<level_att>(new_node, G->get_node_level(parent_node.value()).value() + 1);
        // draw in graph
        std::tuple<float, float> graph_pos;
        if(node_name.find("person") != string::npos)
            graph_pos = get_random_position_to_draw_in_graph("");
        else
            graph_pos = get_position_by_level_in_graph(parent_node.value());
        const auto &[random_x, random_y] = graph_pos;
        G->add_or_modify_attrib_local<pos_x_att>(new_node, random_x);
        G->add_or_modify_attrib_local<pos_y_att>(new_node, random_y);
        G->update_node(new_node);
        if (std::optional<int> id = G->insert_node(new_node); id.has_value())
        {
            DSR::Edge edge = DSR::Edge::create<RT_edge_type>(parent_node.value().id(), new_node.id());
            G->add_or_modify_attrib_local<rt_translation_att>(edge, std::vector<float>{0.f, 0.f, 0.f});
            G->add_or_modify_attrib_local<rt_rotation_euler_xyz_att>(edge, std::vector<float>{0., 0., 0.});
            G->insert_or_assign_edge(edge);
            G->update_node(parent_node.value());
        }
        else
            qWarning() << "Node " << QString::fromStdString(node_name) << " could NOT be created";
    }

    tuple<float, float> get_position_by_level_in_graph(const DSR::Node &parent);
};

#endif
