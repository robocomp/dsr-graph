/*
 *    Copyright (C) 2022 by YOUR NAME HERE
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
#include <custom_widget.h>
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"
#include <doublebuffer/DoubleBuffer.h>

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
	// Widget
	DSR::QScene2dViewer *widget_2d;
	Custom_widget custom_widget;

	// DSR graph
	std::shared_ptr<DSR::DSRGraph> G;
	std::shared_ptr<DSR::InnerEigenAPI> inner_eigen;
    std::shared_ptr<DSR::RT_API> rt_api;

	// DSR params
	std::string agent_name;
	int agent_id;

	bool tree_view;
	bool graph_view;
	bool qscene_2d_view;
	bool osg_3d_view;

	std::vector<std::string> words;
	bool rotating;
	float base_start_angle;

	// DSR graph viewer
	std::unique_ptr<DSR::DSRViewer> graph_viewer;
	QHBoxLayout mainLayout;
	void modify_node_slot(std::uint64_t, const std::string &type){};
	void modify_attrs_slot(std::uint64_t id, const std::vector<std::string> &att_names){};
	void modify_edge_slot(std::uint64_t from, std::uint64_t to, const std::string &type){};

	void del_edge_slot(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){};
	void del_node_slot(std::uint64_t from){};
	bool startup_check_flag;

	void set_focus(DSR::Node &node);
	bool delete_on_focus_edge();
	optional<DSR::Node> get_object(string object, string container, string size);
	std::vector<std::string> string_splitter(std::string s);
	void set_robot_rot_speed(float speed);

public slots:
	void queries();
};

#endif
