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
#include  "../../../etc/viriato_graph_names.h"
#include <fps/fps.h>


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
	void add_or_assign_node_slot(std::uint64_t, const std::string &type);
	void add_or_assign_attrs_slot(std::uint64_t id, const std::map<std::string, DSR::Attribute> &attribs){};
	void add_or_assign_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type){};
	void del_edge_slot(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){};
	void del_node_slot(std::uint64_t from){};     
	bool startup_check_flag;

    //local widget
    DSR::QScene2dViewer *widget_2d;

	// Laser
    using LaserData = std::tuple<std::vector<float>, std::vector<float>>;  //<angles, dists>
    DoubleBuffer<LaserData, QPolygonF> laser_buffer;
    vector<QPointF> get_points_along_extended_robot_polygon(int offset, int chunck);

    // Robot
    struct CONSTANTS
    {
        float robot_length = 500;
        float robot_radius = robot_length / 2.0;
        float max_adv_speed = 400;
        float max_rot_speed = 1;
        float max_side_speed = 400;
        float robot_width = 470;
        float offset = 250;
    };
    CONSTANTS consts;
    QRectF robot_rect, robot_rect_extended;
    QLineF line_top, line_bottom, line_left, line_right;
    std::vector<QPointF> robot_extended_points;
    std::tuple<float, float, float> send_command_to_robot(const std::tuple<float, float, float> &speeds);

	// FPS
    FPSCounter fps;

    // draw
    void draw_bumper(const std::vector<QPointF> &contacts, const QVector2D &total);
};

#endif
