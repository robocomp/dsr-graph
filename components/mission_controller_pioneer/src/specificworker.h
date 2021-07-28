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
#include <custom_widget.h>
#include  "../../../etc/graph_names.h"
#include <opencv2/opencv.hpp>
#include "plan.h"

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
    void new_target_from_mouse(int pos_x, int pos_y, std::uint64_t id);
    void slot_start_mission();
    void slot_stop_mission();
    void slot_cancel_mission();

private:
	// DSR graph
	std::shared_ptr<DSR::DSRGraph> G;
    std::shared_ptr<DSR::CameraAPI> cam_api;
    std::shared_ptr<DSR::InnerEigenAPI> inner_eigen;
    std::shared_ptr<DSR::RT_API> rt_api;

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
	Eigen::IOFormat OctaveFormat, CommaInitFmt;

    // local widgets
    DSR::QScene2dViewer* widget_2d;
    Custom_widget custom_widget;

    // Laser
    using LaserData = std::tuple<std::vector<float>, std::vector<float>>;  //<angles, dists>
    DoubleBuffer<LaserData, std::tuple<std::vector<float>, std::vector<float>, QPolygonF, std::vector<QPointF>>> laser_buffer;

    // Robot and shape
    QPolygonF robot_polygon;
    DSR::Node get_robot_node();
    void send_command_to_robot(const std::tuple<float, float, float> &speeds);   //adv, side, rot

    // Camera
    //DoubleBuffer<std::vector<std::uint8_t>, cv::Mat> virtual_camera_buffer;
    DoubleBuffer<std::vector<std::uint8_t>, std::vector<std::uint8_t>> virtual_camera_buffer;
    void project_robot_on_image(const DSR::Node &robot_node, const QPolygonF &robot_polygon, cv::Mat virtual_frame, float focal);
    void project_laser_on_image(const DSR::Node &robot_node, const QPolygonF &laser_poly_local, cv::Mat virtual_frame, float focal);
    void project_path_on_image(const std::vector<Eigen::Vector3d> &path, const DSR::Node robot_node, cv::Mat virtual_frame, float focal);

    // Missions
    DoubleBuffer<Plan, Plan> plan_buffer;
    Plan current_plan;
    void insert_intention_node(const Plan &plan);

    //Path
    std::vector<Eigen::Vector3d> path;
    void draw_path(std::vector<Eigen::Vector3d> &path, QGraphicsScene* viewer_2d);


};

#endif
