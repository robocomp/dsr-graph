/*
 *    Copyright (C) 2020 by YOUR NAME HERE
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
#include <dsr/api/dsr_api.h>
#include <dsr/gui/dsr_gui.h>
#include <dsr/gui/viewers/qscene_2d_viewer/qscene_2d_viewer.h>
#include <localPerson.h>
#include <fps/fps.h>
#include <QGraphicsPolygonItem>
#include <doublebuffer/DoubleBuffer.h>
#include  "../../../etc/graph_names.h"

class Plan
{
    public:
        enum class Actions {GOTO};
        Actions action;
        std::string target_place;
        std::map<std::string, double> params;
        bool is_active = false;
        bool is_location(const Mat::Vector2d &loc)
        {
            return Mat::Vector2d(params.at("x"), params.at("y")) == loc;
        }
        void print()
        {
            std::cout << "------ Begin Plan ----------" << std::endl;
            std::cout << "\t Action: " << action_strings[action] << " Taget_place: " << target_place << std::endl;
            for(auto &&[k,v]: params)
                std::cout << "\t par1: " << k << " : " << std::to_string(v) << std::endl;
            std::cout << "------ End Plan ----------" << std::endl;
        };
    private:
        std::map<Actions, std::string> action_strings{{Actions::GOTO, "GOTO"}};
};

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
        void new_target_from_mouse(int pos_x, int pos_y, int id);

    private:
        // DSR graph
        std::shared_ptr<DSR::DSRGraph> G;
        std::shared_ptr<DSR::InnerEigenAPI> inner_eigen;
        std::shared_ptr<DSR::RT_API> rt_api;
        std::unique_ptr<DSR::AgentInfoAPI> agent_info_api;

        //DSR params
        std::string agent_name;
        std::string dsr_input_file;
        int agent_id;
        bool tree_view;
        bool graph_view;
        bool qscene_2d_view;
        bool osg_3d_view;
        std::shared_ptr<RoboCompCommonBehavior::ParameterList> conf_params;

        // DSR graph viewer
        std::unique_ptr<DSR::DSRViewer> dsr_viewer;
        QHBoxLayout mainLayout;
        void add_or_assign_node_slot(std::uint64_t, const std::string &type);
        void add_or_assign_attrs_slot(std::uint64_t id, const std::map<std::string, DSR::Attribute> &attribs){};
        void add_or_assign_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type){};
        void del_edge_slot(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){};
        void del_node_slot(std::uint64_t from);
        bool startup_check_flag;

        //local widget
        Custom_widget custom_widget;
        FPSCounter fps;

        //drawing
        DSR::QScene2dViewer *widget_2d;

        // path
       void draw_path(std::vector<Eigen::Vector2f> &path, QGraphicsScene* viewer_2d);

        //laser
        using LaserData = std::tuple<std::vector<float>, std::vector<float>>;  //<angles, dists>

        //Signal subscription
        DoubleBuffer<std::vector<Eigen::Vector2f>, std::vector<Eigen::Vector2f>> path_buffer;
        DoubleBuffer<LaserData, std::tuple<std::vector<float>, std::vector<float>, QPolygonF, std::vector<QPointF>>> laser_buffer;

        struct CONSTANTS
        {
            float robot_length = 500;
            float robot_width = 400;
            float robot_radius = robot_length / 2.0;
            float max_adv_speed = 2000;
            float max_rot_speed = 3;
            float max_side_speed = 400;
            float max_lag = 100;  // ms
            float lateral_correction_gain = 0.2;
            float lateral_correction_for_side_velocity = 500;
            float rotation_gain = 0.9;
            float times_final_distance_to_target_before_zero_rotation = 3;
            float advance_gaussian_cut_x = 0.7;
            float advance_gaussian_cut_y = 0.3;
            float final_distance_to_target = 200; // mm
        };
        CONSTANTS consts;

        // controller
        void path_follower_initialize();
        std::tuple<float, float, float>
        update(const std::vector<Eigen::Vector2f> &path, const QPolygonF &laser_poly, const Eigen::Vector2f &robot_pose,
               const Eigen::Vector2f &robot_nose, const Eigen::Vector2f &target);
        float robotXWidth, robotZLong; //robot dimensions read from config
        Eigen::Vector3d robotBottomLeft, robotBottomRight, robotTopRight, robotTopLeft;
        float exponentialFunction(float value, float xValue, float yValue, float min);
        float rewrapAngleRestricted(const float angle);
        std::vector<QPointF> get_points_along_extended_robot_polygon(int offset, int chunck);

    // target
        Eigen::Vector2f current_target;
        std::tuple<float, float, float> send_command_to_robot(const std::tuple<float, float, float> &speeds);  //adv, side, rot
        bool robot_is_active = false;

        // remove trailing path
        void remove_trailing_path(const std::vector<Eigen::Vector2f> &path, const Eigen::Vector2f &robot_pose );
        std::atomic_bool is_cyclic = ATOMIC_VAR_INIT(false);
        float dist_along_path(const std::vector<Eigen::Vector2f> &path);
        void print_current_state(const std::vector<Eigen::Vector2f> &path, Eigen::Matrix<float, 2, 1> matrix1, float adv, float side, float rot);
};
#endif