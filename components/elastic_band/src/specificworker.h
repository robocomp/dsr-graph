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
#include  "../../../etc/graph_names.h"

#include <custom_widget.h>
#include <dsr/api/dsr_api.h>
#include <dsr/gui/dsr_gui.h>
#include <dsr/gui/viewers/qscene_2d_viewer/qscene_2d_viewer.h>
#include <localPerson.h>
#include <QGraphicsPolygonItem>
#include "grid.h"
#include <fps/fps.h>
#include <doublebuffer/DoubleBuffer.h>

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

private:
        // DSR graph
        std::shared_ptr<DSR::DSRGraph> G;
        std::unique_ptr<DSR::InnerEigenAPI> inner_eigen;
        std::unique_ptr<DSR::RT_API> rt_api;
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
        std::unique_ptr<DSR::DSRViewer> graph_viewer;
        QHBoxLayout mainLayout;
        void add_or_assign_node_slot(std::uint64_t, const std::string &type);
        void add_or_assign_attrs_slot(std::uint64_t id, const std::map<std::string, DSR::Attribute> &attribs){};
        void add_or_assign_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type){};
        void del_edge_slot(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){};
        void del_node_slot(std::uint64_t from){};
        bool startup_check_flag;

        //local widget
        Custom_widget custom_widget;
        FPSCounter fps;

        // drawing
        DSR::QScene2dViewer* widget_2d;
        std::vector<std::tuple<QVector2D, QVector2D>> forces_vector;

        // laser
        using LaserData = std::tuple<std::vector<float>, std::vector<float>>;  //<angles, dists>
        std::optional<std::tuple<QPolygonF, std::vector<QPointF>>> get_laser_data(const DSR::Node &node);

        // double buffers
        DoubleBuffer<std::vector<QPointF>, std::vector<QPointF>> path_buffer;
        DoubleBuffer<LaserData, std::tuple<QPolygonF, std::vector<QPointF>>> laser_buffer;

        //elastic band
        struct CONSTANTS   //will be reinitialzes in setparams
        {
            int number_of_not_visible_points = 0;  // should be 0 in very large grids
            float robot_length = 500;
            float robot_width = 400;
            float robot_radius = robot_length /2.0 ;
            float road_step_separation = robot_length * 0.9;
            float KE = 30;
            float KI = 10;
            float delta = 50;   // x and y displacement for gradient
            float max_laser_range = 4000.f;
            float very_large_distance = 100000.f;
            int max_free_energy_iterations = 3;
            float max_total_energy_ratio = 10;
            float max_distance_range = 1000;
            void update()
            {
                road_step_separation = robot_length * 0.9;
            }
        };
        CONSTANTS constants;

        std::uint64_t last_path_id;  // ID of last path node that came through the slot
        //enum class SearchState {NEW_TARGET, AT_TARGET, NO_TARGET_FOUND, NEW_FLOOR_TARGET};

        // robot draw
        Mat::Vector3d robotBottomLeft, robotBottomRight, robotTopRight, robotTopLeft;
        void draw_path( std::vector<QPointF> &path, QGraphicsScene *viewer_2d, const QPolygonF &laser_poly);
        QPolygonF get_robot_polygon();

        // band
        bool is_visible(QPointF p, const QPolygonF &laser_poly);
        bool is_point_visitable(const QPointF &point);
        void compute_forces(std::vector<QPointF> &path, const std::vector<QPointF> &laser_cart,
                            const QPolygonF &laser_poly,   const QPolygonF &current_robot_polygon,
                            const QPointF &current_robot_nose);
        void add_points(std::vector<QPointF> &path, const QPolygonF &laser_poly,  const QPolygonF &current_robot_polygon);
        void clean_points(std::vector<QPointF> &path, const QPolygonF &laser_poly,  const QPolygonF &current_robot_polygon);
        void save_path_in_G(const std::vector<QPointF> &path);

        // Grid
        Grid grid;
        std::shared_ptr<Collisions> collisions;
};

#endif
