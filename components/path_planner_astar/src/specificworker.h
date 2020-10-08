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
#include "grid.cpp"
#include "grid.h"
#include <custom_widget.h>
#include <dsr/api/dsr_api.h>
#include <dsr/gui/dsr_gui.h>
#include <dsr/gui/viewers/qscene_2d_viewer/qscene_2d_viewer.h>
#include <localPerson.h>
#include <QGraphicsPolygonItem>
#include <doublebuffer/DoubleBuffer.h>
#include  "../../../etc/viriato_graph_names.h"

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
        void update_node_slot(const std::int32_t id, const std::string &type);

    private:
        // DSR graph
        std::shared_ptr<DSR::DSRGraph> G;
        std::shared_ptr<DSR::InnerEigenAPI> inner_eigen;
        std::shared_ptr<DSR::RT_API> rt_api;

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
        bool startup_check_flag;

        //local widget
        Custom_widget custom_widget;

        //drawing
        DSR::QScene2dViewer* widget_2d;

        //Plan
        Plan current_plan;
        DoubleBuffer<std::string, Plan> plan_buffer;
        void json_to_plan(const std::string &plan_string, Plan &plan);

        //Path planner
        enum class SearchState {NEW_TARGET, AT_TARGET, NO_TARGET_FOUND, NEW_FLOOR_TARGET};
        std::tuple<SearchState, Mat::Vector2d> search_a_feasible_target(const Node &target, const std::map<std::string, double> &params, const Node &robot);
        void path_planner_initialize(  QGraphicsScene *scene, bool read_from_file = false, const std::string file_name = std::string());

        std::shared_ptr<Collisions> collisions;
        Grid<> grid;
        float robotXWidth, robotZLong; //robot dimensions read from config
        Mat::Vector3d robotBottomLeft, robotBottomRight, robotTopRight, robotTopLeft;
        Grid<>::Dimensions dim;
        void draw_path( std::list<QPointF> &path, QGraphicsScene *viewer_2d);

};

#endif
