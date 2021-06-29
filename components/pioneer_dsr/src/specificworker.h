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

#include "../../../etc/graph_names.h"

#include <genericworker.h>
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"
#include <doublebuffer/DoubleBuffer.h>
//#include "../../../etc/pioneer_world_names.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

class SpecificWorker : public GenericWorker
{
    using MyClock = std::chrono::system_clock;
    using mSec = std::chrono::duration<double, std::milli>;

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
        std::unique_ptr<DSR::RT_API> rt;


        //DSR params
        int agent_id;
        std::string agent_name;
        std::string dsr_input_file;

        // UI
        bool tree_view;
        bool graph_view;
        bool qscene_2d_view;
        bool osg_3d_view;

        // Flag camara robot
        bool robot_real;

        // DSR graph viewer
        std::unique_ptr<DSR::DSRViewer> graph_viewer;
        QHBoxLayout mainLayout;
        void add_or_assign_node_slot(std::uint64_t, const std::string &type);
        void add_or_assign_attrs_slot(std::uint64_t id, const std::map<std::string, DSR::Attribute> &attribs){};
        void add_or_assign_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type){};
        void del_edge_slot(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){};
        void del_node_slot(std::uint64_t from){};
        bool startup_check_flag;

        // remote services
        void update_robot_localization();
        void read_battery();
        void read_RSSI();

        //laser
        using Point = std::pair<float, float>;
        struct LaserPoint{ float dist; float angle;};
        std::vector<LaserPoint> read_laser_from_robot();
        void update_laser(const std::vector<LaserPoint> &laser_data);
        QPolygonF filter_laser(const std::vector<SpecificWorker::LaserPoint> &ldata);
        void ramer_douglas_peucker(const std::vector<Point> &pointList, double epsilon, std::vector<Point> &out);

        // virtual_frame
        void update_virtual(const cv::Mat &virtual_frame, float focalx, float focaly);
        std::optional<std::tuple<cv::Mat, std::vector<LaserPoint>>> compute_mosaic(int subsampling = 1);
        cv::Mat compute_virtual_frame();
        float focalx, focaly;
        bool are_different(const vector<float> &a, const vector<float> &b, const vector<float> &epsilon);
        template <typename T>
        inline bool is_in_bounds(const T& value, const T& low, const T& high) { return !(value < low) && (value < high); }
        void update_rgbd();

        // robot
        void check_base_velocity_reference();
        DoubleBuffer<std::tuple<float, float, float>, std::tuple<float, float, float>> base_target_buffer;
};

#endif
