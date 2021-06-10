/*
 *    Copyright (C) 2020 by Mohamed Shawky
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

#include <QHBoxLayout>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <algorithm>
#include <iterator>
#include <fps/fps.h>
#include <doublebuffer/DoubleBuffer.h>
#include <random/random_selector.h>
#include  "../../../etc/viriato_graph_names.h"
#include "plan.h"
#include <QImage>

using my_clock = std::chrono::system_clock;
using msec = std::chrono::duration<int , std::milli>;

struct CONSTANTS_DATA
{
    float max_distance_between_target_and_pan_tilt = 150; //mm
    float max_pan_angle = 1; //rads
    float max_tilt_angle = 1; //rads
};
const CONSTANTS_DATA CONSTANTS;

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
        void stop_button_slot(bool);
        void reset_button_slot();
        void clear_button_slot();
        void change_attention_object_slot(int);
        void pan_slot(double);
        void tilt_slot(double);

private:
        struct WaitState
        {
            void init(int duration){ start = my_clock::now(); DURATION = duration; active = true;};
            bool waiting()
            {
                if(not active) return false;
                if (std::chrono::duration_cast<std::chrono::milliseconds>(my_clock::now() - start).count() >= DURATION)
                {
                    active = false;
                    return false;
                }
                else
                    return true;
            };
            bool active = false;
            int DURATION;
            std::chrono::time_point<my_clock> start;
        };
        WaitState wait_state;

        // PROTO SEMANTIC MEMORY
        std::map<std::string, std::vector<int>> known_object_types
            {
                    {glass_type_name,        {80, 100, 80}},
                    {cup_type_name,          {80, 100, 80}},
                    {microwave_type_name,    {450, 250, 350}},
                    {plant_type_name,        {500, 900, 500}},
                    {person_type_name,       {350, 1700, 350}},
                    {vase_type_name,         {300, 300, 350}},
                    {oven_type_name,         {400, 100, 400}},
                    {refrigerator_type_name, {600, 1600, 600}},
                    {apple_type_name,        {80, 80, 80}}
            };

        struct Box      // Bounding boxes struct. Y axis goes down from 0 to HEIGHT
        {
            std::string name;
            int left;
            int top;
            int right;
            int bot;
            float prob;
            float depth;
            int width() const { return right-left;};
            int height() const { return bot-top;};
            int cx() const { return left + width()/2;};
            int cy() const { return top + height()/2;};
        };

        // NODE NAMES
        std::string object_of_interest = "no_object";
        const std::string viriato_pan_tilt = "viriato_head_camera_pan_tilt";
        const std::string camera_name = "viriato_head_camera_sensor";
        const std::string world_node = "world";

        // ATTRIBUTE NAMES
        const std::string nose_target = "viriato_pan_tilt_nose_target";

        // DSR graph
        std::shared_ptr<DSR::DSRGraph> G;
        std::shared_ptr<DSR::CameraAPI> cam_api;
        std::shared_ptr<DSR::InnerEigenAPI> inner_eigen;
        std::shared_ptr<DSR::RT_API> rt_api;
        std::unique_ptr<DSR::AgentInfoAPI> agent_info_api;

        //DSR params
        std::string agent_name;
        std::string cfg_file;
        std::string weights_file;
        std::string names_file;

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
        void add_or_assign_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type);
        void del_edge_slot(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){};
        void del_node_slot(std::uint64_t from){};
        bool startup_check_flag;

        //local widget
        Custom_widget custom_widget;
        void initialize_combobox();
        FPSCounter fps;
        void print_data(const DSR::Node &target, int error, const Eigen::Vector3d &target_in_camera,
                        std::uint64_t cam_timestamp, const DSR::Node &pan_tilt,  const Eigen::Vector3f &vel, bool saccade);



    // Double buffer
        DoubleBuffer<std::vector<std::uint8_t>, std::tuple<cv::Mat, std::uint64_t>> rgb_buffer;
        DoubleBuffer<std::vector<float>, std::vector<float>> depth_buffer;
        DoubleBuffer<std::uint64_t, std::uint64_t> target_buffer;

        // Plan
        Plan current_plan;

        // Tracker
        void track_object_of_interest(DSR::Node &robot);
        void set_nose_target_to_default();
        void change_to_new_target();
        const std::vector<float> nose_default_pose{0.f, 1000.f, 0.f};
        enum class TState {IDLE, TRACKING, CHANGING };
        TState tracking_state = TState::IDLE;
        bool active = false;  //overall active switch

        // objects
        bool time_to_change = true;
        bool tracking = false;
        RandomSelector<> random_selector{};
        std::deque<std::uint64_t> set_of_objects_to_attend_to;
        std::uint64_t last_object_of_attention;
        std::vector<Box> compute_attention_list(const std::vector<Box> &synth_objects);
        void track_target_set( std::uint64_t target_id, std::uint64_t cam_timestamp);


        // IMAGE
        void show_image(cv::Mat image,  const std::string &target_name, std::uint64_t timestamp);
        std::tuple<int, int, int, int> project_target_on_image(const DSR::Node &target, std::uint64_t timestamp);

        // clamp for vectors with zero zone
        inline Eigen::Vector3f inner_clip(const Eigen::Vector3d& vec, float inner, float outer)
        {
            return vec.unaryExpr( [inner, outer](auto &x)
                {
                    if(fabs(x) < inner) return 0.f;
                    else return std::clamp((float)x, -outer, outer);
                });
        }
};

#endif
