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
#include <yolo_v2_class.hpp>
#include <fps/fps.h>
#include <doublebuffer/DoubleBuffer.h>
#include <random/random_selector.h>
#include  "../../../etc/viriato_graph_names.h"
#include "plan.h"
#include <Eigen/Core>

#define YOLO_IMG_SIZE 512  // 608, 512 change also in yolo.cgf file

using myclock = std::chrono::system_clock;
using msec = std::chrono::duration<int , std::milli>;

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
        void clear_button_slot();
        void change_attention_object_slot(int);


private:
        struct CONSTANTS_DATA
        {
            float min_yolo_probability_threshold = 50;
            float times_the_width_of_synth_box = 3;
            int min_ticks_to_add_object_threshold = 7;
            int max_allowed_unseen_ticks = 50;
            int min_time_to_add_object_threshold = 2000;  // milliseconds
            float percentage_of_visible_area_to_be_visible = 25;
        };
        const CONSTANTS_DATA CONSTANTS;

        // Bounding boxes struct. Y axis goes down from 0 to HEIGHT
        struct Box
        {
            std::string name;
            int left;
            int top;
            int right;
            int bot;
            float prob;
            float depth;
            bool visible;
            float Tx, Ty, Tz;  // world ref sys
            float Cx, Cy, Cz;  // camera ref sys
            bool match;        // true if matched to a synth one
            float area;        // intersected area
            float match_error; // % of full intersection
            int width() const { return right-left;};
            int height() const { return bot-top;};
            int cx() const { return left + width()/2;};
            int cy() const { return top + height()/2;};
            bool marked_for_delete = false;  // to be deleted after match or creation
            int creation_ticks = 0;
            std::chrono::steady_clock::time_point creation_time;
            std::string type;
            float distance_in_world_frame_to(const Box &b) const
                    { return (Eigen::Vector3f(b.Tx,b.Ty,b.Tz)-Eigen::Vector3f(Tx,Ty,Tz)).norm();}
            void print(const std::string_view &s) const
            {
                std::cout << "--- " << s << " -----" << std::endl;
                std::cout << "\t name " << name << std::endl;
                std::cout << "\t prob " << prob << std::endl;
                std::cout << "\t visible " << visible << std::endl;
                std::cout << "\t match " << match << std::endl;
                std::cout << "\t depth " << depth << std::endl;
                std::cout << "\t box " << left << " " << top << " " << width() << " " << height() << std::endl;
                std::cout << "\t World: " << Tx << " " << Ty << " " << Tz << std::endl;
                std::cout << "\t Camera: " << Cx << " " << Cy << " " << Cz << std::endl;
                std::cout << "\t centre: " << cx() << " " << cy() << std::endl;
                std::cout << "\t match error: " << match_error << std::endl;
                std::cout << "\t area: " << area << std::endl;
                std::cout << "\t marked_for_delete: " << marked_for_delete << std::endl;
                std::cout << "---------------------" << std::endl;
            };
        };
        using Boxes = std::vector<Box>;

        // KNOWN OBJECTS

       std::map<std::string, std::vector<int>> known_object_types;

        // DSR graph
        std::shared_ptr<DSR::DSRGraph> G;
        std::shared_ptr<DSR::CameraAPI> cam_api;
        std::shared_ptr<DSR::InnerEigenAPI> inner_eigen;
        std::shared_ptr<DSR::RT_API> rt_api;

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

        // Double buffer
        //DoubleBuffer<std::vector<std::uint8_t>, std::vector<std::uint8_t>> rgb_buffer;
        DoubleBuffer<std::vector<std::uint8_t>, std::tuple<cv::Mat, float>> rgb_buffer;
        DoubleBuffer<std::vector<float>, std::vector<float>> depth_buffer;
        DoubleBuffer<std::string, Plan> plan_buffer;

        //Plan
        Plan current_plan;

        // YOLOv4 attributes
        const std::size_t YOLO_INSTANCES = 1;
        std::vector<Detector*> ynets;
        std::vector<std::string> yolo_names;
        bool SHOW_IMAGE = false;
        bool READY_TO_GO = false;
        FPSCounter fps;
        bool already_in_default = false;

        // YOLOv4 methods
        Detector* init_detector();
        std::vector<Box> process_image_with_yolo(const cv::Mat& img, const std::vector<float> &depth_array);
        image_t createImage(const cv::Mat &src);
        void show_image(cv::Mat &imgdst, const vector<Box> &real_boxes, const std::vector<Box> synth_boxes);
        bool both_boxes_match(Box &real_box, Box &synth_box);
        std::tuple<bool, std::string> contained_in_known_objects(const std::string &candidate);
        DSR::Node create_node_with_type(const std::string &type, const std::string &name);
        void remove_edge();
        void compute_visible_objects();
        std::vector<Box> get_visible_objects_from_graph();
        std::tuple<Boxes, Boxes> match_lists(Boxes &real_objects, Boxes &synth_objects, const std::vector<float> &depth_array);
        std::tuple<Boxes, Boxes> add_new_objects(std::tuple<Boxes, Boxes> &lists_after_match);
        std::tuple<Boxes, Boxes> delete_unseen_objects(std::tuple<Boxes, Boxes> &lists_after_add);
        std::tuple<float, float, float> estimate_object_size_through_projection_optimization(const Box &b_synth, const Box &b_real);
        bool real_object_is_stable(Box box, const QPolygonF &robot_room);
        std::tuple<float, float> get_random_position_to_draw_in_graph(const std::string &type);

        // objects
        bool time_to_change = true;
        bool tracking = false;
        RandomSelector<> random_selector{};

        // attention
        void initialize_combobox();
        std::uint64_t last_object_of_attention;
        void clear_all_attention_edges();


};

#endif
