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
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"
#include <QHBoxLayout>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>
#include <yolo_v2_class.hpp>
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
    // NODE NAMES
    const std::string object_of_interest = "glass_1";
    const std::string viriato_pan_tilt = "viriato_head_camera_pan_tilt";
    const std::string camera_name = "viriato_head_camera_sensor";
    const std::string world_node = "world";

    // ATTRIBUTE NAMES
    const std::string nose_target = "viriato_pan_tilt_nose_target";

    // DSR graph
    std::shared_ptr<DSR::DSRGraph> G;

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
    QWidget window;
    bool startup_check_flag;

    // YOLOv4 attributes
    const std::size_t YOLO_INSTANCES = 1;
    std::vector<Detector*> ynets;
    std::vector<std::string> names;
    clock_t ytime1;
    bool SHOW_IMAGE = false;
    bool READY_TO_GO = false;
    FPSCounter fps;

    // YOLOv4 methods
    Detector* init_detector();
    std::vector<bbox_t> process_image_with_yolo(const cv::Mat& img);
    image_t createImage(const cv::Mat &src);
    image_t createImage(const std::vector<uint8_t> &src, int width, int height, int depth);
    std::vector<bbox_t> detectLabels(const image_t &img, float thresh, float hier_thresh);
    void show_image(cv::Mat &imgdst, const vector<bbox_t> &boxes, const std::vector<bbox_t> synth_boxes);
    cv::Mat get_rgb_image(const DSR::Node &rgb_node);
    vector<bbox_t> process_graph_with_yolosynth(const std::vector<std::string> &object_names, const DSR::Node& rgb_cam);
    void compute_prediction_error(const vector<bbox_t> &real_boxes, const vector<bbox_t> synth_boxes);
};

#endif
