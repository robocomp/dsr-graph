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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
    this->startup_check_flag = startup_check;
    QLoggingCategory::setFilterRules("*.debug=false\n");
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
    std::cout << "Destroying SpecificWorker" << std::endl;
    G->write_to_json_file("./"+agent_name+".json");
    G.reset();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
    agent_name = params["agent_name"].value;
    agent_id = stoi(params["agent_id"].value);
    tree_view = params["tree_view"].value == "true";
    graph_view = params["graph_view"].value == "true";
    qscene_2d_view = params["2d_view"].value == "true";
    osg_3d_view = params["3d_view"].value == "true";

    return true;
}

void SpecificWorker::initialize(int period)
{
    std::cout << "Initialize worker" << std::endl;
    this->Period = period;
    if(this->startup_check_flag)
    {
        this->startup_check();
    }
    else
    {
        timer.start(Period);
        // create graph
        G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id); // Init nodes
        std::cout<< __FUNCTION__ << "Graph loaded" << std::endl;  

        // Graph viewer
        using opts = DSR::DSRViewer::view;
        int current_opts = 0;
        opts main = opts::none;
        if(tree_view)
        {
            current_opts = current_opts | opts::tree;
        }
        if(graph_view)
        {
            current_opts = current_opts | opts::graph;
            main = opts::graph;
        }
        if(qscene_2d_view)
        {
            current_opts = current_opts | opts::scene;
        }
        if(osg_3d_view)
        {
            current_opts = current_opts | opts::osg;
        }
        graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
        setWindowTitle(QString::fromStdString(agent_name + "-") + QString::number(agent_id));

        this->Period = period;
        timer.start(Period);
    }
}

void SpecificWorker::compute()
{
    // read RGBD image from graph
    RoboCompCameraRGBDSimple::TImage rgb = get_rgb_from_G();
    RoboCompCameraRGBDSimple::TDepth depth = get_depth_from_G();
    // call pose estimation on RGBD and receive estimated poses
    RoboCompObjectPoseEstimationRGBD::PoseType poses;
    try
    {
        poses = this->objectposeestimationrgbd_proxy->getObjectPose(rgb, depth);
    }
    catch (const Ice::Exception &e)
    {
        std::cout << e << " No RoboCompPoseEstimation component found" << std::endl;
    }
    if (poses.size() != 0)
    {
        // inject estimated poses into graph
        this->inject_estimated_poses(poses);
    }
}

RoboCompCameraRGBDSimple::TImage SpecificWorker::get_rgb_from_G()
{
    // get head camera node
    auto cam = G->get_node("viriato_head_camera_sensor");
    if (cam.has_value())
    {
        // read RGB data attributes from graph 
        RoboCompCameraRGBDSimple::TImage rgb;
        try
        {
            const std::vector<uint8_t> rgb_data = G->get_rgb_image(cam.value());
            const auto width = G->get_attrib_by_name<int32_t>(cam.value(), "rgb_width");
            const auto height = G->get_attrib_by_name<int32_t>(cam.value(), "rgb_height");
            const auto depth = G->get_attrib_by_name<int32_t>(cam.value(), "rgb_depth");
            const auto cam_id = G->get_attrib_by_name<int32_t>(cam.value(), "rgb.cameraID");
            const auto focalx = G->get_attrib_by_name<int32_t>(cam.value(), "rgb_focalx");
            const auto focaly = G->get_attrib_by_name<int32_t>(cam.value(), "rgb_focaly");
            const auto alivetime = G->get_attrib_by_name<int32_t>(cam.value(), "rgb_alivetime");
            return rgb;
        }
        catch (const std::exception &e)
        {
            std::cout << __FILE__ << __FUNCTION__ << __LINE__ << " " << e.what() << std::endl;
            std::terminate();
        }
    }
    else
    {
        qFatal("Terminate in Compute. No node rgbd found");
    }
}

RoboCompCameraRGBDSimple::TDepth SpecificWorker::get_depth_from_G()
{
    // get head camera node
    auto cam = G->get_node("viriato_head_camera_sensor");
    if (cam.has_value())
    {
        // read depth data attributes from graph
        RoboCompCameraRGBDSimple::TDepth depth;
        try
        {
            const std::vector<uint8_t> depth_data = G->get_depth_image(cam.value());
            const auto width = G->get_attrib_by_name<int32_t>(cam.value(), "depth_width");
            const auto height = G->get_attrib_by_name<int32_t>(cam.value(), "depth_height");
            const auto cam_id = G->get_attrib_by_name<int32_t>(cam.value(), "depth.cameraID");
            const auto focalx = G->get_attrib_by_name<int32_t>(cam.value(), "focalx");
            const auto focaly = G->get_attrib_by_name<int32_t>(cam.value(), "focaly");
            const auto depth_factor = G->get_attrib_by_name<float_t>(cam.value(), "depthFactor");
            const auto alivetime = G->get_attrib_by_name<int32_t>(cam.value(), "alivetime");
            return depth;
        }
        catch(const std::exception& e)
        {
            std::cout << __FILE__ << __FUNCTION__ << __LINE__ << " " << e.what() << std::endl;
            std::terminate();
        }
    }
    else
    {
        qFatal("Terminate in Compute. No node rgbd found");
    }
}

void SpecificWorker::inject_estimated_poses(RoboCompObjectPoseEstimationRGBD::PoseType poses)
{
    // get innermodel sub-API
    auto innermodel = G->get_inner_api();
    // loop over each estimated object pose
    for (auto pose : poses)
    {
        // convert quaternions into euler angles
        vector<float> quat{pose.qx, pose.qy, pose.qz, pose.qw};
        vector<float> angles = this->quat_to_euler(quat);

        // re-project estimated poses into world coordinates
        QVec orig_point = QVec(6);
        orig_point.setItem(0, pose.x);
        orig_point.setItem(1, pose.y);
        orig_point.setItem(2, pose.z);
        orig_point.setItem(3, angles.at(0));
        orig_point.setItem(4, angles.at(1));
        orig_point.setItem(5, angles.at(2));
        auto final_pose = innermodel->transform("world", orig_point, "camera_pose");
    }
}

vector<float> SpecificWorker::quat_to_euler(vector<float> quat)
{
    // euler angles vector
    vector<float> angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (quat.at(3) * quat.at(0) + quat.at(1) * quat.at(2));
    double cosr_cosp = 1 - 2 * ( quat.at(0) *  quat.at(0) +  quat.at(1) *  quat.at(1));
    angles.push_back(std::atan2(sinr_cosp, cosr_cosp));

    // pitch (y-axis rotation)
    double sinp = 2 * (quat.at(3) * quat.at(1) - quat.at(2) * quat.at(0));
    if (std::abs(sinp) >= 1)
        angles.push_back(std::copysign(M_PI / 2, sinp)); // use 90 degrees if out of range
    else
        angles.push_back(std::asin(sinp));

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (quat.at(3) * quat.at(2) + quat.at(0) * quat.at(1));
    double cosy_cosp = 1 - 2 * (quat.at(1) * quat.at(1) + quat.at(2) * quat.at(2));
    angles.push_back(std::atan2(siny_cosp, cosy_cosp));

    return angles;
}

int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}

/**************************************/
// From the RoboCompObjectPoseEstimationRGBD you can call this methods:
// this->objectposeestimationrgbd_proxy->getObjectPose(...)

/**************************************/
// From the RoboCompObjectPoseEstimationRGBD you can use this types:
// RoboCompObjectPoseEstimationRGBD::ObjectPose
