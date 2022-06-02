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
#include "specificworker.h"
#include <cppitertools/zip.hpp>
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/enumerate.hpp>
#include <chrono>

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
    // TODO ESTO EN UN TRY CON .at()
    //////////////////////////////////
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
    // create graph
    G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, ""); // Init nodes
    std::cout<< __FUNCTION__ << "Graph loaded" << std::endl;

    //dsr update signals
    connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::add_or_assign_node_slot);
    connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::modify_edge_slot);
    //connect(G.get(), &DSR::DSRGraph::update_node_attr_signal, this, &SpecificWorker::modify_attrs_slot);
    connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
    connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);

    // Graph viewer
    using opts = DSR::DSRViewer::view;
    int current_opts = 0;
    opts main = opts::none;
    if(tree_view)
        current_opts = current_opts | opts::tree;
    if(graph_view)
    {
        current_opts = current_opts | opts::graph;
        main = opts::graph;
    }
    if(qscene_2d_view)
        current_opts = current_opts | opts::scene;
    if(osg_3d_view)
        current_opts = current_opts | opts::osg;
    graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
    setWindowTitle(QString::fromStdString(agent_name + "-") + QString::number(agent_id));

//        try {
////            float x = 3.85;
//            float x = 0.0;
////            float y = -22.387;
//            float y = 0.0;
//            float z = 0.0;
//            float rx = 0;
//            float ry = 0;
//            float rz = 180;
//            fullposeestimation_proxy->setInitialPose(x, y, z, rx, ry, rz);
//        }
//        catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; };


    if(auto robot_id = G->get_id_from_name(robot_name); robot_id.has_value())
        robot_id = robot_id.value();
    else
    {
        qWarning() << "No robot node found. Terminate";
        std::terminate();
    }
	this->Period = 1;
	if(this->startup_check_flag)
		this->startup_check();
	else
	{
		this->Period = period;
		timer.start(Period);
	}
}
void SpecificWorker::compute()
{

    auto t_start = std::chrono::high_resolution_clock::now();
    update_robot_localization();

    read_battery();
//    auto camera_rgbd_frame = compute_camera_rgbd_frame();

// FOR COPPELIA
try
{
    auto rgbd = camerargbdsimple_proxy->getImage("camera_top");
    cv::Mat rgbd_frame (cv::Size(rgbd.width, rgbd.height), CV_8UC3, &rgbd.image[0]);

    // FOR REAL ROBOT
//    auto rgbd = camerargbdsimple_proxy->getImage("");
//    cv::Mat rgbd_frame (cv::Size(rgbd.height, rgbd.width), CV_8UC3, &rgbd.image[0]);
//    cv::Mat rgbd_frame_rotated (cv::Size(rgbd.width, rgbd.height), CV_8UC3);
//    // Rotation process
//    cv::Point2f src_center(rgbd_frame.cols/2.0F, rgbd_frame.rows/2.0F);
//    cv::Mat rot_matrix = getRotationMatrix2D(src_center, 90, 1.0);
//    warpAffine(rgbd_frame, rgbd_frame_rotated, rot_matrix, rgbd_frame.size());
//
//    cv::cvtColor(rgbd_frame_rotated, rgbd_frame_rotated, 4);

    update_camera_rgbd(giraff_camera_realsense_name, rgbd_frame, rgbd.focalx, rgbd.focaly);
}
catch(const Ice::Exception &e) { /*std::cout << e.what() << std::endl;*/}



//    auto camera_simple_frame = compute_camera_simple_frame();
//    update_camera_simple(giraff_camera_usb_name, camera_simple_frame);
    //auto camera_simple1_frame = compute_camera_simple1_frame();
    //update_camera_simple1(giraff_camera_face_id_name, camera_simple1_frame);
    update_servo_position();
    auto laser = read_laser_from_robot();
    update_laser(laser);
    auto t_end = std::chrono::high_resolution_clock::now();
    double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();
//    std::cout << elapsed_time_ms << std::endl;
}
void SpecificWorker::read_battery()
{
    try
    {
        auto battery_level = batterystatus_proxy->getBatteryState();
        if( auto battery = G->get_node(battery_name); battery.has_value())
        {
            G->add_or_modify_attrib_local<battery_load_att>(battery.value(), (100*battery_level.percentage));
            G->update_node(battery.value());
        }
    }
    catch(const Ice::Exception &e) { /*std::cout << e.what() << std::endl;*/}
}

void SpecificWorker::update_servo_position()
{
    try
    {
        auto servo_data = this->jointmotorsimple_proxy->getMotorState("eye_motor");
        float servo_position = (float)servo_data.pos;
        float servo_vel = (float)servo_data.vel;
        bool moving = servo_data.isMoving;
        if( auto servo = G->get_node("servo"); servo.has_value())
        {
            G->add_or_modify_attrib_local<servo_pos_att>(servo.value(), servo_position);
            G->add_or_modify_attrib_local<servo_speed_att>(servo.value(), servo_vel);
            G->add_or_modify_attrib_local<servo_moving_att>(servo.value(), moving);
            G->update_node(servo.value());
        }
    }
    catch(const Ice::Exception &e){ /*std::cout << e.what() <<  __FUNCTION__ << std::endl;*/};
}

void SpecificWorker::update_robot_localization()
{
    static RoboCompFullPoseEstimation::FullPoseEuler last_state;
    RoboCompFullPoseEstimation::FullPoseEuler pose;
    try
    {
        pose = fullposeestimation_proxy->getFullPoseEuler();
        //qInfo() << "X:" << pose.x  << "// Y:" << pose.y << "// Z:" << pose.z << "// RX:" << pose.rx << "// RY:" << pose.ry << "// RZ:" << pose.rz;
    }
    catch(const Ice::Exception &e){ /*std::cout << e.what() <<  __FUNCTION__ << std::endl;*/};

    if( auto robot = G->get_node(robot_name); robot.has_value())
    {
        if( auto parent = G->get_parent_node(robot.value()); parent.has_value())
        {
            if (are_different(std::vector < float > {pose.x, pose.y, pose.rz},
                          std::vector < float > {last_state.x, last_state.y, last_state.rz},
                          std::vector < float > {1, 1, 0.05}))
            {
                auto edge = rt->get_edge_RT(parent.value(), robot->id()).value();
                G->modify_attrib_local<rt_rotation_euler_xyz_att>(edge, std::vector < float > {0.0, 0.0, pose.rz});
                G->modify_attrib_local<rt_translation_att>(edge, std::vector < float > {pose.x, pose.y, 0.0});
                G->modify_attrib_local<rt_translation_velocity_att>(edge, std::vector<float>{pose.vx, pose.vy, pose.vz});
                G->modify_attrib_local<rt_rotation_euler_xyz_velocity_att>(edge, std::vector<float>{pose.vrx, pose.vry, pose.vrz});
                // linear velocities are WRT world axes, so local speed has to be computed WRT to the robot's moving frame
                float side_velocity = -sin(pose.rz) * pose.vx + cos(pose.rz) * pose.vy;
                float adv_velocity = -cos(pose.rz) * pose.vx + sin(pose.rz) * pose.vy;
                G->insert_or_assign_edge(edge);
//                std::cout << "VELOCITIES: " << adv_velocity << " " << pose.vrz << std::endl;


                G->add_or_modify_attrib_local<robot_local_linear_velocity_att>(robot.value(), std::vector<float>{adv_velocity, side_velocity, pose.rz});
                G->add_or_modify_attrib_local<robot_local_advance_speed_att>(robot.value(), -adv_velocity);
                G->add_or_modify_attrib_local<robot_local_rotational_speed_att>(robot.value(), pose.vrz);

                G->update_node(robot.value());
                last_state = pose;
            }
        }
        else  qWarning() << __FUNCTION__ << " No parent found for node " << QString::fromStdString(robot_name);
    }
    else    qWarning() << __FUNCTION__ << " No node " << QString::fromStdString(robot_name);
}



cv::Mat SpecificWorker::compute_camera_rgbd_frame()
{
    RoboCompCameraRGBDSimple::TImage rgbd;
    cv::Mat rgbd_frame;
    try
    {
        auto rgbd = camerargbdsimple_proxy->getImage("");
        this->focalx = rgbd.focalx;
        this->focaly = rgbd.focaly;

//        if(!cdata_camera_rgbd.image.empty())
//        {
//            camera_rgbd_frame = cv::imdecode(cdata_camera_rgbd.image, -1 );
//            cv::cvtColor(camera_rgbd_frame, camera_rgbd_frame, cv::COLOR_BGR2RGB);
//        }
        rgbd_frame = cv::Mat(cv::Size(rgbd.width, rgbd.height), CV_8UC3, &rgbd.image[0]);
    }
    catch (const Ice::Exception &e){ /*std::cout << e.what() << std::endl;*/}
    return rgbd_frame;
}

void SpecificWorker::update_camera_rgbd(std::string camera_name, const cv::Mat &v_image, float focalx, float focaly)
{

    if( auto node = G->get_node(camera_name); node.has_value())
    {
        std::vector<uint8_t> rgb; rgb.assign(v_image.data, v_image.data + v_image.total()*v_image.channels());
        G->add_or_modify_attrib_local<cam_rgb_att>(node.value(),  rgb);
        G->add_or_modify_attrib_local<cam_rgb_width_att>(node.value(), v_image.cols);
        G->add_or_modify_attrib_local<cam_rgb_height_att>(node.value(), v_image.rows);
        G->add_or_modify_attrib_local<cam_rgb_depth_att>(node.value(), v_image.depth());
        G->add_or_modify_attrib_local<cam_rgb_cameraID_att>(node.value(), 3);
        G->add_or_modify_attrib_local<cam_rgb_focalx_att>(node.value(), (int)focalx);
        G->add_or_modify_attrib_local<cam_rgb_focaly_att>(node.value(), (int)focaly);
        G->add_or_modify_attrib_local<cam_rgb_alivetime_att>(node.value(), (int)std::chrono::time_point_cast<std::chrono::milliseconds>(MyClock::now()).time_since_epoch().count());

        G->update_node(node.value());
    }
    else
        qWarning() << __FUNCTION__ << "Node camera_rgbd not found";
}

Eigen::Vector2f SpecificWorker::from_world_to_robot(const Eigen::Vector2f &p,
                                                    const RoboCompFullPoseEstimation::FullPoseEuler &r_state)
{
    Eigen::Matrix2f matrix;
    matrix << cos(r_state.rz) , -sin(r_state.rz) , sin(r_state.rz) , cos(r_state.rz);
    return (matrix.transpose() * (p - Eigen::Vector2f(r_state.x, r_state.y)));
}

cv::Mat SpecificWorker::compute_camera_simple_frame()
{
    RoboCompCameraSimple::TImage cdata_camera_simple;
    cv::Mat camera_simple_frame;
    try
    {
        cdata_camera_simple = camerasimple_proxy->getImage();
        //this->focalx = cdata_camera_simple.focalx;
        //this->focaly = cdata_camera_simple.focaly;
        if( not cdata_camera_simple.image.empty())
        {
            if(cdata_camera_simple.compressed)
                camera_simple_frame = cv::imdecode(cdata_camera_simple.image, -1 );
            else
                camera_simple_frame = cv::Mat(cv::Size(cdata_camera_simple.width, cdata_camera_simple.height), CV_8UC3, &cdata_camera_simple.image[0], cv::Mat::AUTO_STEP);
            cv::cvtColor(camera_simple_frame, camera_simple_frame, cv::COLOR_BGR2RGB);
        }
    }
    catch (const Ice::Exception &e){ /*std::cout << e.what() <<  " In compute_camera_simple_frame" << std::endl;*/}
    return camera_simple_frame;
}

void SpecificWorker::update_camera_simple(std::string camera_name, const cv::Mat &v_image)
{

    if( auto node = G->get_node(camera_name); node.has_value())
    {
        std::vector<uint8_t> rgb; rgb.assign(v_image.data, v_image.data + v_image.total()*v_image.channels());
        G->add_or_modify_attrib_local<cam_rgb_att>(node.value(),  rgb);
        G->add_or_modify_attrib_local<cam_rgb_width_att>(node.value(), v_image.cols);
        G->add_or_modify_attrib_local<cam_rgb_height_att>(node.value(), v_image.rows);
        G->add_or_modify_attrib_local<cam_rgb_depth_att>(node.value(), v_image.depth());
        G->add_or_modify_attrib_local<cam_rgb_cameraID_att>(node.value(), 3);
        G->add_or_modify_attrib_local<cam_rgb_alivetime_att>(node.value(), (int)std::chrono::time_point_cast<std::chrono::milliseconds>(MyClock::now()).time_since_epoch().count());

        G->update_node(node.value());
    }
    else
        qWarning() << __FUNCTION__ << "Node camera_simple not found";
}

cv::Mat SpecificWorker::compute_camera_simple1_frame()
{
    RoboCompCameraSimple::TImage cdata_camera_simple1;
    cv::Mat camera_simple1_frame;
    try
    {
        cdata_camera_simple1 = camerasimple1_proxy->getImage();
        //this->focalx = cdata_camera_simple.focalx;
        //this->focaly = cdata_camera_simple.focaly;

        if(!cdata_camera_simple1.image.empty()) {
            camera_simple1_frame = cv::imdecode(cdata_camera_simple1.image, -1 );
            //cv::cvtColor(camera_simple1_frame, camera_simple1_frame, cv::COLOR_BGR2RGB);
        }
    }
    catch (const Ice::Exception &e){ /*std::cout << e.what() << std::endl;*/}

    return camera_simple1_frame;
}

void SpecificWorker::update_camera_simple1(std::string camera_name, const cv::Mat &v_image)
{

    if( auto node = G->get_node(camera_name); node.has_value())
    {
        std::vector<uint8_t> rgb; rgb.assign(v_image.data, v_image.data + v_image.total()*v_image.channels());
        G->add_or_modify_attrib_local<cam_rgb_att>(node.value(),  rgb);
        G->add_or_modify_attrib_local<cam_rgb_width_att>(node.value(), v_image.cols);
        G->add_or_modify_attrib_local<cam_rgb_height_att>(node.value(), v_image.rows);
        G->add_or_modify_attrib_local<cam_rgb_depth_att>(node.value(), v_image.depth());
        G->add_or_modify_attrib_local<cam_rgb_cameraID_att>(node.value(), 3);
        G->add_or_modify_attrib_local<cam_rgb_alivetime_att>(node.value(), (int)std::chrono::time_point_cast<std::chrono::milliseconds>(MyClock::now()).time_since_epoch().count());

        G->update_node(node.value());
    }
    else
        qWarning() << __FUNCTION__ << "Node camera_simple1 not found";
}

void SpecificWorker::update_rgbd()
{
    RoboCompCameraRGBDSimple::TImage rgb;
    try
    {
        rgb = camerargbdsimple_proxy->getImage("giraff_camera_realsense");
    }
    catch (const Ice::Exception &e){ /*std::cout << e.what() << std::endl;*/}

    if( auto node = G->get_node(giraff_camera_realsense_name); node.has_value())
    {
        G->add_or_modify_attrib_local<cam_rgb_att>(node.value(), rgb.image);
        G->add_or_modify_attrib_local<cam_rgb_width_att>(node.value(), rgb.width);
        G->add_or_modify_attrib_local<cam_rgb_height_att>(node.value(), rgb.height);
        G->add_or_modify_attrib_local<cam_rgb_depth_att>(node.value(), rgb.depth);
        G->add_or_modify_attrib_local<cam_rgb_cameraID_att>(node.value(), rgb.cameraID);
        G->add_or_modify_attrib_local<cam_rgb_focalx_att>(node.value(), rgb.focalx);
        G->add_or_modify_attrib_local<cam_rgb_focaly_att>(node.value(), rgb.focaly);
        G->add_or_modify_attrib_local<cam_rgb_alivetime_att>(node.value(), rgb.alivetime);
        // depth
//        G->add_or_modify_attrib_local<cam_depth_att>(node.value(), depth.depth);
//        G->add_or_modify_attrib_local<cam_depth_width_att>(node.value(), depth.width);
//        G->add_or_modify_attrib_local<cam_depth_height_att>(node.value(), depth.height);
//        G->add_or_modify_attrib_local<cam_depth_focalx_att>(node.value(), depth.focalx);
//        G->add_or_modify_attrib_local<cam_depth_focaly_att>(node.value(), depth.focaly);
//        G->add_or_modify_attrib_local<cam_depth_cameraID_att>(node.value(), depth.cameraID);
//        G->add_or_modify_attrib_local<cam_depthFactor_att>(node.value(), depth.depthFactor);
//        G->add_or_modify_attrib_local<cam_depth_alivetime_att>(node.value(), depth.alivetime);
        G->update_node(node.value());
    }
    else
        qWarning() << __FUNCTION__ << "Node not found";
}



int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

//////////////////// AUX ///////////////////////////////////////////////
bool SpecificWorker::are_different(const std::vector<float> &a, const std::vector<float> &b, const std::vector<float> &epsilon)
{
    for(auto &&[aa, bb, e] : iter::zip(a, b, epsilon))
        if (fabs(aa - bb) > e)
            return true;
    return false;
};

//////////////////////////////// LASER ////////////////////////////////

std::vector<SpecificWorker::LaserPoint> SpecificWorker::read_laser_from_robot()
{
    std::vector<LaserPoint> laser_data;

    try {
        auto laser = laser_proxy->getLaserData();
        //for(auto &d : laser)
        //    qInfo() << d.angle << d.dist;
        std::transform(laser.begin(), laser.end(), std::back_inserter(laser_data), [](const auto &l) {return LaserPoint{l.dist, l.angle}; });
    }catch (const Ice::Exception &e){ /*std::cout << e.what() << " No laser_pioneer_data" << std::endl;*/ return {};}

    return laser_data;
}

void SpecificWorker::update_laser(const std::vector<LaserPoint> &laser_data)
{
    if( auto node = G->get_node(laser_name); node.has_value())
    {
        // Transform laserData into two std::vector<float>
        std::vector<float> dists;
        std::transform(laser_data.begin(), laser_data.end(), std::back_inserter(dists), [](const auto &l) { return l.dist; });
        std::vector<float> angles;
        std::transform(laser_data.begin(), laser_data.end(), std::back_inserter(angles), [](const auto &l) { return l.angle; });

        // update laser in DSR
        G->add_or_modify_attrib_local<laser_dists_att>(node.value(), dists);
        G->add_or_modify_attrib_local<laser_angles_att>(node.value(), angles);
        G->update_node(node.value());
    }
    else
        qWarning() << __FUNCTION__ << "No laser node found";
}

QPolygonF SpecificWorker::filter_laser(const std::vector<SpecificWorker::LaserPoint> &ldata)
{
    static const float MAX_RDP_DEVIATION_mm  =  70;
    static const float MAX_SPIKING_ANGLE_rads = 0.2;
    QPolygonF laser_poly;
    try
    {
        // Simplify laser contour with Ramer-Douglas-Peucker
        std::vector<Point> plist(ldata.size()+1);
        plist[0]=std::make_pair(0,0);
        std::generate(plist.begin()+1, plist.end(), [ldata, k=0]() mutable
        { auto &l = ldata[k++]; return std::make_pair(l.dist * sin(l.angle), l.dist * cos(l.angle));});
        std::vector<Point> pointListOut;
        ramer_douglas_peucker(plist, MAX_RDP_DEVIATION_mm, pointListOut);
        laser_poly.resize(pointListOut.size());
        std::generate(laser_poly.begin(), laser_poly.end(), [pointListOut, this, k=0]() mutable
        { auto &p = pointListOut[k++]; return QPointF(p.first, p.second);});
        laser_poly << QPointF(0,0);

        // Filter out spikes. If the angle between two line segments is less than to the specified maximum angle
        std::vector<QPointF> removed;
        for(auto &&[k, ps] : iter::sliding_window(laser_poly,3) | iter::enumerate)
            if( MAX_SPIKING_ANGLE_rads > acos(QVector2D::dotProduct( QVector2D(ps[0] - ps[1]).normalized(), QVector2D(ps[2] - ps[1]).normalized())))
                removed.push_back(ps[1]);
        for(auto &&r : removed)
            laser_poly.erase(std::remove_if(laser_poly.begin(), laser_poly.end(), [r](auto &p) { return p == r; }), laser_poly.end());
    }
    catch(const Ice::Exception &e)
    { std::cout << "Error reading from Laser" << e << std::endl;}
    laser_poly.pop_back();
    return laser_poly;  // robot coordinates
}

void SpecificWorker::ramer_douglas_peucker(const std::vector<Point> &pointList, double epsilon, std::vector<Point> &out)
{
    if(pointList.size()<2)
    {
        qWarning() << "Not enough points to simplify";
        return;
    }
    // Find the point with the maximum distance from line between start and end
    auto line = Eigen::ParametrizedLine<float, 2>::Through(Eigen::Vector2f(pointList.front().first, pointList.front().second),
                                                           Eigen::Vector2f(pointList.back().first, pointList.back().second));
    auto max = std::max_element(pointList.begin()+1, pointList.end(), [line](auto &a, auto &b)
    { return line.distance(Eigen::Vector2f(a.first, a.second)) < line.distance(Eigen::Vector2f(b.first, b.second));});
    float dmax =  line.distance(Eigen::Vector2f((*max).first, (*max).second));

    // If max distance is greater than epsilon, recursively simplify
    if(dmax > epsilon)
    {
        // Recursive call
        vector<Point> recResults1;
        vector<Point> recResults2;
        vector<Point> firstLine(pointList.begin(), max + 1);
        vector<Point> lastLine(max, pointList.end());

        ramer_douglas_peucker(firstLine, epsilon, recResults1);
        ramer_douglas_peucker(lastLine, epsilon, recResults2);

        // Build the result list
        out.assign(recResults1.begin(), recResults1.end() - 1);
        out.insert(out.end(), recResults2.begin(), recResults2.end());
        if (out.size() < 2)
        {
            qWarning() << "Problem assembling output";
            return;
        }
    }
    else
    {
        //Just return start and end points
        out.clear();
        out.push_back(pointList.front());
        out.push_back(pointList.back());
    }
}


///////////////////////////////////////////////////////////////////
/// Asynchronous changes on G nodes from G signals
///////////////////////////////////////////////////////////////////

void SpecificWorker::add_or_assign_node_slot(const std::uint64_t id, const std::string &type)
{
    std::cout << "SIGNAL: " << type << std::endl;
    if (type == "servo")
    {
        //            // servo
        if (auto servo = G->get_node("servo"); servo.has_value())
        {
//            std::cout << "ENTERING IN SERVO" << std::endl;
            if(auto servo_send_pos = G->get_attrib_by_name<servo_send_pos_att>(servo.value()); servo_send_pos.has_value())
            {
                float servo_pos = servo_send_pos.value();
                if(auto servo_send_speed = G->get_attrib_by_name<servo_send_speed_att>(servo.value()); servo_send_speed.has_value())
                {
                    float servo_speed = servo_send_speed.value();
                    servo_pos_anterior = servo_pos;
                    servo_speed_anterior =  servo_speed;

                    try {
//                        std::cout << "SENDING POSITION" << std::endl;
                        RoboCompJointMotorSimple::MotorGoalPosition goal;
                        goal.maxSpeed = (float)servo_speed;
                        goal.position = (float)servo_pos;
                        this->jointmotorsimple_proxy->setPosition("eye_motor", goal);
                    }
                    catch (const RoboCompGenericBase::HardwareFailedException &re) {
//                        std::cout << __FUNCTION__ << "Exception setting base speed " << re << '\n';
//                        std::cout << __FUNCTION__ << "Exception setting base speed " << re << '\n';
                    }
                    catch (const Ice::Exception &e) {
                        //std::cout << e.what() << '\n';
                    }
                }
            }

        }
    }

    if (type == differentialrobot_type_name)   // pasar al SLOT the change attrib
    {
        qInfo() << __FUNCTION__  << "DIFFERIAL ROBOT";
        if (auto robot = G->get_node(robot_name); robot.has_value())
        {
            // speed
            auto ref_adv_speed = G->get_attrib_by_name<robot_ref_adv_speed_att>(robot.value());
            auto ref_rot_speed = G->get_attrib_by_name<robot_ref_rot_speed_att>(robot.value());
//            qInfo() << __FUNCTION__ << ref_adv_speed.has_value() << ref_rot_speed.has_value();
            if (ref_adv_speed.has_value() and ref_rot_speed.has_value())
            {
                //comprobar si la velocidad ha cambiado y el cambio es mayor de 10mm o algo así, entonces entra y tiene que salir estos mensajes
                std::cout << __FUNCTION__  <<endl;
                // Check de values are within robot's accepted range. Read them from config
                //const float lowerA = -10, upperA = 10, lowerR = -10, upperR = 5, lowerS = -10, upperS = 10;
                //std::clamp(ref_adv_speed.value(), lowerA, upperA);
                float adv = ref_adv_speed.value();
                float rot = ref_rot_speed.value();
                //float inc = 10.0;
                cout << __FUNCTION__ << "adv " << adv << " rot " << rot << endl;
                if ( adv != av_anterior or rot != rot_anterior)
                {
                    std::cout<< "..................................."<<endl;
//                    std::cout << __FUNCTION__ << " " << ref_adv_speed.value() << " " << ref_rot_speed.value()
//                              << std::endl;
                    av_anterior = adv;
                    rot_anterior = rot;
                    try {
                        differentialrobot_proxy->setSpeedBase(ref_adv_speed.value(), ref_rot_speed.value());
                        std::cout << "VELOCIDADES: " << ref_adv_speed.value() << " " << ref_rot_speed.value() << std::endl;
                    }
                    catch (const RoboCompGenericBase::HardwareFailedException &re) {
//                        std::cout << __FUNCTION__ << "Exception setting base speed " << re << '\n';
//                        std::cout << __FUNCTION__ << "Exception setting base speed " << re << '\n';
                    }
                    catch (const Ice::Exception &e) {
                        //std::cout << e.what() << '\n';
                    }
                }
            }
        }
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////


/**************************************/
// From the RoboCompBatteryStatus you can call this methods:
// this->batterystatus_proxy->getBatteryState(...)

/**************************************/
// From the RoboCompBatteryStatus you can use this types:
// RoboCompBatteryStatus::TBattery

/**************************************/
// From the RoboCompCameraRGBDSimple you can call this methods:
// this->camerargbdsimple_proxy->getAll(...)
// this->camerargbdsimple_proxy->getDepth(...)
// this->camerargbdsimple_proxy->getImage(...)

/**************************************/
// From the RoboCompCameraRGBDSimple you can use this types:
// RoboCompCameraRGBDSimple::TImage
// RoboCompCameraRGBDSimple::TDepth
// RoboCompCameraRGBDSimple::TRGBD

/**************************************/
// From the RoboCompCameraSimple you can call this methods:
// this->camerasimple_proxy->getImage(...)

/**************************************/
// From the RoboCompCameraSimple you can use this types:
// RoboCompCameraSimple::TImage

/**************************************/
// From the RoboCompCameraSimple you can call this methods:
// this->camerasimple1_proxy->getImage(...)

/**************************************/
// From the RoboCompCameraSimple you can use this types:
// RoboCompCameraSimple::TImage

/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// this->differentialrobot_proxy->correctOdometer(...)
// this->differentialrobot_proxy->getBasePose(...)
// this->differentialrobot_proxy->getBaseState(...)
// this->differentialrobot_proxy->resetOdometer(...)
// this->differentialrobot_proxy->setOdometer(...)
// this->differentialrobot_proxy->setOdometerPose(...)
// this->differentialrobot_proxy->setSpeedBase(...)
// this->differentialrobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompFullPoseEstimation you can call this methods:
// this->fullposeestimation_proxy->getFullPoseEuler(...)
// this->fullposeestimation_proxy->getFullPoseMatrix(...)
// this->fullposeestimation_proxy->setInitialPose(...)

/**************************************/
// From the RoboCompFullPoseEstimation you can use this types:
// RoboCompFullPoseEstimation::FullPoseMatrix
// RoboCompFullPoseEstimation::FullPoseEuler

/**************************************/
// From the RoboCompGiraff you can call this methods:
// this->giraff_proxy->decTilt(...)
// this->giraff_proxy->getBotonesState(...)
// this->giraff_proxy->getTilt(...)
// this->giraff_proxy->incTilt(...)
// this->giraff_proxy->setTilt(...)

/**************************************/
// From the RoboCompGiraff you can use this types:
// RoboCompGiraff::Botones

/**************************************/
// From the RoboCompLaser you can call this methods:
// this->laser_proxy->getLaserAndBStateData(...)
// this->laser_proxy->getLaserConfData(...)
// this->laser_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

/**************************************/
// From the RoboCompRealSenseFaceID you can call this methods:
// this->realsensefaceid_proxy->authenticate(...)
// this->realsensefaceid_proxy->enroll(...)
// this->realsensefaceid_proxy->eraseAll(...)
// this->realsensefaceid_proxy->eraseUser(...)
// this->realsensefaceid_proxy->getQueryUsers(...)
// this->realsensefaceid_proxy->startPreview(...)
// this->realsensefaceid_proxy->stopPreview(...)

/**************************************/
// From the RoboCompRealSenseFaceID you can use this types:
// RoboCompRealSenseFaceID::UserData


