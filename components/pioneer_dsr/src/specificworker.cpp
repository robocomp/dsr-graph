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
	robot_real = params["robot_real"].value == "true";

    dsr_input_file = params["dsr_input_file"].value;

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
		G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, ""); // Init nodes
		std::cout<< __FUNCTION__ << "Graph loaded" << std::endl;

		//APIS
        rt = G->get_rt_api();
        inner_eigen = G->get_inner_eigen_api();

		//dsr update signals
		connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::add_or_assign_node_slot);
		//connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::add_or_assign_edge_slot);
		//connect(G.get(), &DSR::DSRGraph::update_attrs_signal, this, &SpecificWorker::add_or_assign_attrs_slot);
		//connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
		//connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);

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

		if(robot_real)
		{
		    qInfo() << "Robot real";
            try {
                float x = 3305;
                float y = -21699;
                float z = 0;
                float rx = 0;
                float ry = 0;
                float rz = 0;
                fullposeestimation_proxy->setInitialPose(x, y, z, rx, ry, rz);
            }
            catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; };
        }

        this->Period = period;
        timer.start(Period);
	}
}

void SpecificWorker::compute()
{
    update_robot_localization();
    if (robot_real)
    {
        //Llamada a metodo para guardar la imagen virtual del robot
        auto virtual_frame = compute_virtual_frame();
        // get laser data from robot and call update_laser
        auto laser = read_laser_from_robot();
        update_virtual(virtual_frame, focalx, focaly);
        update_laser(laser);
    }
    else // Coppelia
    {
        if (auto res = compute_mosaic(); res.has_value())
        {
            auto &[virtual_frame, laser] = res.value();
            update_virtual(virtual_frame, focalx, focaly);
            update_laser(laser);
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////

std::vector<SpecificWorker::LaserPoint> SpecificWorker::read_laser_from_robot()
{
    std::vector<LaserPoint> laser_data;

    try {
        auto laser = laser_proxy->getLaserData();
        //for(auto &d : laser)
        //    qInfo() << d.angle << d.dist;
        std::transform(laser.begin(), laser.end(), std::back_inserter(laser_data), [](const auto &l) {return LaserPoint{l.dist, l.angle}; });
    }catch (const Ice::Exception &e){ std::cout << e.what() << " No laser_pioneer_data" << std::endl; return {};}

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
std::optional<std::tuple<cv::Mat, std::vector<SpecificWorker::LaserPoint>>> SpecificWorker::compute_mosaic(int subsampling)
{
    RoboCompCameraRGBDSimple::TRGBD cdata_left, cdata_right;
    try
    {
        cdata_left = camerargbdsimple_proxy->getAll("pioneer_camera_left");
        this->focalx = cdata_left.image.focalx;
        this->focaly = cdata_left.image.focaly;
    }
    catch (const Ice::Exception &e){ std::cout << e.what() << " No pioneer_camera_left" << std::endl; return {};}
    try
    {  cdata_right = camerargbdsimple_proxy->getAll("pioneer_camera_right"); }
    catch (const Ice::Exception &e){ std::cout << e.what() << " No pioneer_camera_right" << std::endl; return {};}

    // check that both cameras are equal
    // declare frame virtual
    cv::Mat frame_virtual = cv::Mat::zeros(cv::Size(cdata_left.image.width*2.5, cdata_left.image.height), CV_8UC3);
    float center_virtual_i = frame_virtual.cols / 2.0;
    float center_virtual_j = frame_virtual.rows / 2.0;
    float frame_virtual_focalx = cdata_left.image.focalx;
    auto before = MyClock::now();

    // laser stuff
    const int MAX_LASER_BINS = 100;
    const float TOTAL_HOR_ANGLE = 2.094;  // para 120º
    using Point = std::tuple< float, float, float>;
    auto cmp = [](Point a, Point b) { auto &[ax,ay,az] = a; auto &[bx,by,bz] = b; return (ax*ax+ay*ay+az*az) < (bx*bx+by*by+bz*bz);};
    std::vector<std::set<Point, decltype(cmp)>> hor_bins(MAX_LASER_BINS);

    // Left image check that rgb and depth are equal
    if(cdata_left.image.width == cdata_left.depth.width and cdata_left.image.height == cdata_left.depth.height)
    {
        // cast depth
        float *depth_array = (float *) cdata_left.depth.depth.data();
        // cast rgb
        const auto &rgb_img_data = const_cast<std::vector<uint8_t> &>(cdata_left.image.image).data();
        // vars
        float X, Y, Z;
        int cols, rows;
        std::size_t num_pixels = cdata_left.depth.depth.size() / sizeof(float);
        float coseno = cos(-M_PI / 6.0);
        float seno = sin(-M_PI / 6.0);
        float h_offset = -100;
        for (std::size_t i = 0; i < num_pixels; i += subsampling)
        {
            cols = (i % cdata_left.depth.width) - (cdata_left.depth.width / 2);
            rows = (cdata_left.depth.height / 2) - (i / cdata_left.depth.height);
            // compute axis coordinates according to the camera's coordinate system (Y outwards and Z up)
            Y = depth_array[i] * 1000.f; // we transform measurements to millimeters
            if (Y < 100) continue;
            X = -cols * Y / cdata_left.depth.focalx;
            Z = rows * Y / cdata_left.depth.focalx;
            // transform to virtual camera CS at center of both cameras. Assume equal height (Z). Needs angle and translation
            float XV = coseno * X - seno * Y + h_offset;
            float YV = seno * X + coseno * Y;
            // project on virtual camera
            auto col_virtual = frame_virtual_focalx * XV / YV + center_virtual_i;
            auto row_virtual = frame_virtual_focalx * Z / YV + center_virtual_j;
            if (is_in_bounds<float>(floor(col_virtual), 0, frame_virtual.cols) and is_in_bounds<float>(floor(row_virtual), 0, frame_virtual.rows))
            {
                cv::Vec3b &color = frame_virtual.at<cv::Vec3b>(floor(row_virtual), floor(col_virtual));
                color[0] = rgb_img_data[i * 3]; color[1] = rgb_img_data[i * 3 + 1]; color[2] = rgb_img_data[i * 3 + 2];
            }
            if (is_in_bounds<float>(ceil(col_virtual), 0, frame_virtual.cols) and is_in_bounds<float>(ceil(row_virtual), 0, frame_virtual.rows))
            {
                cv::Vec3b &color = frame_virtual.at<cv::Vec3b>(ceil(row_virtual), ceil(col_virtual));
                color[0] = rgb_img_data[i * 3]; color[1] = rgb_img_data[i * 3 + 1]; color[2] = rgb_img_data[i * 3 + 2];
            }
            if (is_in_bounds<float>(floor(col_virtual), 0, frame_virtual.cols) and is_in_bounds<float>(ceil(row_virtual), 0, frame_virtual.rows))
            {
                cv::Vec3b &color = frame_virtual.at<cv::Vec3b>(ceil(row_virtual), floor(col_virtual));
                color[0] = rgb_img_data[i * 3]; color[1] = rgb_img_data[i * 3 + 1]; color[2] = rgb_img_data[i * 3 + 2];
            }
            if (is_in_bounds<float>(ceil(col_virtual), 0, frame_virtual.cols) and is_in_bounds<float>(floor(row_virtual), 0, frame_virtual.rows))
            {
                cv::Vec3b &color = frame_virtual.at<cv::Vec3b>(floor(row_virtual), ceil(col_virtual));
                color[0] = rgb_img_data[i * 3]; color[1] = rgb_img_data[i * 3 + 1]; color[2] = rgb_img_data[i * 3 + 2];
            }
            // laser computation
            if(Z<-100 or Z>100) continue;
            // accumulate in bins of equal horizontal angle from optical axis
            float hor_angle = atan2(cols, cdata_left.depth.focalx) - M_PI / 6.0 ;
            // map from +-MAX_ANGLE to 0-MAX_LASER_BINS
            int angle_index = (int)((MAX_LASER_BINS/TOTAL_HOR_ANGLE) * hor_angle + (MAX_LASER_BINS/2));
            hor_bins[angle_index].emplace(std::make_tuple(X,Y,Z));
        }
    }
    else
    {
        qWarning() << __FUNCTION__ << " Depth and RGB sizes not equal";
        return std::make_tuple(cv::Mat(), std::vector<LaserPoint>());
    }
    // right image
    if(cdata_right.image.width == cdata_right.depth.width and cdata_right.image.height == cdata_right.depth.height)
    {
        // cast depth
        float *depth_array = (float *) cdata_right.depth.depth.data();
        // cast rgb
        const auto &rgb_img_data = const_cast<std::vector<uint8_t> &>(cdata_right.image.image).data();
        // vars
        float X, Y, Z;
        int cols, rows;
        std::size_t num_pixels = cdata_right.depth.depth.size() / sizeof(float);
        float coseno = cos(M_PI / 6.0);
        float seno = sin(M_PI / 6.0);
        float h_offset = 100;
        for (std::size_t i = 0; i < num_pixels; i += subsampling)
        {
            cols = (i % cdata_right.depth.width) - (cdata_right.depth.width / 2);
            rows = (cdata_right.depth.height / 2) - (i / cdata_right.depth.height);
            // compute axis coordinates according to the camera's coordinate system (Y outwards and Z up)
            Y = depth_array[i] * 1000.f; // we transform measurements to millimeters
            if (Y < 100) continue;
            X = -cols * Y / cdata_right.depth.focalx;
            Z = rows * Y / cdata_right.depth.focalx;
            // transform to virtual camera CS at center of both cameras. Assume equal height (Z). Needs angle and translation
            float XV = coseno * X - seno * Y + h_offset;
            float YV = seno * X + coseno * Y;
            // project on virtual camera
            auto col_virtual = frame_virtual_focalx * XV / YV + center_virtual_i;
            auto row_virtual = frame_virtual_focalx * Z / YV + center_virtual_j;
            if (is_in_bounds<float>(floor(col_virtual), 0, frame_virtual.cols) and is_in_bounds<float>(floor(row_virtual), 0, frame_virtual.rows))
            {
                cv::Vec3b &color = frame_virtual.at<cv::Vec3b>(floor(row_virtual), floor(col_virtual));
                color[0] = rgb_img_data[i * 3]; color[1] = rgb_img_data[i * 3 + 1]; color[2] = rgb_img_data[i * 3 + 2];
            }
            if (is_in_bounds<float>(ceil(col_virtual), 0, frame_virtual.cols) and is_in_bounds<float>(ceil(row_virtual), 0, frame_virtual.rows))
            {
                cv::Vec3b &color = frame_virtual.at<cv::Vec3b>(ceil(row_virtual), ceil(col_virtual));
                color[0] = rgb_img_data[i * 3]; color[1] = rgb_img_data[i * 3 + 1]; color[2] = rgb_img_data[i * 3 + 2];
            }
            if (is_in_bounds<float>(floor(col_virtual), 0, frame_virtual.cols) and is_in_bounds<float>(ceil(row_virtual), 0, frame_virtual.rows))
            {
                cv::Vec3b &color = frame_virtual.at<cv::Vec3b>(ceil(row_virtual), floor(col_virtual));
                color[0] = rgb_img_data[i * 3]; color[1] = rgb_img_data[i * 3 + 1]; color[2] = rgb_img_data[i * 3 + 2];
            }
            if (is_in_bounds<float>(ceil(col_virtual), 0, frame_virtual.cols) and is_in_bounds<float>(floor(row_virtual), 0, frame_virtual.rows))
            {
                cv::Vec3b &color = frame_virtual.at<cv::Vec3b>(floor(row_virtual), ceil(col_virtual));
                color[0] = rgb_img_data[i * 3]; color[1] = rgb_img_data[i * 3 + 1]; color[2] = rgb_img_data[i * 3 + 2];
            }
            // laser computation
            if(Z<-100 or Z>100) continue;
            // accumulate in bins of equal horizontal angle from optical axis
            float hor_angle = atan2(cols, cdata_left.depth.focalx) + M_PI / 6.0;
            // map from +-MAX_ANGLE to 0-MAX_LASER_BINS
            int angle_index = (int)((MAX_LASER_BINS/TOTAL_HOR_ANGLE) * hor_angle + (MAX_LASER_BINS/2));
            hor_bins[angle_index].emplace(std::make_tuple(X,Y,Z));
        }
    }
    else
    {
        qWarning() << __FUNCTION__ << " Depth and RGB sizes not equal";
        return std::make_tuple(cv::Mat(), std::vector<LaserPoint>());
    }
    // Fill gaps
    cv::medianBlur(frame_virtual, frame_virtual, 3);
    mSec duration = MyClock::now() - before;
    before = MyClock::now();   // so it is remembered across QTimer calls to compute()

    // compression
    //vector<int> compression_params{cv::IMWRITE_JPEG_QUALITY, 50};
    //compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    //compression_params.push_back(50);
    //vector<uchar> frame_virtual_encoded;
    //cv::imencode(".jpg", frame_virtual, frame_virtual_encoded, compression_params);
    //duration = myclock::now() - before;
    //std::cout << "Encode took " << duration.count() << "ms" << std::endl;
    //qInfo() << "virtual_frame " << frame_virtual.cols * frame_virtual.rows * 3;
    //qInfo() << "virtual_frame_encoded " << frame_virtual_encoded.size();

    // laser computation
    std::vector<LaserPoint> laser_data(MAX_LASER_BINS);
    uint i=0;
    for(auto &bin : hor_bins)
    {
        if( bin.size() > 0)
        {
            const auto &[X, Y, Z] = *bin.cbegin();
            laser_data[i] = LaserPoint{sqrt(X * X + Y * Y + Z * Z), (i - MAX_LASER_BINS / 2.f) / (MAX_LASER_BINS / TOTAL_HOR_ANGLE)};
        }
        else
            laser_data[i] = LaserPoint{0.f,(i - MAX_LASER_BINS / 2.f) / (MAX_LASER_BINS / TOTAL_HOR_ANGLE)};
        i++;
    }
    auto laser_poly = filter_laser(laser_data);
//    vector<SpecificWorker::LaserPoint> laser_data_filtered;
//    for(auto &l : laser_poly)
//        laser_data_filtered.emplace_back(LaserPoint{(float)l.x(), (float)l.y()});

    //std::generate(laser_data_filtered.begin(), laser_data_filtered.end(), [&laser_poly]() mutable {  auto n = laser_poly.takeFirst(); return LaserPoint{(float)n.x(), (float)n.y()};});
    cv::flip(frame_virtual, frame_virtual, -1);
    return std::make_tuple(frame_virtual, laser_data);
}
cv::Mat SpecificWorker::compute_virtual_frame()
{
    RoboCompCameraRGBDSimple::TImage cdata_virtual;
    cv::Mat virtual_frame;
    try
    {
        cdata_virtual = camerargbdsimple_proxy->getImage("pioneer_camera_virtual");
        this->focalx = cdata_virtual.focalx;
        this->focaly = cdata_virtual.focaly;

        vector<int> compression_params;
        compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
        compression_params.push_back(50);
        //qInfo() <<"1: "<< m.cols*m.rows*3;

        if(!cdata_virtual.image.empty()) {
            cv::imdecode(cdata_virtual.image, 1, &virtual_frame);
            cv::cvtColor(virtual_frame, virtual_frame, cv::COLOR_BGR2RGB);
        }
    }
    catch (const Ice::Exception &e){ std::cout << e.what() << std::endl;}

    return virtual_frame;
}
void SpecificWorker::update_virtual(const cv::Mat &v_image, float focalx, float focaly)
{

    if( auto node = G->get_node(pioneer_camera_virtual_name); node.has_value())
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
        qWarning() << __FUNCTION__ << "Node camera_virtual not found";
}
void SpecificWorker::update_rgbd()
{
    RoboCompCameraRGBDSimple::TImage rgb;
    try
    {
        rgb = camerargbdsimple_proxy->getImage("pioneer_head_camera_0");
    }
    catch (const Ice::Exception &e){ std::cout << e.what() << std::endl;}

    if( auto node = G->get_node(pioneer_head_camera_left_name); node.has_value())
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
void SpecificWorker::read_battery()
{
    try
    {
        auto battery_level = batterystatus_proxy->getBatteryState();
        if( auto battery = G->get_node(battery_name); battery.has_value())
        {
            G->add_or_modify_attrib_local<battery_load_att>(battery.value(), (int)battery_level.percentage);
            G->update_node(battery.value());
        }
    }
    catch(const Ice::Exception &e) { std::cout << e.what() << std::endl;}
}
void SpecificWorker::read_RSSI()
{
    try
    {
        auto rssi = rssistatus_proxy->getRSSIState();
        if( auto wifi = G->get_node(wifi_name); wifi.has_value())
        {
            G->add_or_modify_attrib_local<wifi_signal_att>(wifi.value(), (int)rssi.percentage);
            G->update_node(wifi.value());
        }
    }
    catch(const Ice::Exception &e) { std::cout << e.what() << std::endl;}
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
    catch(const Ice::Exception &e){ std::cout << e.what() <<  __FUNCTION__ << std::endl;};

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
                G->add_or_modify_attrib_local<robot_local_linear_velocity_att>(robot.value(), std::vector<float>{adv_velocity, side_velocity, pose.rz});
                G->update_node(robot.value());
                last_state = pose;
            }
        }
        else  qWarning() << __FUNCTION__ << " No parent found for node " << QString::fromStdString(robot_name);
    }
    else    qWarning() << __FUNCTION__ << " No node " << QString::fromStdString(robot_name);
}

//  Simplify contour with Ramer-Douglas-Peucker and filter out spikes
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
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
/// Asynchronous changes on G nodes from G signals
///////////////////////////////////////////////////////////////////
void SpecificWorker:: add_or_assign_node_slot(std::uint64_t, const std::string &type)
{
    if (type == differentialrobot_type_name)
    {
        if (auto robot = G->get_node(robot_name); robot.has_value())
        {
            // speed
            auto ref_adv_speed = G->get_attrib_by_name<robot_ref_adv_speed_att>(robot.value());
            auto ref_rot_speed = G->get_attrib_by_name<robot_ref_rot_speed_att>(robot.value());
            auto ref_side_speed = G->get_attrib_by_name<robot_ref_side_speed_att>(robot.value());
            if (ref_adv_speed.has_value() and ref_rot_speed.has_value() and ref_side_speed.has_value())
            {
                // Check de values are within robot's accepted range. Read them from config
                //const float lowerA = -10, upperA = 10, lowerR = -10, upperR = 5, lowerS = -10, upperS = 10;
                //std::clamp(ref_adv_speed.value(), lowerA, upperA);
                //std::cout << __FUNCTION__ << ref_side_speed.value() << " "  << ref_adv_speed.value() << " "  << ref_rot_speed.value() << std::endl;
                try
                { differentialrobot_proxy->setSpeedBase(ref_adv_speed.value(), ref_rot_speed.value()); }
                catch (const RoboCompGenericBase::HardwareFailedException &re)
                { std::cout << "Exception setting base speed " << re << '\n'; }
                catch (const Ice::Exception &e)
                { std::cout << e.what() << '\n'; }
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
// this->fullposeestimation_proxy->getFullPose(...)
// this->fullposeestimation_proxy->setInitialPose(...)

/**************************************/
// From the RoboCompFullPoseEstimation you can use this types:
// RoboCompFullPoseEstimation::FullPose

/**************************************/
// From the RoboCompRSSIStatus you can call this methods:
// this->rssistatus_proxy->getRSSIState(...)

/**************************************/
// From the RoboCompRSSIStatus you can use this types:
// RoboCompRSSIStatus::TRSSI

/**************************************/
// From the RoboCompUltrasound you can call this methods:
// this->ultrasound_proxy->getAllSensorDistances(...)
// this->ultrasound_proxy->getAllSensorParams(...)
// this->ultrasound_proxy->getBusParams(...)
// this->ultrasound_proxy->getSensorDistance(...)
// this->ultrasound_proxy->getSensorParams(...)

/**************************************/
// From the RoboCompUltrasound you can use this types:
// RoboCompUltrasound::BusParams
// RoboCompUltrasound::SensorParams

