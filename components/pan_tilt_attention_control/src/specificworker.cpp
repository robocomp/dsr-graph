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
#include <ranges>
#include <algorithm>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
    QLoggingCategory::setFilterRules("*.debug=false\n");
	this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	//G->write_to_json_file("./"+agent_name+".json");
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

	cfg_file = params["cfg_file"].value;
    weights_file = params["weight_file"].value;
	names_file = params["names_file"].value;

	return true;
}

void SpecificWorker::initialize(int period)
{
    std::cout << __FUNCTION__ << std::endl;
    this->Period = period;
    if (this->startup_check_flag)
        this->startup_check();
    else
    {
        // create graph
        G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id); // Init nodes
        std::cout << __FUNCTION__ << " Graph loaded" << std::endl;

        //dsr update signals
        connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::add_or_assign_node_slot);
        connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::add_or_assign_edge_slot);
        //connect(G.get(), &DSR::DSRGraph::update_attrs_signal, this, &SpecificWorker::add_or_assign_attrs_slot);
        //connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
        //connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);

        // Graph viewer
        using opts = DSR::DSRViewer::view;
        int current_opts = 0;
        opts main = opts::none;
        if (tree_view)
            current_opts = current_opts | opts::tree;
        if (graph_view)
        {
            current_opts = current_opts | opts::graph;
            main = opts::graph;
        }
        if (qscene_2d_view)
            current_opts = current_opts | opts::scene;
        if (osg_3d_view)
            current_opts = current_opts | opts::osg;

        graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
        setWindowTitle(QString::fromStdString(agent_name + "-") + QString::number(agent_id));

        // Inner Api
        inner_eigen = G->get_inner_eigen_api();

        // get camera_api
        if(auto cam_node = G->get_node(camera_name); cam_node.has_value())
            cam_api = G->get_camera_api(cam_node.value());
        else
            qFatal("YoloV4_tracker terminate: could not find a camera node");

        // RT APi
        rt_api = G->get_rt_api();

        // get camera_api
        if (auto cam_node = G->get_node(camera_name); cam_node.has_value())
            cam_api = G->get_camera_api(cam_node.value());
        else
            qFatal("YoloV4_tracker terminate: could not find a camera node");

		// custom_widget
		graph_viewer->add_custom_widget_to_dock("YoloV4-tracker", &custom_widget);
		setWindowTitle(QString::fromStdString(agent_name + "-") + QString::number(agent_id));

		// ignore attributes
        G->set_ignored_attributes<laser_angles_att, laser_dists_att>();

        connect(custom_widget.startButton, SIGNAL(clicked(bool)), this, SLOT(start_button_slot(bool)));

        this->Period = 100;
        timer.start(Period);
	}
}

// slot updating will keep an updated list of edges "attention_to" from "camera_pan_tilt" to some object
// here the list is analyzed and a next_object is selected to change/keep current attention
void SpecificWorker::compute()
{
    static auto before = myclock::now();
    static bool first_time = true;
    const auto g_image = rgb_buffer.try_get();
    const auto g_depth = depth_buffer.try_get();
    auto robot = G->get_node(robot_name);
    if (robot.has_value() and g_image.has_value() and g_depth.has_value())
    {
        show_image(g_image.value());
        for( const auto &el : set_of_objects_to_attend_to)
            std::cout << el << std::endl;
        std::cout << "------------------------" << std::endl;


//        if( time_to_change )       // if finished with current object, pick a new object from G -> update
//        {
//            auto glass_nodes = G->get_nodes_by_type(glass_type_name);
//            remove_edge();
//            object_of_interest = random_selector(glass_nodes).name();
//            auto index = custom_widget.comboBox->findText(QString::fromStdString(object_of_interest));
//            custom_widget.comboBox->setCurrentIndex(index);
//            time_to_change = false;
//            std::cout << __FUNCTION__ << " " << object_of_interest << std::endl;
//        }
//        this->depth_array = g_depth.value();
//        cv::Mat imgyolo = g_image.value();
//        std::vector<Box> real_objects = process_image_with_yolo(imgyolo);
//        std::vector<Box> synth_objects = process_graph_with_yolosynth({object_of_interest});
//        show_image(imgyolo, real_objects, synth_objects);
//        if( tracking )
//        {
//            track_object_of_interest(robot.value());
//            if(auto res = std::ranges::find_if(real_objects, [](auto &a){return(a.name == "cup");}); res != real_objects.end())
//                compute_prediction_error(*res, synth_objects.front());
//
//            // check end of object cycle
//            const auto link = G->get_edge(camera_name, object_of_interest, "looking-at");
//            const auto r_angle = inner_eigen->transform_axis(robot_name, viriato_head_camera_pan_joint).value();
//            const auto dist = inner_eigen->transform_axis(camera_name, object_of_interest).value().norm();
//
//            //std::cout << __FUNCTION__ << " Tracking: " << link.has_value() << " " << fabs(r_angle[5]) << " " << elapsed.count() << std::endl;
//            if( link.has_value() and (fabs(r_angle[5])<0.04 or dist < 1300))
//            {
//                stop_robot();
//                if( first_time )
//                {
//                    before = myclock::now();
//                    first_time = false;
//                }
//                msec duration = myclock::now() - before;
//                if(duration.count() > 2000)
//                {
//                    time_to_change = true;
//                    std::cout << __FUNCTION__ << " object completed" << std::endl;
//                }
//                else std::cout << __FUNCTION__ << " Watching" << std::endl;
//            }
//            else first_time = true;
//        }
//        else
//            set_nose_target_to_default();

//        qInfo() << real_objects.size() << synth_objects.size();
//        for(auto s : synth_objects)  qInfo() << QString::fromStdString(s.name) << s.bot << s.top << s.left << s.right;
//        for(auto s : real_objects)  qInfo() << QString::fromStdString(s.name) << s.bot << s.top << s.left << s.right << s.prob;
    }
}
void SpecificWorker::set_nose_target_to_default()
{
    //if(this->already_in_default == true) return;
    if(auto pan_tilt = G->get_node(viriato_head_camera_pan_tilt); pan_tilt.has_value())
    {
        G->add_or_modify_attrib_local<viriato_head_pan_tilt_nose_target_att>(pan_tilt.value(), nose_default_pose);
        G->update_node(pan_tilt.value());
        //this->already_in_default = true;
    }
}
void SpecificWorker::track_object_of_interest(DSR::Node &robot)
{
    static Eigen::Vector3d ant_pose;
    auto object = G->get_node(object_of_interest);
    auto pan_tilt = G->get_node(viriato_pan_tilt);
    if(object.has_value() and pan_tilt.has_value())
    {
        // get object pose in world coordinate frame
        auto po = inner_eigen->transform(world_name, object_of_interest);
        auto pose = inner_eigen->transform(viriato_head_camera_pan_tilt, object_of_interest);
        // pan-tilt center
        if (po.has_value() and pose.has_value() /*and ((pose.value() - ant_pose).cwiseAbs2().sum() > 10)*/)   // OJO AL PASAR A METROS
        {
//            G->add_or_modify_attrib_local<viriato_head_pan_tilt_nose_target_att>(pan_tilt.value(), std::vector<float>{(float)po.value().x(), (float)po.value().y(), (float)po.value().z()});
            G->add_or_modify_attrib_local<viriato_head_pan_tilt_nose_target_att>(pan_tilt.value(), std::vector<float>{(float)pose.value().x(), (float)pose.value().y(), (float)pose.value().z()});
            G->update_node(pan_tilt.value());
            //qInfo() <<"NOW ...." << pose.value().x() << pose.value().y() << pose.value().z();
        }
        ant_pose = pose.value();
    }
    else
        qWarning() << __FILE__ << __FUNCTION__ << "No object of interest " << QString::fromStdString(object_of_interest) << "found in G";
}
void SpecificWorker::show_image(cv::Mat image)
{
    auto qimage = QImage(image.data, cam_api->get_width(), cam_api->get_height(), QImage::Format_RGB888).scaled(custom_widget.width(), custom_widget.height(), Qt::KeepAspectRatioByExpanding);;
    auto pix = QPixmap::fromImage(qimage);
    custom_widget.rgb_image->setPixmap(pix);
}
///////////////////////////////////////////////////////////////////
/// Asynchronous changes on G nodes from G signals
///////////////////////////////////////////////////////////////////
void SpecificWorker::add_or_assign_node_slot(std::uint64_t id, const std::string &type)
{
    if (type == rgbd_type_name and id == cam_api->get_id())
    {
        if(auto cam_node = G->get_node(id); cam_node.has_value())
        {
            if (const auto g_image = G->get_attrib_by_name<cam_rgb_att>(cam_node.value()); g_image.has_value())
                {
                    rgb_buffer.put(std::vector<uint8_t>(g_image.value().get().begin(), g_image.value().get().end()),
                                   [this](const std::vector<std::uint8_t> &in, cv::Mat &out) {
                                       out = cv::Mat(cam_api->get_height(), cam_api->get_width(), CV_8UC3,
                                                   const_cast<std::vector<uint8_t> &>(in).data());
                                   });
               }
            if (auto g_depth = G->get_attrib_by_name<cam_depth_att>(cam_node.value()); g_depth.has_value())
            {
                float *depth_array = (float *) g_depth.value().get().data();
                std::vector<float> res{depth_array, depth_array + g_depth.value().get().size() /sizeof(float) };
                depth_buffer.put(std::move(res));
            }
        }
        else  qWarning() << __FUNCTION__ << "No camera_node found in G";
    }
}
void SpecificWorker::add_or_assign_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type)
{
    if (type == attention_action_type_name)
    {
        if(auto cam_node = G->get_node(from); cam_node.has_value() and cam_node.value().id() == cam_api->get_id())
        {
            if( auto object_node = G->get_node(to); object_node.has_value())
            this->set_of_objects_to_attend_to.insert(object_node.value().id());
        }
    }
}
//////////////////////////////////////////////////////77777
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

/**************************************/
// From the RoboCompDSRGetID you can call this methods:
// this->dsrgetid_proxy->getID(...)

/**************************************/
// From the RoboCompDSRGetID you can call this methods:
// this->dsrgetid1_proxy->getID(...)
