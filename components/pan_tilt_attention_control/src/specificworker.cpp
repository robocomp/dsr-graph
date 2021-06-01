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
            qFatal("Pan-Tilt-Attention-Control terminate: could not find a camera node");

        // RT APi
        rt_api = G->get_rt_api();

        // own node api
        agent_info_api = std::make_unique<DSR::AgentInfoAPI>(G.get());

        // custom_widget
		graph_viewer->add_custom_widget_to_dock("Pan-Tilt Attention Control", &custom_widget);
		setWindowTitle(QString::fromStdString(agent_name + "-") + QString::number(agent_id));

		// ignore attributes
        G->set_ignored_attributes<laser_angles_att, laser_dists_att, cam_depth_att>();

        // local UI
        // Initialize combobox
        initialize_combobox();
        connect(custom_widget.comboBox, SIGNAL(activated(int)), this, SLOT(change_attention_object_slot(int)));
        connect(custom_widget.stopButton, SIGNAL(clicked(bool)), this, SLOT(stop_button_slot(bool)));
        connect(custom_widget.resetButton, SIGNAL(clicked()), this, SLOT(reset_button_slot()));
        connect(custom_widget.clearButton, SIGNAL(clicked()), this, SLOT(clear_button_slot()));
        connect(custom_widget.panSpinBox, SIGNAL(valueChanged(double)), this, SLOT(pan_slot(double)));
        connect(custom_widget.tiltSpinBox, SIGNAL(valueChanged(double)), this, SLOT(tilt_slot(double)));


        // set nose to default position
        set_nose_target_to_default();

        // check existing attention_action nodes
        auto camera_node = G->get_node(cam_api->get_id());
        for( const auto nodes = G->get_node_edges_by_type(camera_node.value(), attention_action_type_name); const auto &e : nodes)
            if( auto to_node = G->get_node(e.to()); to_node.has_value())
                //this->set_of_objects_to_attend_to.push_back(to_node.value().id());
                target_buffer.put(to_node.value().id());

        this->Period = 50;
        timer.start(this->Period);
	}
}

// slot updating will keep an updated list of edges "attention_to" from "camera_pan_tilt" to some object
// here the list is analyzed and a next_object is selected to change/keep current attention
void SpecificWorker::compute()
{
    static auto before = my_clock::now();
    //static bool first_time = true;
    static std::uint64_t current_target;
    static std::string target_name;

    if(const auto current_target_o = target_buffer.try_get(); current_target_o.has_value())
    {
        current_target = current_target_o.value();
        if (auto target_o = G->get_node(current_target); target_o.has_value())
        {
            target_name = target_o.value().name();
            qInfo() << __FUNCTION__ << " New target arrived: " << current_target << " - " << QString::fromStdString(target_name);
        }
    }
    const auto g_image_o = rgb_buffer.try_get();
    //const auto g_depth = depth_buffer.try_get();
    if (g_image_o.has_value())
    {
        auto &[g_image, cam_timestamp] = g_image_o.value();
        show_image(g_image, target_name, cam_timestamp);
        if(this->active)
        {
            if(this->wait_state.waiting())  return;
            if (auto target_o = G->get_node(current_target); target_o.has_value())
            {
                const auto target = target_o.value();
                if (auto pan_tilt = G->get_node(viriato_head_camera_pan_tilt_name); pan_tilt.has_value())
                {
                    // compute coordinates of target in camera's reference system. Should be zero if centered
                    if(auto target_in_camera_o = inner_eigen->transform(viriato_head_camera_name, target.name(), cam_timestamp); target_in_camera_o.has_value())
                    {
                        const Eigen::Vector3d &target_in_camera = target_in_camera_o.value();
                        const Eigen::Vector2d target_in_camera_2d{target_in_camera.x(), target_in_camera.z()};  // Z points upwards as -Y in image plane
                        float dist = target_in_camera_2d.norm();
                        if (dist > CONSTANTS.max_distance_between_target_and_pan_tilt)
                        {
                            // saccade to G position
                            if(auto target_in_pan_tilt_o = inner_eigen->transform(viriato_head_camera_pan_tilt_name, target.name(), cam_timestamp); target_in_pan_tilt_o.has_value())
                            {
                                const Eigen::Vector3d &target_in_pan_tilt = target_in_pan_tilt_o.value();

                                std::vector<float> target_v{(float)target_in_pan_tilt.x(), (float)target_in_pan_tilt.y(), (float)target_in_pan_tilt.z()};

                                G->add_or_modify_attrib_local<viriato_head_pan_tilt_nose_target_att>(pan_tilt.value(), target_v);
                                G->update_node(pan_tilt.value());
                                print_data(target, dist, target_in_camera, cam_timestamp, pan_tilt.value(), true);
                                this->wait_state.init(1000); //ms   learn this time dynamically
                            }
                        }
                        else
                            print_data(target, dist, target_in_camera, cam_timestamp, pan_tilt.value(), false);
                    }
                }
                // else
                // compute reference speed for pan-tilt
            }
            else qWarning() << __FUNCTION__ << " No node " << QString::fromStdString(target_name) << " found in G";
        }
        else {} //qWarning() << __FUNCTION__ << " No active ";
    }
    else {} //qWarning() << __FUNCTION__ << " No image ready";
    fps.print("FPS: ", [this](auto x){ graph_viewer->set_external_hz(x);});
}
void SpecificWorker::print_data(const DSR::Node &target, int error, const Eigen::Vector3d &target_in_camera,
                                std::uint64_t cam_timestamp, const DSR::Node &pan_tilt, bool saccade)
{
    custom_widget.text_pane->clear();
    auto target_w = inner_eigen->transform(world_name, target.name(), cam_timestamp).value();
    auto camera_tip = G->get_attrib_by_name<viriato_head_pan_tilt_nose_target_att>(pan_tilt).value().get();
    auto camera_tip_w = inner_eigen->transform(world_name, Eigen::Vector3d{camera_tip[0],camera_tip[1],camera_tip[2]}, viriato_head_camera_pan_tilt_name , cam_timestamp).value();
    QString output;
    auto w_error = (target_w - camera_tip_w).norm();
    if(saccade)
        output = " Saccade to target: ";
    else
        output = " Gaze at target: ";
    output +=        QString::fromStdString(target.name())
                     + "\n  Error in camera: " + QString::number(error) + " mm. Threshold: " + QString::number(CONSTANTS.max_distance_between_target_and_pan_tilt)
                     + "\n  Error in world: " +  QString::number(w_error) + " mm" +
                     + "\n  Target in camera: \n    [" + QString::number(target_in_camera.x()) + "   " + QString::number(target_in_camera.y()) + " " + QString::number(target_in_camera.z()) + "]"
                     + "\n  Target in world: \n    [" + QString::number(target_w.x()) + "   " + QString::number(target_w.y()) + "    " + QString::number(target_w.z()) + "]"
                     + "\n  Tip in world: \n    [" + QString::number(camera_tip_w.x()) + "  " + QString::number(camera_tip_w.y()) + "   " + QString::number(camera_tip_w.z()) + "]";

    custom_widget.text_pane->setPlainText(output);
    //qInfo() << __FUNCTION__ << " " << output;
}
void SpecificWorker::set_nose_target_to_default()
{
    qInfo() << __FUNCTION__;
    //if(this->already_in_default == true) return;
    if(auto pan_tilt = G->get_node(viriato_head_camera_pan_tilt_name); pan_tilt.has_value())
    {
        G->add_or_modify_attrib_local<viriato_head_pan_tilt_nose_target_att>(pan_tilt.value(), nose_default_pose);
        G->update_node(pan_tilt.value());
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
        auto pose = inner_eigen->transform(viriato_head_camera_pan_tilt_name, object_of_interest);
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
void SpecificWorker::show_image(cv::Mat image, const std::string &target_name, std::uint64_t timestamp)
{
    int width = cam_api->get_width();
    int height = cam_api->get_height();
    if(const auto target = G->get_node(target_name); target.has_value())
    {
        const auto &[left, top, right, bottom] = project_target_on_image(target.value(), timestamp);
        cv::rectangle(image, cv::Point((int) left + width / 2, (int) top + height / 2),
                      cv::Point((int) right + width / 2, (int) bottom + height / 2), cv::Scalar(0, 255, 0), 4);

    }
    cv::drawMarker(image, cv::Point(image.cols/2, image.rows/2),  cv::Scalar(0, 228, 100), cv::MARKER_CROSS, 60, 2);
    auto pix = QPixmap::fromImage(QImage(image.data, image.cols, image.rows, QImage::Format_RGB888)).scaled(custom_widget.rgb_image->width(), custom_widget.rgb_image->height());
    custom_widget.rgb_image->setPixmap(pix);
}
std::tuple<int, int, int, int> SpecificWorker::project_target_on_image(const DSR::Node &target, std::uint64_t timestamp)
{
        auto w_attr = G->get_attrib_by_name<obj_width_att>(target);
        float w = w_attr.value();
        auto h_attr = G->get_attrib_by_name<obj_height_att>(target);
        float h = h_attr.value();
        auto d_attr = G->get_attrib_by_name<obj_depth_att>(target);
        float d = d_attr.value();
        
        if(auto pos_wrt_camera = inner_eigen->transform(viriato_head_camera_name, target.name(), timestamp); pos_wrt_camera.has_value() and pos_wrt_camera.value().y() > 0)
        {
            // project corners of object's bounding box in the camera image plane
            std::vector<Mat::Vector2d> bb_in_camera(8);
            bb_in_camera[0] = cam_api->project(
                    inner_eigen->transform(viriato_head_camera_name, Eigen::Vector3d(w / 2, d / 2, -h / 2), target.name(), timestamp).value(), 0, 0);
            bb_in_camera[1] = cam_api->project(
                    inner_eigen->transform(viriato_head_camera_name, Eigen::Vector3d(-w / 2, d / 2, -h / 2), target.name(), timestamp).value(), 0, 0);
            bb_in_camera[2] = cam_api->project(
                    inner_eigen->transform(viriato_head_camera_name, Eigen::Vector3d(w / 2, -d / 2, -h / 2), target.name(), timestamp).value(), 0, 0);
            bb_in_camera[3] = cam_api->project(
                    inner_eigen->transform(viriato_head_camera_name, Eigen::Vector3d(-w / 2, -d / 2, -h / 2), target.name(), timestamp).value(), 0, 0);
            bb_in_camera[4] = cam_api->project(
                    inner_eigen->transform(viriato_head_camera_name, Eigen::Vector3d(w / 2, d / 2, h / 2), target.name(), timestamp).value(), 0, 0);
            bb_in_camera[5] = cam_api->project(
                    inner_eigen->transform(viriato_head_camera_name, Eigen::Vector3d(-w / 2, d / 2, h / 2), target.name(), timestamp).value(), 0, 0);
            bb_in_camera[6] = cam_api->project(
                    inner_eigen->transform(viriato_head_camera_name, Eigen::Vector3d(w / 2, -d / 2, h / 2), target.name(), timestamp).value(), 0, 0);
            bb_in_camera[7] = cam_api->project(
                    inner_eigen->transform(viriato_head_camera_name, Eigen::Vector3d(-w / 2, -d / 2, h / 2), target.name(), timestamp).value(), 0, 0);

            // Compute a 2D projected bounding box
            auto xExtremes = std::minmax_element(bb_in_camera.begin(), bb_in_camera.end(),
                                                 [](const Mat::Vector2d &lhs, const Mat::Vector2d &rhs) {
                                                     return lhs.x() < rhs.x();
                                                 });
            auto yExtremes = std::minmax_element(bb_in_camera.begin(), bb_in_camera.end(),
                                                 [](const Mat::Vector2d &lhs, const Mat::Vector2d &rhs) {
                                                     return lhs.y() < rhs.y();
                                                 });
            // Take the most separated ends to build the rectangle
            return std::make_tuple(xExtremes.first->x(), yExtremes.first->y(), xExtremes.second->x(), yExtremes.second->y());
        }
        else return std::make_tuple(0,0,0,0);
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
                                   [this, cam_node](const std::vector<std::uint8_t> &in, std::tuple<cv::Mat, std::uint64_t> &out)
                                   {
                                        auto image = cv::Mat(cam_api->get_height(), cam_api->get_width(), CV_8UC3, const_cast<std::vector<uint8_t> &>(in).data());
                                        if( auto timestamp = G->get_attrib_timestamp<cam_rgb_att>(cam_node.value()); timestamp.has_value())
                                           out = std::make_tuple(image, timestamp.value()/1000000);  // cambiar cuando venga en millio !!!!!!!!!!
                                        else
                                           out = std::make_tuple(image, 0);
                                   });
               }
//            if (auto g_depth = G->get_attrib_by_name<cam_depth_att>(cam_node.value()); g_depth.has_value())
//            {
//                float *depth_array = (float *) g_depth.value().get().data();
//                std::vector<float> res{depth_array, depth_array + g_depth.value().get().size() /sizeof(float) };
//                depth_buffer.put(std::move(res));
//            }
        }
        else  qWarning() << __FUNCTION__ << "No camera_node found in G";
    }
}
void SpecificWorker::add_or_assign_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type)
{
    if (type == attention_action_type_name)
    {
        if(auto cam_node = G->get_node(from); cam_node.has_value() and (cam_node.value().id() == cam_api->get_id()))
        {
            if( auto object_node = G->get_node(to); object_node.has_value())
            {
                //this->set_of_objects_to_attend_to.push_back(object_node.value().id());
                target_buffer.put(object_node.value().id());
                //qInfo() << __FUNCTION__ << " Node id stored " << object_node.value().id();
            }
        }
    }
}
///////////////////////////////////////////////////////////
void SpecificWorker::stop_button_slot(bool state)
{
    qInfo() << __FUNCTION__ << "In slot " << state;
    if(state)
    {
        this->active = false;
        custom_widget.stopButton->setText("Start");
    }
    else
    {
        this->active = true;
        custom_widget.stopButton->setText("Stop");
    }
}
void SpecificWorker::reset_button_slot()
{
    qInfo() << __FUNCTION__;
    set_nose_target_to_default();
    this->active = false;
}
void SpecificWorker::clear_button_slot()
{
    qInfo() << __FUNCTION__;
    this->active = false;
    // NEED A buffer->clear() method here
}
void SpecificWorker::change_attention_object_slot(int index)
{
    std::string node_name = custom_widget.comboBox->itemText(index).toStdString();
    if( auto node = G->get_node(node_name); node.has_value())
    {
        qInfo() << __FUNCTION__ << " " << index << " " << QString::fromStdString(node_name);
        target_buffer.put(node.value().id());
    }
    else
        qWarning() << __FUNCTION__ << "No node " << QString::fromStdString(node_name) << "found";

}
void SpecificWorker::initialize_combobox()
{
    custom_widget.comboBox->clear();
    for (const auto &[k, v] : known_object_types)
    {
        auto nodes = G->get_nodes_by_type(k);
        for (const auto &node : nodes)
        {
            QVariant data;
            data.setValue(node.name());
            custom_widget.comboBox->addItem(QString::fromStdString(node.name()), data);
        }
    }
}
void SpecificWorker::pan_slot(double)
{
//    if (auto pan_tilt = G->get_node(viriato_head_camera_pan_tilt_name); pan_tilt.has_value())
//    {
//        G->add_or_modify_attrib_local<viriato_head_pan_tilt_nose_target_att>(pan_tilt.value(), std::vector<float>);
//        G->update_node(pan_tilt.value());
//    }
}
void SpecificWorker::tilt_slot(double)
{
//    if (auto pan_tilt = G->get_node(viriato_head_camera_pan_tilt_name); pan_tilt.has_value())
//    {
//        G->add_or_modify_attrib_local<viriato_head_pan_tilt_nose_target_att>(pan_tilt.value(), std::vector<float>);
//        G->update_node(pan_tilt.value());
//    }
}
///////////////////////////////////////////////////////////
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
