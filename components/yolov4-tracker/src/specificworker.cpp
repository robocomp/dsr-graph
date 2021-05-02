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
	G->write_to_json_file("./"+agent_name+".json");
    delete ynets[0]; // deallocate YOLOv4 network
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

    // initialize YOLOv4 network instances
    for(uint i=0; i<YOLO_INSTANCES; ++i)
    {
        ynets.push_back(init_detector());
    }
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
        connect(G.get(), &DSR::DSRGraph::update_attrs_signal, this, &SpecificWorker::add_or_assign_attrs_slot);
        connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
        connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);

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

        //Inner Api
        inner_eigen = G->get_inner_eigen_api();

        // get camera_api
        if(auto cam_node = G->get_node(camera_name); cam_node.has_value())
            cam_api = G->get_camera_api(cam_node.value());
        else
            qFatal("YoloV4_tracker terminate: could not find a camera node");

        //RT APi
        rt_api = G->get_rt_api();

		// custom_widget
		graph_viewer->add_custom_widget_to_dock("YoloV4-tracker", &custom_widget);
		setWindowTitle(QString::fromStdString(agent_name + "-") + QString::number(agent_id));

		// ignore attributes
        G->set_ignored_attributes<laser_angles_att, laser_dists_att>();

        // Initialize combobox
        auto glass_nodes = G->get_nodes_by_type(glass_type_name);
        for(const auto &node : glass_nodes)
        {
            QVariant data;
            data.setValue(node.name());
            custom_widget.comboBox->addItem(QString::fromStdString(node.name()), data);
        }
        connect(custom_widget.startButton, SIGNAL(clicked(bool)), this, SLOT(start_button_slot(bool)));
        connect(custom_widget.comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(change_object_slot(int)));

        this->Period = 100;
        timer.start(Period);
        READY_TO_GO = true;
	}
}

void SpecificWorker::compute()
{
    static auto before = myclock::now();
    static bool first_time = true;
    const auto g_image = rgb_buffer.try_get();
    const auto g_depth = depth_buffer.try_get();
    auto robot = G->get_node(robot_name);
    if (robot.has_value() and g_image.has_value() and g_depth.has_value())
    {
        if( time_to_change )       // if finished with current object, pick a new object from G -> update
        {
            auto glass_nodes = G->get_nodes_by_type(glass_type_name);
            remove_edge();
            object_of_interest = random_selector(glass_nodes).name();
            auto index = custom_widget.comboBox->findText(QString::fromStdString(object_of_interest));
            custom_widget.comboBox->setCurrentIndex(index);
            time_to_change = false;
            std::cout << __FUNCTION__ << " " << object_of_interest << std::endl;
        }
        this->depth_array = g_depth.value();
        cv::Mat imgyolo = g_image.value();
        std::vector<Box> real_objects = process_image_with_yolo(imgyolo);
        std::vector<Box> synth_objects = process_graph_with_yolosynth({object_of_interest});
        show_image(imgyolo, real_objects, synth_objects);
        if( tracking )
        {
            track_object_of_interest(robot.value());
            if(auto res = std::ranges::find_if(real_objects, [](auto &a){return(a.name == "cup");}); res != real_objects.end())
                compute_prediction_error(*res, synth_objects.front());

            // check end of object cycle
            const auto link = G->get_edge(camera_name, object_of_interest, "looking-at");
            const auto r_angle = inner_eigen->transform_axis(robot_name, viriato_head_camera_pan_joint).value();
            const auto dist = inner_eigen->transform_axis(camera_name, object_of_interest).value().norm();

            //std::cout << __FUNCTION__ << " Tracking: " << link.has_value() << " " << fabs(r_angle[5]) << " " << elapsed.count() << std::endl;
            if( link.has_value() and (fabs(r_angle[5])<0.04 or dist < 1300))
            {
                stop_robot();
                if( first_time )
                {
                    before = myclock::now();
                    first_time = false;
                }
                msec duration = myclock::now() - before;
                if(duration.count() > 2000)
                {
                    time_to_change = true;
                    std::cout << __FUNCTION__ << " object completed" << std::endl;
                }
                else std::cout << __FUNCTION__ << " Watching" << std::endl;
            }
            else first_time = true;
        }
        else
            set_nose_target_to_default();

//        qInfo() << real_objects.size() << synth_objects.size();
//        for(auto s : synth_objects)  qInfo() << QString::fromStdString(s.name) << s.bot << s.top << s.left << s.right;
//        for(auto s : real_objects)  qInfo() << QString::fromStdString(s.name) << s.bot << s.top << s.left << s.right << s.prob;
    }
}
Detector* SpecificWorker::init_detector()
{
    // read objects names from file
    std::ifstream file(names_file);
    for(std::string line; getline(file, line);) names.push_back(line);
    // initialize YOLOv4 detector
    Detector* detector = new Detector(cfg_file, weights_file);
    return detector;
}
std::vector<SpecificWorker::Box> SpecificWorker::process_image_with_yolo(const cv::Mat &img)
{
    // get detections from RGB image
    image_t yolo_img = createImage(img);
    std::vector<bbox_t> detections = ynets[0]->detect(yolo_img, 0.2, false);
    // process detected bounding boxes
    std::vector<Box> bboxes(detections.size());
    for(unsigned int i = 0; i < detections.size(); i++)
    {
        const auto &d = detections[i];
        int cls = d.obj_id;
        int left  = d.x;
        int right = d.x + d.w;
        int top   = d.y;
        int bot   = d.y + d.h;
        if(left < 0) left = 0;
        if(right > yolo_img.w-1) right = yolo_img.w-1;
        if(top < 0) top = 0;
        if(bot > yolo_img.h-1) bot = yolo_img.h-1;
        float prob = d.prob;
        bboxes[i] = Box{names.at(cls), left, top, right, bot, prob*100, 0};
    }
    qDebug() << __FILE__ << __FUNCTION__ << "LABELS " << bboxes.size();
    ynets[0]->free_image(yolo_img);
    return bboxes;
}
std::vector<SpecificWorker::Box> SpecificWorker::process_graph_with_yolosynth(const std::vector<std::string> &object_names)
{
    std::vector<Box> synth_box;
    //320/np.tan(np.deg2rad(30))
    auto c = YOLO_IMG_SIZE/2;
    for(auto &&object_name : object_names)
    {
        //get object from G
        if (auto object = G->get_node(object_name); object.has_value())
        {
            // project corners of object's bounding box in the camera image plane
            std::vector<Mat::Vector2d> bb_in_camera(8);
            const float h = 150;
            bb_in_camera[0] = cam_api->project(inner_eigen->transform(camera_name, Mat::Vector3d(40,40,0), object_name).value(),c,c);
            bb_in_camera[1] = cam_api->project(inner_eigen->transform(camera_name, Eigen::Vector3d(-40,40,0), object_name).value(),c,c);
            bb_in_camera[2] = cam_api->project(inner_eigen->transform(camera_name, Eigen::Vector3d(40,-40,0), object_name).value(),c,c);
            bb_in_camera[3] = cam_api->project(inner_eigen->transform(camera_name, Eigen::Vector3d(-40,-40,0), object_name).value(),c,c);
            bb_in_camera[4] = cam_api->project(inner_eigen->transform(camera_name, Eigen::Vector3d(40, 40, h), object_name).value(),c,c);
            bb_in_camera[5] = cam_api->project(inner_eigen->transform(camera_name, Eigen::Vector3d(-40,40, h), object_name).value(),c,c);
            bb_in_camera[6] = cam_api->project(inner_eigen->transform(camera_name, Eigen::Vector3d(40, -40, h), object_name).value(),c,c);
            bb_in_camera[7] = cam_api->project(inner_eigen->transform(camera_name, Eigen::Vector3d(-40, -40,h), object_name).value(),c,c);

            // Compute a 2D bounding box
            auto xExtremes = std::minmax_element(bb_in_camera.begin(), bb_in_camera.end(),
                                                 [](const Mat::Vector2d & lhs, const Mat::Vector2d& rhs) {
                                                     return lhs.x() < rhs.x();
                                                 });
            auto yExtremes = std::minmax_element(bb_in_camera.begin(), bb_in_camera.end(),
                                                 [](const Mat::Vector2d& lhs, const Mat::Vector2d& rhs) {
                                                     return lhs.y() < rhs.y();
                                                 });
            // Take the most separated ends to build the rectangle
            Box box;
            box.left = xExtremes.first->x();
            box.top = yExtremes.first->y();
            box.right = xExtremes.second->x();
            box.bot = yExtremes.second->y();
            box.prob = 100;
            box.name = object_name;
            if( auto d = rt_api->get_translation(cam_api->get_id(), object.value().id()); d.has_value())
                box.depth = d.value().norm();
            else box.depth = 0;
            synth_box.push_back(box);
        }
    }
    return synth_box;
}
void SpecificWorker::compute_prediction_error(Box &real_box, const Box &synth_box)
{
    auto are_close = [](const Box &b1, const Box &b2)
                     {
                        //A rectangle with the real object is created
                        QRect r(QPoint(b1.top, b1.left), QSize(b1.width(), b1.height()));
                        //A rectangle with the sythetic object is created
                        QRect rs(QPoint(b2.top, b2.left), QSize(b2.width(), b2.height()));
                        //Compute intersection percentage between synthetic and real
                        QRect i = rs.intersected(r);
                        //The area is normalized between 0 and 1 dividing by the minimum between both areas of each object
                        float area = (float)(i.width() * i.height()) / std::min(rs.width() * rs.height(), r.width() * r.height());
                        //The displacement vector between the two images is calculated
                        QPoint error = r.center() - rs.center();
                        // If the area is 0 there is no intersection
                        // If the error is less than twice the width of the synthetic rectangle
                        if(area > 0 or error.manhattanLength() < rs.width()*3) //ADD DEPTH CHECK
                        {
                            //qInfo() << __FUNCTION__ << r << rs << area << error.manhattanLength() << rs.width()*3;
                            return true;
                        }
                        else
                            return false;
                     };

    const auto &r = real_box;
    auto tp = cam_api->get_roi_depth(this->depth_array, Eigen::AlignedBox<float, 2>(Eigen::Vector2f(r.left, r.bot), Eigen::Vector2f(r.right, r.top)));
    if (tp.has_value())
    {
        auto [x,y,z] = tp.value();
        real_box.depth = sqrt(x*x+y*y+z*z);
        if (are_close(real_box, synth_box))
                add_edge(tp.value());
    }
}
void SpecificWorker::add_edge(const std::tuple<float,float,float> &tp)
{
    if (auto object = G->get_node(object_of_interest); object.has_value())
    {
        //DSR::Edge edge(object.value().id(), cam_api->get_id(), "looking-at", agent_id);
        auto edge = DSR::Edge::create<looking_at_edge_type>(object.value().id(), cam_api->get_id());
        auto &[x, y, z] = tp;
        G->add_attrib_local<looking_at_translation_att>(edge, std::vector<float>{x, y, z});
        G->add_attrib_local<looking_at_rotation_euler_xyz_att>(edge, std::vector<float>{0.f, 0.f, 0.f});
        if (G->insert_or_assign_edge(edge))
        {
//            if (const auto loop = inner_eigen->transform(viriato_head_camera_name, object_of_interest); loop.has_value())
//                std::cout << __FUNCTION__ << " [" << x << " " << y << " " << z << "] - Loop [" << loop.value().x()
//                          << " " << loop.value().y() << " " << loop.value().z() << "]" << std::endl;
        }
        else
            std::cout << __FUNCTION__ << "WARNING: Error inserting new edge: " << cam_api->get_id() << "->"
                  << object.value().id() << " type: has" << std::endl;
    }
}
void SpecificWorker::remove_edge()
{
    if (auto object = G->get_node(object_of_interest); object.has_value())
        if (not G->delete_edge(cam_api->get_id(), object.value().id(), "looking-at"))
            qWarning() << "Edge from camera to object_of_interest could not be deleted";
}
void SpecificWorker::change_to_new_target()
{
    auto index = custom_widget.comboBox->currentIndex();
    remove_edge();
    object_of_interest = custom_widget.comboBox->itemData(index).value<std::string>();
    qInfo() << __FUNCTION__ << "Changed object of interest to " << QString::fromStdString(object_of_interest);
    this->tracking_state = TState::TRACKING;
}
void SpecificWorker::set_nose_target_to_default()
{
    //if(this->already_in_default == true) return;
    if(auto pan_tilt = G->get_node(viriato_head_camera_pan_tilt); pan_tilt.has_value())
    {
        G->add_or_modify_attrib_local<viriato_head_pan_tilt_nose_target_att>(pan_tilt.value(), nose_default_pose);
        G->update_node(pan_tilt.value());
        this->already_in_default = true;
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
        move_base(robot);
    }
    else
        qWarning() << __FILE__ << __FUNCTION__ << "No object of interest " << QString::fromStdString(object_of_interest) << "found in G";
}
void SpecificWorker::move_base(DSR::Node &robot)
{
    if(auto pantilt = inner_eigen->transform_axis(robot_name, viriato_head_camera_pan_joint); pantilt.has_value())
    {
        float current_pan_angle = pantilt.value()[5];
        if(auto robot_pose = inner_eigen->transform_axis(world_name, robot_name); robot_pose.has_value() )
        {
            if(const auto dist_o = inner_eigen->transform(camera_name, object_of_interest); dist_o.has_value())
            {
                float dist = dist_o.value().norm();
                float rx = robot_pose.value()[0];
                float ry = robot_pose.value()[1];
                float r_angle = robot_pose.value()[5];
                // select place to send the robot
                // move towards a safe spot close to the object but look for a good approaching direction
                const float landa = -1.f / log(0.1);
                float adv_speed =
                        1000 * exp(-(current_pan_angle * current_pan_angle) / landa) * std::min(dist / 3000.f, 1.f);
                // store target base dummy
                //            G->add_or_modify_attrib_local<robot_target_x_att>(robot, rx);
                //            G->add_or_modify_attrib_local<robot_target_y_att>(robot, ry);
                //            G->add_or_modify_attrib_local<robot_target_angle_att>(robot, r_angle + current_pan_angle);
                G->add_or_modify_attrib_local<robot_ref_adv_speed_att>(robot, adv_speed/60.f);
                G->add_or_modify_attrib_local<robot_ref_side_speed_att>(robot, 0.f);
                G->add_or_modify_attrib_local<robot_ref_rot_speed_att>(robot, -current_pan_angle * 10.f);
                std::cout << "dist " << dist << " rot:" << -current_pan_angle * 10 << " adv: " << adv_speed/40 << std::endl;
                G->update_node(robot);
                update_base_slider();
            }
        }
    }
}
void SpecificWorker::stop_robot()
{
    if(auto robot = G->get_node(robot_name); robot.has_value())
    {
        G->add_or_modify_attrib_local<robot_ref_adv_speed_att>(robot.value(), 0.f);
        G->add_or_modify_attrib_local<robot_ref_side_speed_att>(robot.value(), 0.f);
        G->add_or_modify_attrib_local<robot_ref_rot_speed_att>(robot.value(), 0.f);
        G->update_node(robot.value());
    }
}
void SpecificWorker::update_base_slider()
{
    if(auto r = inner_eigen->transform_axis(robot_name, viriato_head_camera_pan_joint); r.has_value())
    {
        float current_angle = r.value()[5];
        custom_widget.horizontalSlider->setValue(current_angle*100);   // scale to -100, 100
        custom_widget.pan->display(current_angle);
    }
}

/////////////////////////////////////////////////////////////////////////////
image_t SpecificWorker::createImage(const cv::Mat &src)
{
    // create YOLOv4 image from opencv matrix
    const int &h = src.rows;
    const int &w = src.cols;
    const int &c = src.channels();
    int step = w*c;

    int i, j, k;
    image_t out;
    out.h = h;
    out.w = w;
    out.c = c;
    out.data = (float *)calloc(h*w*c, sizeof(float));

    unsigned char *data = (unsigned char *)src.data;

    for(i = 0; i < h; ++i){
        for(k= 0; k < c; ++k){
            for(j = 0; j < w; ++j){
                out.data[k*w*h + i*w + j] = data[i*step + j*c + k]/255.;
            }
        }
    }
    return out;
}
void SpecificWorker::show_image(cv::Mat &imgdst, const vector<Box> &real_boxes, const std::vector<Box> synth_boxes)
{
    // display RGB image with detections
    for(const auto &box : real_boxes)
    {
        if(box.prob > 40)
        {
            auto p1 = cv::Point(box.left, box.top);
            auto p2 = cv::Point(box.right, box.bot);
            auto offset = int((box.bot - box.top) / 2);
            auto pt = cv::Point(box.left + offset, box.top + offset);
            cv::rectangle(imgdst, p1, p2, cv::Scalar(0, 0, 255), 4);
            auto font = cv::FONT_HERSHEY_SIMPLEX;
            cv::putText(imgdst, box.name + " " + std::to_string(int(box.prob)) + "%", pt, font, 0.8, cv::Scalar(0, 255, 255), 2);
        }
    }
    for(const auto &box : synth_boxes)
    {
        auto p1 = cv::Point(box.left, box.top);
        auto p2 = cv::Point(box.right, box.bot);
        auto offset = int((box.bot - box.top) / 2);
        auto pt = cv::Point(box.left + offset, box.top + offset);
        cv::rectangle(imgdst, p1, p2, cv::Scalar(0, 255, 0), 4);
        auto font = cv::FONT_HERSHEY_SIMPLEX;
        cv::putText(imgdst, box.name + " " + std::to_string(int(box.prob)) + "%", pt, font, 0.8, cv::Scalar(0, 255, 0), 2);
    }
    auto pix = QPixmap::fromImage(QImage(imgdst.data, imgdst.cols, imgdst.rows, QImage::Format_RGB888));
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
            //cam_api->bind_node(std::move(cam_node.value()));
            //if (const auto g_image = cam_api->get_existing_rgb_image(); g_image.has_value())
            if (const auto g_image = G->get_attrib_by_name<cam_rgb_att>(cam_node.value()); g_image.has_value())
                {
                    rgb_buffer.put(g_image.value().get(),
                               [this](const std::vector<std::uint8_t> &in, cv::Mat &out) {
                                   cv::Mat img(cam_api->get_height(), cam_api->get_width(), CV_8UC3,
                                               const_cast<std::vector<uint8_t> &>(in).data());
                                   //if(cam_api->get_width() == YOLO_IMG_SIZE and cam_api->get_height == YOLO_IMG_SIZE)
                                   cv::resize(img, out, cv::Size(YOLO_IMG_SIZE, YOLO_IMG_SIZE), 0, 0);
                               });
               }
            if (auto g_depth = G->get_attrib_by_name<cam_depth_att>(cam_node.value()); g_depth.has_value())
            {
                float *depth_array = (float *) g_depth.value().get().data();
                std::vector<float> res{depth_array, depth_array + g_depth.value().get().size() /sizeof(float) };
                depth_buffer.put(std::move(res));
            }
        }
        else
        {
            qWarning() << __FUNCTION__ << "No camera_node found in G";
        }
    }
    else if (type == intention_type_name)
    {
        auto node = G->get_node(id);
        if (auto parent = G->get_parent_node(node.value()); parent.has_value() and parent.value().name() == robot_name)
        {
            std::optional<std::string> plan = G->get_attrib_by_name<plan_att>(node.value());
            if (plan.has_value())
                plan_buffer.put(std::move(plan.value()),[](const std::string& plan_text, Plan &plan){ plan.json_to_plan(plan_text);});
        }
    }
}
void SpecificWorker::start_button_slot(bool checked)
{
    if(checked)  //track
    {
        custom_widget.startButton->setText("Stop");
        this->already_in_default = false;
        this->tracking = true;
        this->time_to_change = true;
    }
    else //stop tracking
    {
        custom_widget.startButton->setText("Start");
        this->tracking = false;
        // stop_robot();
    }
}
void SpecificWorker::change_object_slot(int index)
{
    qInfo() << __FUNCTION__ << "HERE";
    this->tracking_state = TState::CHANGING;
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
