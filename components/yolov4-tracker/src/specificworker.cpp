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
		G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, "", dsrgetid_proxy); // Init nodes
		std::cout<< __FUNCTION__ << " Graph loaded" << std::endl;

        // get camera_api
        if(auto cam_node = G->get_node(camera_name); cam_node.has_value())
        {
            cam_api = G->get_camera_api(cam_node.value());
            //    const double fx=527; const double fy=527;
        }
        else
            qFatal("YoloV4_tracker terminate: could not find a camera node");
        inner_eigen = G->get_inner_eigen_api();
        rt_api = G->get_rt_api();

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

		// custom_widget
		graph_viewer->add_custom_widget_to_dock("YoloV4-tracker", &custom_widget);
		setWindowTitle(QString::fromStdString(agent_name + "-") + QString::number(agent_id));

		// ignore attributes
        G->set_ignored_attributes<laser_angles_att, laser_dists_att>();

        // Connect G SLOTS
        connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::update_node_slot);

        // Initialize combobox
        auto glass_nodes = G->get_nodes_by_type(glass_type);
        for(const auto &node : glass_nodes)
        {
            QVariant data;
            data.setValue(node.name());
            custom_widget.comboBox->addItem(QString::fromStdString(node.name()), data);
        }
        connect(custom_widget.startButton, SIGNAL(clicked(bool)), this, SLOT(start_button_slot(bool)));
        connect(custom_widget.comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(change_object_slot(int)));

        this->Period = period;
        timer.start(Period);
        READY_TO_GO = true;
	}
}

void SpecificWorker::compute()
{
    // select object of interest

    const auto g_image = rgb_buffer.try_get();
    const auto g_depth = depth_buffer.try_get();
    if (g_image.has_value() and g_depth.has_value())
    {
        this->depth_array = g_depth.value();
        cv::Mat imgyolo = g_image.value();
        // get detections using YOLOv4 network
        std::vector<SpecificWorker::Box> real_objects = process_image_with_yolo(imgyolo);
        // predict where OIs will be in yolo space
        std::vector<SpecificWorker::Box> synth_objects = process_graph_with_yolosynth({object_of_interest});
        show_image(imgyolo, real_objects, synth_objects);
        switch (tracking_state)
        {
            case TState::IDLE:
                set_nose_target_to_default();
                break;
            case TState::TRACKING:
                if(auto res = std::find_if(real_objects.begin(), real_objects.end(), [this](auto &a)
                        { return (a.name == "cup");}); res != real_objects.end())
                    compute_prediction_error(*res, synth_objects.front());
                track_object_of_interest();
                break;
            case TState::CHANGING:
                change_to_new_target();
                break;
        };

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
void SpecificWorker::track_object_of_interest()
{
    static Mat::Vector3d ant_pose;
    auto object = G->get_node(object_of_interest);
    auto pan_tilt = G->get_node(viriato_pan_tilt);
    if(object.has_value() and pan_tilt.has_value())
    {
        // get object pose in world coordinate frame
        auto po = inner_eigen->transform(world_name, object_of_interest);
        auto pose = inner_eigen->transform(camera_name, object_of_interest);
        if (po.has_value() and pose.has_value() and ((pose.value() - ant_pose).cwiseAbs2().sum() > 10))
        {
               G->add_or_modify_attrib_local<viriato_head_pan_tilt_nose_target_att>(pan_tilt.value(), std::vector<float>{(float)po.value().x(), (float)po.value().y(), (float)po.value().z()});
               G->update_node(pan_tilt.value());
               //qInfo() <<"NOW ...." << po.value().x() << po.value().y() << po.value().z() << " - " << (pose.value() - ant_pose).cwiseAbs2().sum() ;
               //qInfo() <<"NOW ...." << pose.value().x() << pose.value().y() << pose.value().z() << " - " << ant_pose.x() << ant_pose.y() << ant_pose.z();
        }
        ant_pose = pose.value();
    }
    else
        qWarning() << __FILE__ << __FUNCTION__ << "No object of interest " << QString::fromStdString(object_of_interest) << "found in G";
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
                            qInfo() << __FUNCTION__ << r << rs << area << error.manhattanLength() << rs.width()*3;
                            return true;
                        }
                        else
                            return false;
                     };

    const auto &r = real_box;
    auto tp = get_existing_roi_depth(Eigen::AlignedBox<float, 2>(Eigen::Vector2f(r.left, r.bot), Eigen::Vector2f(r.right, r.top)));
    if (tp.has_value())
    {
        auto [x,y,z] = tp.value();
        real_box.depth = sqrt(x*x+y*y+z*z);
        qInfo() << "depth " << real_box.depth;
        if (are_close(real_box, synth_box))
                add_edge(tp.value());
    }
}
void SpecificWorker::add_edge(const std::tuple<float,float,float> &tp)
{
    if (auto object = G->get_node(object_of_interest); object.has_value())
    {
        DSR::Edge edge(object.value().id(), cam_api->get_id(), "looking-at", agent_id);
        auto &[x, y, z] = tp;
        G->add_attrib_local<looking_at_translation_att>(edge, std::vector<float>{x, y, z});
        G->add_attrib_local<looking_at_rotation_euler_xyz_att>(edge, std::vector<float>{0.f, 0.f, 0.f});
        if (G->insert_or_assign_edge(edge))
        {
            if (const auto loop = inner_eigen->transform(viriato_head_camera_name, object_of_interest); loop.has_value())
                std::cout << __FUNCTION__ << " [" << x << " " << y << " " << z << "] - Loop [" << loop.value().x()
                          << " " << loop.value().y() << " " << loop.value().z() << "]" << std::endl;
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
std::optional<std::tuple<float,float,float>> SpecificWorker::get_existing_roi_depth(const Eigen::AlignedBox<float, 2> &roi)
{

    auto left = (int)roi.min().x(); auto bot = (int)roi.min().y();
    auto right = (int)roi.max().x(); auto top = (int)roi.max().y();  // botom has higher numeric value. rows start in 0 up
    if(left<right and bot>top)
    {
        auto size = (right - left) * (bot - top);
        std::vector<float> values(size);
        std::size_t k = 0;
        const auto &width = cam_api->get_width();
        for (int i = left; i < right; i++)
            for (int j = top; j < bot; j++)
                values[k++] = this->depth_array[i * width + j];

        auto mv = std::min(values.begin(), values.end());
        auto Y = *mv * 1000;
        auto X = (right - left) / 2 * Y / cam_api->get_focal_x();
        auto Z = (bot - top) / 2 * Y / cam_api->get_focal_y();
        qInfo() << size << X << Y << Z;
        return std::make_tuple(X, Y, Z);
    }
    else
    {
        qWarning() << __FUNCTION__ << "Incorrect ROI dimensions l r t b: " << left << right << top << bot << ". Returning empty";
        return {};
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
void SpecificWorker::set_nose_target_to_default()
{
    //if(this->already_in_default == true) return;
    if(auto pan_tilt = G->get_node(viriato_head_camera_pan_tilt); pan_tilt.has_value())
    {
        if(auto nose = inner_eigen->transform(world_name, this->nose_default_pose, viriato_head_camera_pan_tilt); nose.has_value())
        {
            G->add_or_modify_attrib_local<viriato_head_pan_tilt_nose_target_att>(pan_tilt.value(), std::vector<float>{(float)nose.value().x(), (float)nose.value().y(), (float)nose.value().z()});
            G->update_node(pan_tilt.value());
        }
        this->already_in_default = true;
    }
}

///////////////////////////////////////////////////////////////////
/// Asynchronous changes on G nodes from G signals
///////////////////////////////////////////////////////////////////
void SpecificWorker::update_node_slot(const std::int32_t id, const std::string &type)
{
    if (type == rgbd_type and (std::uint32_t)id == cam_api->get_id())
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
    else if (type == intention_type)
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
        const auto index = custom_widget.comboBox->currentIndex();
        object_of_interest = custom_widget.comboBox->itemData(index).value<std::string>();
        custom_widget.startButton->setText("Stop");
        this->already_in_default = false;
        this->tracking_state = TState::TRACKING;
    }
    else //stop tracking
    {
        custom_widget.startButton->setText("Start");
        //object_of_interest = "no_object";
        this->tracking_state = TState::IDLE;
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
