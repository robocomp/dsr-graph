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

        // innermodel
        innermodel = G->get_inner_api();

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
        READY_TO_GO = true;
	}
}

void SpecificWorker::compute()
{
    // get camera node from G
    if( auto rgb_camera = G->get_node(camera_name); rgb_camera.has_value())
    {
        // get and process RGB image
        auto g_image = G->get_rgb_image(rgb_camera.value());
        auto width = G->get_attrib_by_name<rgb_width>(rgb_camera.value());
        auto height = G->get_attrib_by_name<rgb_height>(rgb_camera.value());
        if (width.has_value() and height.has_value())
        {
            // create opencv image
            auto g_img = g_image.value().get();
            cv::Mat img = cv::Mat(height.value(), width.value(), CV_8UC3, &g_img[0]);
            // process opencv image
            //const int img_size = 416;
            const int img_size = 608;   // for faster performance and lower memory usage, set to 416 (check in yolov4.cfg)
            cv::Mat imgyolo(img_size, img_size, CV_8UC3);
            cv::resize(img, imgyolo, cv::Size(img_size, img_size), 0, 0);

            // get detections using YOLOv4 network
            std::vector<SpecificWorker::Box> real_objects = process_image_with_yolo(imgyolo);
            // predict where OI will be in yolo space
            std::vector<SpecificWorker::Box> synth_objects = process_graph_with_yolosynth({object_of_interest}, rgb_camera.value());

            // show detections on image
            qInfo() << real_objects.size() << synth_objects.size();
            for(auto s : synth_objects)
                qInfo() << QString::fromStdString(s.name) << s.bot << s.top << s.left << s.right;
            show_image(imgyolo, real_objects, synth_objects);

            // compute_prediction_error( real_objects, synth_objects);
            // compute corrections and insert or assign to G
            // assess current state of the plan and choose top-down or bottom-up
            // if top-down choose OI
            // if bottom-up choose another object
            // option2: write in G the target object pose in t + delta (node head_camera) as "center_target_reference"
            // so ViriatoDSR can send the dummy command. ViriatoPyrep, on receiving it must stretch the camera "nose" to the target pose.

            // track object of interest
            track_object_of_interest();

        }
        else
        {
            qWarning() << __FILE__ << __FUNCTION__ << "No attributes image, widht or height found in G";
        }
    }
    else
    {
        qWarning() << __FILE__ << __FUNCTION__ << "No node Viriato_head_camera_front_sensor found in G";
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
    std::vector<Box> bboxes;
    for(unsigned int i = 0; i < detections.size(); ++i)
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
        bboxes.emplace_back(Box{names.at(cls), left, top, right, bot, prob*100});
    }
    qDebug() << __FILE__ << __FUNCTION__ << "LABELS " << bboxes.size();
    ynets[0]->free_image(yolo_img);
    return bboxes;
}

std::vector<SpecificWorker::Box> SpecificWorker::process_graph_with_yolosynth(const std::vector<std::string> &object_names, const DSR::Node& rgb_cam)
{
    std::string camera_name = "camera_pose";
    std::vector<Box> synth_box;
    //  get camera subAPI
    RMat::Cam camera(527, 527, 608/2, 608/2);
    for(auto &&object_name : object_names)
    {
        //get object from G
        if (auto object = G->get_node(object_name); object.has_value())
        {
            // compute bounding box projected in the camera coordinate frame
            std::vector<QVec> bb_in_camera;
            const float h = 150;
            bb_in_camera.emplace_back(camera.project(innermodel->transformS(camera_name, QVec::vec3(40,0,40), object_name).value()));
            bb_in_camera.emplace_back(camera.project(innermodel->transformS(camera_name, QVec::vec3(-40,0,40), object_name).value()));
            bb_in_camera.emplace_back(camera.project(innermodel->transformS(camera_name, QVec::vec3(40,0,-40), object_name).value()));
            bb_in_camera.emplace_back(camera.project(innermodel->transformS(camera_name, QVec::vec3(-40,-0,-40), object_name).value()));
            bb_in_camera.emplace_back(camera.project(innermodel->transformS(camera_name, QVec::vec3(40,h,40), object_name).value()));
            bb_in_camera.emplace_back(camera.project(innermodel->transformS(camera_name, QVec::vec3(-40,h,40), object_name).value()));
            bb_in_camera.emplace_back(camera.project(innermodel->transformS(camera_name, QVec::vec3(40,h,-40), object_name).value()));
            bb_in_camera.emplace_back(camera.project(innermodel->transformS(camera_name, QVec::vec3(-40,h,-40), object_name).value()));
            // Compute a bounding box of pixel coordinates
            // Sort the coordinates x
            auto xExtremes = std::minmax_element(bb_in_camera.begin(), bb_in_camera.end(),
                                                 [](const QVec& lhs, const QVec& rhs) {
                                                     return lhs.x() < rhs.x();
                                                 });
            // Sort the coordinates y
            auto yExtremes = std::minmax_element(bb_in_camera.begin(), bb_in_camera.end(),
                                                 [](const QVec& lhs, const QVec& rhs) {
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
            // store projection of bounding box
            synth_box.push_back(box);
        }
    }
    return synth_box;
}

void SpecificWorker::track_object_of_interest()
{
    auto object = G->get_node(object_of_interest);
    auto pan_tilt = G->get_node(viriato_pan_tilt);
    if(object.has_value() and pan_tilt.has_value())
    {
        // get object pose in camera coordinate frame
        auto pose = innermodel->transformS(viriato_pan_tilt, object_of_interest);
        // make it 200 mm vector
        auto n_pose = pose->normalize() * (RMat::T)200;
        //n_pose = n_pose * (RMat::T)200;
        // transform to world coordinate frame so in Coppelia appears as the nose_target_dummy
        pose = innermodel->transformS(world_node, n_pose, viriato_pan_tilt);
        if (pose.has_value())
        {
            // get pan_tilt current target pose
            if(auto current_pose = G->get_attrib_by_name<viriato_pan_tilt_nose_target>(pan_tilt.value()); current_pose.has_value())
            {
                QVec qcurrent_pose(current_pose.value());
                //if they are different modify G
                if (not pose.value().equals(qcurrent_pose, 1.0))  // use an epsilon limited difference
                {
                    G->add_or_modify_attrib_local<viriato_pan_tilt_nose_target>(pan_tilt.value(), std::vector<float>{pose.value().x(), pose.value().y(), pose.value().z()});
                    G->update_node(pan_tilt.value());
                }
            }
            else
            {
                qWarning() << __FILE__ << __FUNCTION__ << "No attribute " << QString::fromStdString(nose_target) << " found in G for camera_head_pan_tilt node";
            }
        }
        else
        {
            qWarning() << __FILE__ << __FUNCTION__ << "No attribute pose found in G for " << QString::fromStdString(object_of_interest);
        }
    }
    else
    {
        qWarning() << __FILE__ << __FUNCTION__ << "No object of interest " << QString::fromStdString(object_of_interest) << "found in G";
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

image_t SpecificWorker::createImage(const std::vector<uint8_t> &src, int width, int height, int depth)
{
    // create YOLOv4 image from array of bytes
    const int &h = height;
    const int &w = width;
    const int &c = depth;
    int step = w*c;

    int i, j, k;
    image_t out;
    out.h = h;
    out.w = w;
    out.c = c;
    out.data = (float *)calloc(h*w*c, sizeof(float));

    for(i = 0; i < h; ++i){
        for(k= 0; k < c; ++k){
            for(j = 0; j < w; ++j){
                out.data[k*w*h + i*w + j] = src[i*step + j*c + k]/255.;
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
        if(box.prob > 50)
        {
            auto p1 = cv::Point(box.left, box.top);
            auto p2 = cv::Point(box.right, box.bot);
            auto offset = int((box.bot - box.top) / 2);
            auto pt = cv::Point(box.left + offset, box.top + offset);
            cv::rectangle(imgdst, p1, p2, cv::Scalar(0, 0, 255), 4);
            auto font = cv::FONT_HERSHEY_SIMPLEX;
            cv::putText(imgdst, box.name + " " + std::to_string(int(box.prob)) + "%", pt, font, 1, cv::Scalar(0, 255, 255), 2);
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
        cv::putText(imgdst, box.name + " " + std::to_string(int(box.prob)) + "%", pt, font, 1, cv::Scalar(0, 255, 255), 2);
    }
    cv::drawMarker(imgdst, cv::Point(imgdst.rows/2, imgdst.cols/2),  cv::Scalar(0, 0, 255), cv::MARKER_CROSS,0, 1);
    cv::imshow("", imgdst);
    cv::waitKey(1);
}

void SpecificWorker::compute_prediction_error(const vector<Box> &real_boxes, const vector<Box> synth_boxes)
{

}

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
