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
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	G->write_to_json_file("./"+agent_name+".json");
    delete ynets[0];
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

    for(uint i=0; i<YOLO_INSTANCES; ++i)
        ynets.push_back(init_detector());

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
		std::cout<< __FUNCTION__ << "Graph loaded" << std::endl;

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
	auto rgb_camera = G->get_node(camera_name);
	if (rgb_camera.has_value())
	{
        auto g_image = G->get_rgb_image(rgb_camera.value());
        auto width = G->get_attrib_by_name<width_att>(rgb_camera.value());
        auto height = G->get_attrib_by_name<height_att>(rgb_camera.value());

        if (width.has_value() and height.has_value())
        {

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
    std::ifstream file(names_file);
    for(std::string line; getline(file, line);) names.push_back(line);
    Detector detector(cfg_file, weights_file);
    return &detector;
}

std::vector<SpecificWorker::Box> SpecificWorker::process_image_with_yolo(const cv::Mat &img)
{
    image_t yolo_img = createImage(img);
    std::vector<bbox_t> detections = ynets[0]->detect(yolo_img, 0.2, false);

    std::vector<Box> bboxes;
    for(int i = 0; i < detections.size(); ++i)
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

image_t SpecificWorker::createImage(const cv::Mat &src)
{
    const int &h = src.rows;
    const int &w = src.cols;
    const int &c = src.channels();
    int step = w*c;

    int i, j, k;
    image_t out;
    out.data = 0;
    out.h = h;
    out.w = w;
    out.c = c;

    for(i = 0; i < h; ++i){
        for(k= 0; k < c; ++k){
            for(j = 0; j < w; ++j){
                out.data[k*w*h + i*w + j] = src.data[i*step + j*c + k]/255.;
            }
        }
    }
    return out;
}

image_t SpecificWorker::createImage(const std::vector<uint8_t> &src, int width, int height, int depth)
{
    const int &h = height;
    const int &w = width;
    const int &c = depth;
    int step = w*c;

    int i, j, k;
    image_t out;
    out.data = 0;
    out.h = h;
    out.w = w;
    out.c = c;

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
    cv::imshow("", imgdst);
    cv::waitKey(1);
}

std::vector<SpecificWorker::Box> SpecificWorker::process_graph_with_yolosynth(const std::vector<std::string> &object_names, const DSR::Node& rgb_cam)
{
    std::string camera_name = "camera_pose";
    std::vector<Box> synth_box;
    //  get camera subAPI
    //  camera = G->get_camera_api(cam);        //THIS IDEA COULD BE INTERESTING!!!
    RMat::Cam camera(527, 527, 608/2, 608/2);

    for(auto &&object_name : object_names)
    {
        //get object from G
        if (auto object = G->get_node(object_name); object.has_value())
        {
            // compute bounding box
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
            // get projection of bounding box
            synth_box.push_back(box);
        }
    }
    return synth_box;
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
