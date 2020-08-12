/*
 *    Copyright (C) 2020 by YOUR NAME HERE
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
	delete ynets[0];
	//G->write_to_json_file("./"+agent_name+".json");
	G.reset();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	static_assert(std::is_nothrow_move_constructible<yolo::image>::value, "MyType should be noexcept MoveConstructible");
    agent_name = params["agent_name"].value;
    agent_id = stoi(params["agent_id"].value);
    tree_view = params["tree_view"].value == "true";
    graph_view = params["graph_view"].value == "true";
    qscene_2d_view = params["2d_view"].value == "true";
    osg_3d_view = params["3d_view"].value == "true";
	SHOW_IMAGE = params["ShowImage"].value == "true" or params["ShowImage"].value == "True";
	path_to_yolodata = params["path_to_yolodata"].value;
	for(uint i=0; i<YOLO_INSTANCES; ++i)
		ynets.push_back(init_detector(path_to_yolodata));

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
        // create graph
        G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, "", dsrgetid_proxy); // Init nodes
        std::cout << __FUNCTION__ << "Graph loaded" << std::endl;

        // innermodel
        innermodel = G->get_inner_api();

        // Graph viewer
        using opts = DSR::DSRViewer::view;
        int current_opts = 0;
        //opts main = opts::none;
        if (tree_view)
            current_opts = current_opts | opts::tree;
        if (graph_view)
            current_opts = current_opts | opts::graph;
        if (qscene_2d_view)
            current_opts = current_opts | opts::scene;
        if (osg_3d_view)
            current_opts = current_opts | opts::osg;
        dsr_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts);
        setWindowTitle(QString::fromStdString(agent_name + "-" + std::to_string(agent_id)));

        std::cout << "Worker Initialized" << std::endl;
        this->Period = period;
        timer.start(Period);
        READY_TO_GO = true;
    }
}

void SpecificWorker::compute()
{
    // get current plan and extract Object of Interest
    // if plan exists and same plan
        std::string object_of_interest = "glass_1";
        // get image and extract objects
        auto rgb_camera = G->get_node("Viriato_head_camera_front_sensor");
        if (rgb_camera.has_value())
        {
            //cv::Mat img = get_rgb_image(rgb_camera.value());
            auto g_image = G->get_attrib_by_name<std::vector<uint8_t>>(rgb_camera.value(), "rgb");
            auto width = G->get_attrib_by_name<int32_t>(rgb_camera.value(), "width");
            auto height = G->get_attrib_by_name<int32_t>(rgb_camera.value(), "height");
            if (g_image.has_value() and width.has_value() and height.has_value())
            {
                auto img = cv::Mat(height.value(), width.value(), CV_8UC3, &g_image.value()[0]);
                // resize img for Yolo
                cv::Mat imgyolo(608, 608, CV_8UC3);
                cv::resize(img, imgyolo, cv::Size(608, 608), 0, 0, CV_INTER_LINEAR);
                std::vector<SpecificWorker::Box> real_objects = process_image_with_yolo(imgyolo);
                // predict where OI will be in yolo space
                std::vector<SpecificWorker::Box> synth_objects = process_graph_with_yolosynth({object_of_interest}, rgb_camera.value());

                show_image(imgyolo, real_objects, synth_objects);
                //compute_prediction_error( real_objects, synth_objects);
                // compute corrections and insert or assign to G

                // assess current state of the plan and choose top-down or bottom-up
                // if top-down choose OI
                // if bottom-up choose another object
                // compute correction saccade for t + delta and write in G (node head_camera) as "current_target_orientation"
                // option2: write in G the target object pose in t + delta (node head_camera) as "center_target_reference"
                // so ViriatoDSR can send the dummy command. ViriatoPyrep, on receiving it must stretch the camera "nose" to the target pose.

                // get object_or_interest and pan_tilt nodes
                auto object = G->get_node(object_of_interest);
                auto pan_tilt = G->get_node("head_camera_pan_tilt");
                if(object.has_value() and pan_tilt.has_value())
                {
                    // get object pose in camera coordinate frame
                    //auto pose = G->get_RT_pose_from_parent(object.value());
                    auto pose = innermodel->transformS("head_camera_pan_tilt", object_of_interest);
                    if (pose.has_value())
                    {
                        // get pan_tilt current target pose
                        if(auto current_pose = G->get_attrib_by_name<std::vector<float>>(pan_tilt.value(), "target_pose"); current_pose.has_value())
                        {
                            QVec qcurrent_pose(current_pose.value());
                            // if they are different modify G
                            if (not(pose == qcurrent_pose))
                            {
                                G->add_or_modify_attrib_local(pan_tilt.value(), "target_pose", std::vector<float>{pose.value().x(), pose.value().y(), pose.value().z()});
                                G->update_node(pan_tilt.value());
                            }
                        }
                        else
                            qWarning() << __FILE__ << __FUNCTION__ << "No attribute target_pose found in G for camera_head_pan_tilt node";
                    } else
                        qWarning() << __FILE__ << __FUNCTION__ << "No attribute pose found in G for " << QString::fromStdString(object_of_interest);
                }
                else
                    qWarning() << __FILE__ << __FUNCTION__ << "No object of interest " << QString::fromStdString(object_of_interest) << "found in G";
            }
            else
                qWarning() << __FILE__ << __FUNCTION__ << "No attributes image, widht or height found in G";
        }
        else
            qWarning() << __FILE__ << __FUNCTION__ << "No node Viriato_head_camera_front_sensor found in G";
}

cv::Mat SpecificWorker::get_rgb_image(const Node &rgb_camera)
{
    auto g_image = G->get_attrib_by_name<std::vector<uint8_t>>(rgb_camera, "rgb");
    auto width = G->get_attrib_by_name<int32_t>(rgb_camera, "width");
    auto height = G->get_attrib_by_name<int32_t>(rgb_camera, "height");
    if (g_image.has_value() and width.has_value() and height.has_value())
        return cv::Mat(height.value(), width.value(), CV_8UC3, &g_image.value()[0]);

    qWarning() << __FILE__ << __FUNCTION__ << "No attributes image, widht or height found in G";
    return cv::Mat();
}

std::vector<SpecificWorker::Box> SpecificWorker::process_image_with_yolo(const cv::Mat &img)
{
    return detectLabels(ynets[0], createImage(img), .5, .5);
}

std::vector<SpecificWorker::Box> SpecificWorker::process_graph_with_yolosynth(const std::vector<std::string> &object_names, const Node& rgb_camera)
{
    std::string camera_name = "camera_pose";
    std::vector<Box> synth_box;
    //  get camera subAPI
    //  camera = G->get_camera_api(cam);        //THIS IDEA COULD BE INTERESTING!!!
    //  auto focal = G->get_attrib_by_name<int32_t>(rgb_camera, "focal");
    //  auto width = G->get_attrib_by_name<int32_t>(rgb_camera, "width");
    //  auto height = G->get_attrib_by_name<int32_t>(rgb_camera, "height");
    //RMat::Cam camera(focal, focal, width.value()/2, height.value()/2);
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

void SpecificWorker::compute_prediction_error(const std::vector<SpecificWorker::Box> &real_boxes, const std::vector<SpecificWorker::Box> synth_boxes)
{
    //For each synthetic object
//    for(const auto &synth: synth_boxes)
//    {
//        synth.intersectArea = 0;
//        synth.explained = false;
//        std::vector<TCandidate> listCandidates;
//
//        //It is compared with real objects
//        for(auto &yolo: real_boxes)
//        {
//            //If it is the same type and has not been assigned yet
//            if(synth.type == yolo.type and yolo.assigned == false)
//            {
//                //A rectangle with the real object is created
//                QRect r(QPoint(yolo.box.x,yolo.box.y),QPoint(yolo.box.w, yolo.box.h));
//                //A rectangle with the sythetic object is created
//                QRect rs(QPoint(synth.box.x,synth.box.y),QPoint(synth.box.w, synth.box.h));
//                //Compute intersection percentage between synthetic and real
//                QRect i = rs.intersected(r);
//                //The area is normalized between 0 and 1 dividing by the minimum between both areas of each object
//                float area = (float)(i.width() * i.height()) / std::min(rs.width() * rs.height(), r.width() * r.height());
//                //The displacement vector between the two images is calculated
//                QPoint error = r.center() - rs.center();
//
//                // If the area is 0 there is no intersection
//                // If the error is less than twice the width of the synthetic rectangle
//                if(area > 0 or error.manhattanLength()< rs.width()*3)
//                {
//                    //A candidate is created and added to the list of candidates in an orderly manner according to the area and the error
//                    //The object will be placed earlier in the list the less difference there is with the original
//                    TCandidate tc = {area,error,&yolo};
//                    listCandidates.insert(std::upper_bound( listCandidates.begin(), listCandidates.end(),tc,
//                                                            [](auto a, auto b) {
//                                                                return (a.area > b.area) or ((a.area==b.area) and (a.error.manhattanLength() < b.error.manhattanLength()));
//                                                            }),
//                                          tc);
//                }
//            }
//        }
//
//        // If there are candidates, the first one is taken, assigned and the synthetic object is marked as explained
//        if(listCandidates.empty() == false)
//        {
//            listCandidates.front().yolo->assigned = true;
//            synth.intersectArea = listCandidates.front().area;
//            synth.error = listCandidates.front().error;
//            synth.explained = true;
//        }
//    }
//
//    //listDelete: Extract objects not explained by measurements
//    for(auto &o : tables[processTable].listVisible)
//        if(o.explained == false)
//            tables[processTable].listDelete.push_back(o);
//
//    //listCreate: objects to be created due to measurements not assigned to objects
//    for(auto &y: tables[processTable].listYoloObjects)
//        if(y.assigned == false)
//        {
//            TObject n;
//            n.type = y.type;
//            // Get 3D pose from RGBD
//            QRect r(QPoint(y.box.x,y.box.y),QPoint(y.box.w,y.box.h));
//            int idx = r.center().y()*640 + r.center().x();
//            RoboCompRGBD::PointXYZ p = pointMatrix[idx];
//            n.pose = QVec::vec6(p.x, p.y, p.z, 0, 0, 0);
//            tables[processTable].listCreate.push_back(n);
//        }

}

void SpecificWorker::show_image(cv::Mat &imgdst, const std::vector<SpecificWorker::Box> &real_boxes, const std::vector<SpecificWorker::Box> synth_boxes)
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
    for(const auto &box : synth_boxes)
    {
        if(box.prob > 50)
        {
            auto p1 = cv::Point(box.left, box.top);
            auto p2 = cv::Point(box.right, box.bot);
            auto offset = int((box.bot - box.top) / 2);
            auto pt = cv::Point(box.left + offset, box.top + offset);
            cv::rectangle(imgdst, p1, p2, cv::Scalar(0, 255, 0), 4);
            auto font = cv::FONT_HERSHEY_SIMPLEX;
            cv::putText(imgdst, box.name + " " + std::to_string(int(box.prob)) + "%", pt, font, 1, cv::Scalar(255, 0, 255), 2);
        }
    }
    cv::imshow("", imgdst);
    cv::waitKey(1);
}

yolo::network* SpecificWorker::init_detector(std::string path_to_yolodata_)
{
    qDebug() << __FUNCTION__ << "Reading data from " << QString::fromStdString(path_to_yolodata_);
	std::string cocodata = path_to_yolodata_ +  "coco.data";
	std::string yolocfg = path_to_yolodata_ + "cfg/yolov3.cfg";
	std::string yoloweights = path_to_yolodata_ + "yolov3.weights";
	std::string yolonames = path_to_yolodata_ + "coco.names";
	
	names = yolo::get_labels(const_cast<char*>(yolonames.c_str()));
 	yolo::network *ynet = yolo::load_network(const_cast<char*>(yolocfg.c_str()),const_cast<char*>(yoloweights.c_str()), 0);
	yolo::set_batch_network(ynet, 1);
	cuda_set_device(0);
	return ynet;
	
}

yolo::image SpecificWorker::createImage(const cv::Mat &src)		//reentrant
{
	const int &h = src.rows;
	const int &w = src.cols;
	const int &c = src.channels();
	int step = w*c;
	
	int i, j, k;
	image out = make_image(w, h, c);
			
	for(i = 0; i < h; ++i){
		for(k= 0; k < c; ++k){
			for(j = 0; j < w; ++j){
				out.data[k*w*h + i*w + j] = src.data[i*step + j*c + k]/255.;
			}
		}
	}
	return out;
}

yolo::image SpecificWorker::createImage(const std::vector<uint8_t> &src, int width, int height, int depth)		//reentrant
{
	const int &h = height;
	const int &w = width;
	const int &c = depth;
	int step = w*c;
	
	int i, j, k;
	image out = make_image(w, h, c);
	
	// try with std::parallel
	for(i = 0; i < h; ++i){
		for(k= 0; k < c; ++k){
			for(j = 0; j < w; ++j){
				out.data[k*w*h + i*w + j] = src[i*step + j*c + k]/255.;
			}
		}
	}
	return out;
}

//Must be reentrant
std::vector<SpecificWorker::Box> SpecificWorker::detectLabels(yolo::network *ynet, const yolo::image &yoloImage, float thresh, float hier_thresh)
{
	// ytime1=clock();
	// image sized = letterbox_image(im, net->w, net->h);
	//printf("net\n");
	// printf("Letterbox elapsed %f mseconds.\n", sec(clock()-time1)*1000);
	// ytime1=clock();
	
	//yolo::image yoloImage = createImage( img );	
	
	layer l = ynet->layers[ynet->n-1];
	network_predict(ynet, yoloImage.data);	
	//remember_network(ynet);	
	//avg_predictions(ynet);
	int numboxes;
	yolo::detection *dets = get_network_boxes(ynet, yoloImage.w, yoloImage.h, thresh, hier_thresh, 0, 1, &numboxes);
	//printf("Test-Detector: Network-predict elapsed in %f mseconds.\n",sec(clock()-ytime1)*1000);
	const float solapamiento = 0.5;
	do_nms_obj(dets, numboxes, l.classes, solapamiento);
	
	std::vector<Box> boxes;
	for(int i = 0; i < numboxes; ++i)
	{
		const auto &d = dets[i];
		int clas = max_index(dets[i].prob, dets[i].classes);
		float prob = d.objectness;		
		if(d.objectness > 0.5)
		{
 			const yolo::box &b = d.bbox;
 			int left  = (b.x-b.w/2.)*yoloImage.w;
 			int right = (b.x+b.w/2.)*yoloImage.w;
 			int top   = (b.y-b.h/2.)*yoloImage.h;
 			int bot   = (b.y+b.h/2.)*yoloImage.h;
 			if(left < 0) left = 0;
 			if(right > yoloImage.w-1) right = yoloImage.w-1;
 			if(top < 0) top = 0;
 			if(bot > yoloImage.h-1) bot = yoloImage.h-1;
 			boxes.emplace_back(Box{names[clas], left, top, right, bot, prob*100});
 		}
	}	
	qDebug() << __FILE__ << __FUNCTION__ << "LABELS " << boxes.size();
	free_detections(dets, numboxes);
	free_image(yoloImage);
	return boxes;
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

