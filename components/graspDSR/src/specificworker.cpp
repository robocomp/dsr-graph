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
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	// G->write_to_json_file("./"+agent_name+".json");
	G.reset();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }

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
		this->startup_check();
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
            current_opts = current_opts | opts::tree;
        if(graph_view)
            current_opts = current_opts | opts::graph;
        if(qscene_2d_view)
            current_opts = current_opts | opts::scene;
        if(osg_3d_view)
            current_opts = current_opts | opts::osg;
        graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts);
        setWindowTitle(QString::fromStdString(agent_name + "-" + std::to_string(agent_id)));

		this->Period = period;
		timer.start(Period);
	}

}

void SpecificWorker::compute()
{
    // check if there is an active command
    RoboCompCameraRGBDSimple::TImage rgb = get_rgb_from_G();
    try
    {
        RoboCompCameraRGBDSimple::TDepth depth;
        RoboCompObjectPoseEstimationRGBD::PoseType pose = this->objectposeestimationrgbd_proxy->getObjectPose(rgb, depth);
    }
    catch (const Ice::Exception &e)
    {
        std::cout << e << " No RoboCompPoseEstimation component found" << std::endl;
    }
    // Move arm
        // compute arm pose from object pose
        // G->ass
    // Check if target reached
}

RoboCompCameraRGBDSimple::TImage SpecificWorker::get_rgb_from_G()
{
    auto cam = G->get_node("Viriato_head_camera_front_sensor");
    if (cam.has_value())
    {
        RoboCompCameraRGBDSimple::TImage rgb;
        const auto rgb_data = G->get_attrib_by_name<vector<uint8_t>>(cam.value(), "rgb");
        const auto width = G->get_attrib_by_name<int32_t>(cam.value(), "width");
        const auto height = G->get_attrib_by_name<int32_t>(cam.value(), "height");
        const auto depth = G->get_attrib_by_name<int32_t>(cam.value(), "depth");
        rgb.cameraID = 0;
        rgb.width = width.value();
        rgb.height = height.value();
        rgb.focalx = 450;
        rgb.focaly = 450;
        rgb.alivetime = 0;
        rgb.image = rgb_data.value();

        if (depth.value() == 3)
        {
            cv::Mat image_rgb(height.value(), width.value(), CV_8UC3);
            memcpy(image_rgb.data, &rgb_data.value()[0],
                   width.value() * height.value() * sizeof(std::uint8_t) * depth.value());
            cv::imshow("Head", image_rgb);
            cv::waitKey(1);
        }
        return rgb;
    }
    else
        qFatal("Terminate in Compute. No node rgbd found");
}

////////////////////////////////////////////////////////////////////////

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
// RoboCompObjectPoseEstimationRGBD::TImage
// RoboCompObjectPoseEstimationRGBD::TDepth
// RoboCompObjectPoseEstimationRGBD::ObjectPose

