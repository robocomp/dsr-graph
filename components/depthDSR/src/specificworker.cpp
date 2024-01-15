/*
 *    Copyright (C) 2020 by vaibhaw khemka
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
        G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id); // Init nodes
        std::cout<< __FUNCTION__ << "Graph loaded" << std::endl;  

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
        graph_viewer->add_custom_widget_to_dock("DepthMap", &custom_widget); // custom_widget

        setWindowTitle(QString::fromStdString(agent_name + "-") + QString::number(agent_id));

        // get inner eigen model sub-API
        inner_eigen_api = G->get_inner_eigen_api();

        // get RT sub-API
        rt_api = G->get_rt_api();

        // get camera sub-API
        auto cam = G->get_node(viriato_head_camera_name);
        if (cam.has_value())
        {
            cam_api = G->get_camera_api(cam.value());
        }
        else
        {
            qFatal("Terminate in Initialize. No node rgbd found");
        }


        this->Period = period;
        timer.start(Period);
    }
}

void SpecificWorker::compute()
{
    // read RGBD image from graph
    RoboCompCameraRGBDSimple::TImage rgb = get_rgb_from_G();

    // cast RGB image to OpenCV Mat
    cv::Mat img = cv::Mat(rgb.height, rgb.width, CV_8UC3, &rgb.image[0]);    
	    
    RoboCompDepthEstimation::DepthScene result;
    try
    {
        result = this->depthestimation_proxy->getDepthEstimation(rgb);
    }
    catch (const Ice::Exception &e)
    {
        std::cout << e << " No DepthEstimation component found" << std::endl;
    }

        // display RGB image on QT widget
    show_image(img,result);

        // inject estimated depth into graph
     //   std::cout << "Inject DNN-estimated depth into G" << std::endl;
     //   this->inject_estimated();
   // }
}

/////////////////////////////////////////////////////////////////
//                     G read utilities
/////////////////////////////////////////////////////////////////

RoboCompCameraRGBDSimple::TImage SpecificWorker::get_rgb_from_G()
{
    // get head camera node
    auto cam = G->get_node(viriato_head_camera_name);
    if (cam.has_value())
    {
        // read RGB data attributes from graph 
        RoboCompCameraRGBDSimple::TImage rgb;
        try
        {
            auto rgb_data = cam_api->get_rgb_image();
            const auto width = G->get_attrib_by_name<cam_rgb_width_att>(cam.value());
            const auto height = G->get_attrib_by_name<cam_rgb_height_att>(cam.value());
            const auto depth = G->get_attrib_by_name<cam_rgb_depth_att>(cam.value());
            const auto cam_id = G->get_attrib_by_name<cam_rgb_cameraID_att>(cam.value());
            const auto focalx = G->get_attrib_by_name<cam_rgb_focalx_att>(cam.value());
            const auto focaly = G->get_attrib_by_name<cam_rgb_focaly_att>(cam.value());
            const auto alivetime = G->get_attrib_by_name<cam_rgb_alivetime_att>(cam.value());

            // assign attributes to RoboCompCameraRGBDSimple::TImage
            rgb.image = rgb_data.value();
            rgb.width = width.value();
            rgb.height = height.value();
            rgb.depth = depth.value();
            rgb.cameraID = cam_id.value();
            rgb.focalx = focalx.value();
            rgb.focaly = focaly.value();
            rgb.alivetime = alivetime.value();

            return rgb;
        }
        catch (const std::exception &e)
        {
            std::cout << __FILE__ << __FUNCTION__ << __LINE__ << " " << e.what() << std::endl;
            std::terminate();
        }
    }
    else
    {
        qFatal("Terminate in Compute. No node rgbd found");
    }
}

void SpecificWorker::show_image(cv::Mat &img,RoboCompDepthEstimation::DepthScene result)
{
    // create QImage and display it on the widget
    cv::Mat depthmap = cv::Mat(result.height, result.width, CV_8UC3, &result.image[0]);
    auto pix1 = QPixmap::fromImage(QImage(img.data, img.cols, img.rows, QImage::Format_RGB888));
    auto pix2 = QPixmap::fromImage(QImage(depthmap.data, depthmap.cols, depthmap.rows, QImage::Format_RGB888));
    custom_widget.rgb_image->setPixmap(pix1);
    custom_widget.depth_image->setPixmap(pix2);
}


int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}

