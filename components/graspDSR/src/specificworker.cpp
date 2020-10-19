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
        G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, "", dsrgetid_proxy); // Init nodes
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
        graph_viewer->add_custom_widget_to_dock("Grasping", &custom_widget); // custom_widget

        setWindowTitle(QString::fromStdString(agent_name + "-") + QString::number(agent_id));

        // read objects point cloud for poses visualization
        objects_pcl = this->read_pcl_from_file();

        // get inner eigen model sub_API
        inner_eigen = G->get_inner_eigen_api();

        // set callback function upon left_hand_type node update signal
        connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::update_node_slot);

        this->Period = period;
        timer.start(Period);
    }
}

void SpecificWorker::compute()
{
    // read RGBD image from graph
    RoboCompCameraRGBDSimple::TImage rgb = get_rgb_from_G();
    RoboCompCameraRGBDSimple::TDepth depth = get_depth_from_G();

    // cast RGB image to OpenCV Mat
    cv::Mat img = cv::Mat(rgb.height, rgb.width, CV_8UC3, &rgb.image[0]);

    // get grasp object from QT widget
    QAbstractButton* sel_button = custom_widget.object_sel_group->checkedButton();
    if (sel_button)
    {
        QString button_text = sel_button->text();
        grasp_object = button_text.toStdString();
        std::cout << "Object '" << grasp_object << "' is selected to be the target" << std::endl;
    }

    // call pose estimation on RGBD and receive estimated poses
    std::cout << "Obtain DNN-estimated poses" << std::endl;
    RoboCompObjectPoseEstimationRGBD::PoseType poses;
    try
    {
        poses = this->objectposeestimationrgbd_proxy->getObjectPose(rgb, depth);
    }
    catch (const Ice::Exception &e)
    {
        std::cout << e << " No RoboCompPoseEstimation component found" << std::endl;
    }

    if (poses.size() != 0)
    {
        // display RGB image on QT widget
        show_image(img, poses);

        // inject estimated poses into graph
        std::cout << "Inject DNN-estimated poses into G" << std::endl;
        this->inject_estimated_poses(poses);
    }
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
            auto rgb_data = G->get_rgb_image(cam.value());
            const auto width = G->get_attrib_by_name<cam_rgb_width_att>(cam.value());
            const auto height = G->get_attrib_by_name<cam_rgb_height_att>(cam.value());
            const auto depth = G->get_attrib_by_name<cam_rgb_depth_att>(cam.value());
            const auto cam_id = G->get_attrib_by_name<cam_rgb_cameraID_att>(cam.value());
            const auto focalx = G->get_attrib_by_name<cam_rgb_focalx_att>(cam.value());
            const auto focaly = G->get_attrib_by_name<cam_rgb_focaly_att>(cam.value());
            const auto alivetime = G->get_attrib_by_name<cam_rgb_alivetime_att>(cam.value());

            // assign attributes to RoboCompCameraRGBDSimple::TImage
            rgb.image = rgb_data.value().get();
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

RoboCompCameraRGBDSimple::TDepth SpecificWorker::get_depth_from_G()
{
    // get head camera node
    auto cam = G->get_node(viriato_head_camera_name);
    if (cam.has_value())
    {
        // read depth data attributes from graph
        RoboCompCameraRGBDSimple::TDepth depth;
        try
        {
            auto depth_data = G->get_attrib_by_name<cam_depth_att>(cam.value());
            const auto width = G->get_attrib_by_name<cam_depth_width_att>(cam.value());
            const auto height = G->get_attrib_by_name<cam_depth_height_att>(cam.value());
            const auto cam_id = G->get_attrib_by_name<cam_depth_cameraID_att>(cam.value());
            const auto focalx = G->get_attrib_by_name<cam_depth_focalx_att>(cam.value());
            const auto focaly = G->get_attrib_by_name<cam_depth_focaly_att>(cam.value());
            const auto depth_factor = G->get_attrib_by_name<cam_depthFactor_att>(cam.value());
            const auto alivetime = G->get_attrib_by_name<cam_depth_alivetime_att>(cam.value());

            // assign attributes to RoboCompCameraRGBDSimple::TDepth
            depth.depth = depth_data.value();
            depth.width = width.value();
            depth.height = height.value();
            depth.cameraID = cam_id.value();
            depth.focalx = focalx.value();
            depth.focaly = focaly.value();
            depth.depthFactor = depth_factor.value(); // set to 0.1 for viriato_head_camera_sensor
            depth.alivetime = alivetime.value();

            return depth;
        }
        catch(const std::exception& e)
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

std::vector<std::vector<float>> SpecificWorker::get_camera_intrinsics()
{
    // get head camera node
    auto cam = G->get_node(viriato_head_camera_name);
    if (cam.has_value())
    {
        try
        {
            // read intrinsic parameters from G
            const auto width = G->get_attrib_by_name<cam_rgb_width_att>(cam.value());
            const auto height = G->get_attrib_by_name<cam_rgb_height_att>(cam.value());
            const auto focalx = G->get_attrib_by_name<cam_rgb_focalx_att>(cam.value());
            const auto focaly = G->get_attrib_by_name<cam_rgb_focaly_att>(cam.value());
            
            // define camera intrinsic matrix
            std::vector<std::vector<float>> intrinsic_mat;
            intrinsic_mat.push_back(std::vector<float>{static_cast<float>(focalx.value()), 0.0, static_cast<float>(width.value())/2});
            intrinsic_mat.push_back(std::vector<float>{0.0, static_cast<float>(focaly.value()), static_cast<float>(height.value())/2});
            intrinsic_mat.push_back(std::vector<float>{0.0, 0.0, 1.0});

            return intrinsic_mat;
        }
        catch(const std::exception& e)
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

/////////////////////////////////////////////////////////////////
//                     G injection utilities
/////////////////////////////////////////////////////////////////

void SpecificWorker::inject_estimated_poses(RoboCompObjectPoseEstimationRGBD::PoseType poses)
{
    // get a copy of world node
    auto world = G->get_node(world_name);
    // loop over each estimated object pose
    for (auto pose : poses)
    {
        if (pose.objectname.compare(grasp_object) == 0)
        {
            std::cout << "Target object '" << pose.objectname << "' detected" << std::endl;

            // convert quaternions into euler angles
            vector<float> quat{pose.qx, pose.qy, pose.qz, pose.qw};
            vector<float> angles = this->quat_to_euler(quat);

            // re-project estimated poses into world coordinates
            Eigen::Matrix<double, 6, 1> orig_point;
            orig_point << pose.x, pose.y, pose.z, angles.at(0), angles.at(1), angles.at(2);
            auto final_pose = inner_eigen->transform_axis(world_name, orig_point, viriato_head_camera_name);

            // get object node id (if exists)
            auto id = G->get_id_from_name(pose.objectname);

            // check whether object node already exists
            auto object_node = G->get_node(pose.objectname);
            if (!object_node.has_value()) // if node doesn't exist
            {
                // define object node
                DSR::Node object = DSR::Node();
                object.type("mesh");
                object.agent_id(agent_id);
                object.name(pose.objectname);

                // inject object node into graph
                id = G->insert_node(object);

                // check whether node is inserted or not
                if (id.has_value())
                {
                    std::cout << "Node inserted successfully -> " << id.value() << ":" << G->get_name_from_id(id.value()).value() << std::endl;
                }
                else
                {
                    std::cout << "Failed to insert node!" << std::endl;
                }
            }

            // inject estimated object pose into graph
            vector<float> trans{static_cast<float>(final_pose.value()(0,0)), static_cast<float>(final_pose.value()(1,0)), static_cast<float>(final_pose.value()(2,0))};
            vector<float> rot{static_cast<float>(final_pose.value()(3,0)), static_cast<float>(final_pose.value()(4,0)), static_cast<float>(final_pose.value()(5,0))};
            G->insert_or_assign_edge_RT(world.value(), id.value(), trans, rot);

            // ignore rest of objects
            break;
        }
    }
}

void SpecificWorker::update_node_slot(const std::int32_t id, const std::string &type)
{
    // plan dummy targets for the arm to follow
    if (type == left_hand_type)
    {
        // get world, grasp object and arm nodes
        auto world_node = G->get_node(world_name);
        auto tip_node = G->get_node(viriato_left_arm_tip_name);
        auto object_node = G->get_node(grasp_object);

        if (world_node.has_value() && tip_node.has_value() && object_node.has_value())
        {
            std::cout << "Plan arm's dummy targets" << std::endl;

            // get required nodes poses
            auto object_pose = inner_eigen->transform_axis(world_name, grasp_object);
            auto tip_pose = inner_eigen->transform_axis(world_name, viriato_left_arm_tip_name);

            // get euclidean distance between arm and required object (ignoring distance along z-axis)
            float arm_object_dist = sqrt(pow(object_pose.value()(0)-tip_pose.value()(0), 2.0) + pow(object_pose.value()(1)-tip_pose.value()(1), 2.0));

            // check whether required object is within arm's reach
            if (arm_object_dist >= 0.5)
            {
                // plan a dummy target closer to the object (planning is done on multiple stages | factor = 0.2)
                vector<float> tip_vec(tip_pose.value().data(), tip_pose.value().data() + tip_pose.value().rows() * tip_pose.value().cols());
                vector<float> object_vec(object_pose.value().data(), object_pose.value().data() + object_pose.value().rows() * object_pose.value().cols());

                vector<float> dummy_pose = this->interpolate_trans(tip_vec, object_vec, 0.2); // interpolate dummy target position
                dummy_pose.insert(dummy_pose.end(), tip_vec.begin()+3, tip_vec.end()); // set dummy target rotation with object rotation

                // update arm tip pose attribute with the next desired pose (based on actual current pose)
                G->add_or_modify_attrib_local<viriato_arm_tip_target_att>(tip_node.value(), dummy_pose);

                // check whether the arm target reaches the object
                if (dummy_pose == object_vec)
                {
                    std::cout << "The arm has reached the target object" << std::endl;
                }
            }
        }
    }
}

/////////////////////////////////////////////////////////////////
//                     Geometry utilities
/////////////////////////////////////////////////////////////////

vector<float> SpecificWorker::quat_to_euler(vector<float> quat)
{
    // euler angles vector
    vector<float> angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (quat.at(3) * quat.at(0) + quat.at(1) * quat.at(2));
    double cosr_cosp = 1 - 2 * ( quat.at(0) *  quat.at(0) +  quat.at(1) *  quat.at(1));
    angles.push_back(std::atan2(sinr_cosp, cosr_cosp));

    // pitch (y-axis rotation)
    double sinp = 2 * (quat.at(3) * quat.at(1) - quat.at(2) * quat.at(0));
    if (std::abs(sinp) >= 1)
        angles.push_back(std::copysign(M_PI / 2, sinp)); // use 90 degrees if out of range
    else
        angles.push_back(std::asin(sinp));

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (quat.at(3) * quat.at(2) + quat.at(0) * quat.at(1));
    double cosy_cosp = 1 - 2 * (quat.at(1) * quat.at(1) + quat.at(2) * quat.at(2));
    angles.push_back(std::atan2(siny_cosp, cosy_cosp));

    return angles;
}

vector<vector<float>> SpecificWorker::quat_to_rotm(vector<float> quat)
{
    // rotation matrix
    vector<vector<float>> rot_mat;

    // insert rotation matrix row one by one
    rot_mat.push_back(vector<float>{static_cast<float>(pow(quat.at(3),2.0)+pow(quat.at(0),2.0)-pow(quat.at(1),2.0)-pow(quat.at(2),2.0)),
                                    2*quat.at(0)*quat.at(1)-2*quat.at(2)*quat.at(3),
                                    2*quat.at(0)*quat.at(2)+2*quat.at(1)*quat.at(3)});
    rot_mat.push_back(vector<float>{2*quat.at(0)*quat.at(1)+2*quat.at(2)*quat.at(3),
                                    static_cast<float>(pow(quat.at(3),2.0)-pow(quat.at(0),2.0)+pow(quat.at(1),2.0)-pow(quat.at(2),2.0)),
                                    2*quat.at(1)*quat.at(2)+2*quat.at(0)*quat.at(3)});
    rot_mat.push_back(vector<float>{2*quat.at(0)*quat.at(2)-2*quat.at(1)*quat.at(3),
                                    2*quat.at(1)*quat.at(2)-2*quat.at(0)*quat.at(3),
                                    static_cast<float>(pow(quat.at(3),2.0)-pow(quat.at(0),2.0)-pow(quat.at(1),2.0)+pow(quat.at(2),2.0))});

    return rot_mat;
}

vector<float> SpecificWorker::interpolate_trans(vector<float> src, vector<float> dest, float factor)
{
    // interpolate between the source and destination positions with the given factor
    float interp_x = src.at(0) + (dest.at(0)-src.at(0)) * factor;
    float interp_y = src.at(1) + (dest.at(1)-src.at(1)) * factor;
    float interp_z = src.at(2) + (dest.at(2)-src.at(2)) * factor;
    vector<float> interp_trans{interp_x, interp_y, interp_z};

    float final_pose_dist = sqrt(pow(dest.at(0)-interp_trans.at(0), 2.0) + 
                                pow(dest.at(1)-interp_trans.at(1), 2.0) + 
                                pow(dest.at(2)-interp_trans.at(2), 2.0));

    if (final_pose_dist <= 0.01)
    {
        return dest;
    }

    return interp_trans;
}

vector<vector<float>> SpecificWorker::project_vertices(vector<vector<float>> vertices, vector<vector<float>> rot, vector<float> trans, vector<vector<float>> intrinsics)
{
    // initialize eigen vectors and matrices
    Eigen::Matrix3d intrinsic_mat;
    intrinsic_mat << intrinsics.at(0).at(0), intrinsics.at(0).at(1), intrinsics.at(0).at(2),
                    intrinsics.at(1).at(0), intrinsics.at(1).at(1), intrinsics.at(1).at(2),
                    intrinsics.at(2).at(0), intrinsics.at(2).at(1), intrinsics.at(2).at(2);

    Eigen::Vector3d trans_vec(trans.at(0), trans.at(1), trans.at(2));

    Eigen::Matrix3d rot_mat;
    rot_mat << rot.at(0).at(0), rot.at(0).at(1), rot.at(0).at(2),
            rot.at(1).at(0), rot.at(1).at(1), rot.at(1).at(2),
            rot.at(2).at(0), rot.at(2).at(1), rot.at(2).at(2);

    Eigen::MatrixXd vertices_mat(vertices.size(), 3);
    for (int i = 0; i < vertices.size(); i++)
    {
        vertices_mat.row(i) = Eigen::VectorXd::Map(reinterpret_cast<const double *>(&vertices.at(i).at(0)), 3);
    }

    // perform vertices transformation
    Eigen::MatrixXd rot_vertices = rot_mat*vertices_mat.transpose();
    rot_vertices.colwise() += trans_vec;
    Eigen::MatrixXd proj_vertices = intrinsic_mat*rot_vertices;

    // prepare and normalize 2D vertices from 3D vertices
    vector<vector<float>> pixel_vertices;
    for (int i = 0; i < vertices.size(); i++)
    {
        Eigen::Vector3d vertex_3d = proj_vertices.col(i);
        pixel_vertices.push_back(vector<float>{static_cast<float>(vertex_3d(0)/(vertex_3d(2)+1e-5)), static_cast<float>(vertex_3d(1)/(vertex_3d(2)+1e-5))});
    }

    return pixel_vertices;
}

/////////////////////////////////////////////////////////////////
//                     IO utilities
/////////////////////////////////////////////////////////////////

std::map<std::string,std::vector<std::vector<float>>> SpecificWorker::read_pcl_from_file()
{
    // read objects point cloud from text files and save them to std::map
    std::vector<std::string> filenames;
    std::map<std::string, std::vector<std::vector<float>>> data;

    if(boost::filesystem::is_directory("objects-pcl"))
    {
        for(auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator("objects-pcl"), {}))
        {
            filenames.push_back(entry.path().string());
        }
    }

    for (auto filename : filenames)
    {
        std::ifstream file(filename);
        std::string line;
        std::vector<std::vector<float>> pcl;

        while (std::getline(file, line))
        {
            float value;
            std::stringstream ss(line);
            std::vector<float> point;

            while (ss >> value)
            {
                point.push_back(value);
            }
            pcl.push_back(point);
        }

        data.insert({filename.substr(12, filename.size()-16), pcl});
    }

    return data;
}

/////////////////////////////////////////////////////////////////
//                     Display utilities
/////////////////////////////////////////////////////////////////

void SpecificWorker::show_image(cv::Mat &img, RoboCompObjectPoseEstimationRGBD::PoseType poses)
{
    // visualize the pose of the target object to be grasped
    for (auto pose : poses)
    {
        if (pose.objectname.compare(grasp_object) == 0)
        {
            // read object point cloud and pose
            std::vector<std::vector<float>> obj_pcl = objects_pcl.at(pose.objectname);
            std::vector<float> obj_trans = std::vector<float>{pose.x, pose.y, pose.z};
            std::vector<std::vector<float>> obj_rot = this->quat_to_rotm(std::vector<float>{pose.qx, pose.qy, pose.qz, pose.qw});
            // get camera intrinsic matrix
            std::vector<std::vector<float>> intrinsic_mat = this->get_camera_intrinsics();
            // project point cloud into pixel space
            std::vector<std::vector<float>> proj_vertices = this->project_vertices(obj_pcl, obj_rot, obj_trans, intrinsic_mat);
            // draw projected point cloud on RGB image
            this->draw_vertices(img, proj_vertices);
        }
    }
    // show final RGB image
    cv::imshow("DNN-Estimated Poses", img);
    cv::waitKey(1);
    // create QImage and display it on the widget
    auto pix = QPixmap::fromImage(QImage(img.data, img.cols, img.rows, QImage::Format_RGB888));
    custom_widget.rgb_image->setPixmap(pix);
}

void SpecificWorker::draw_vertices(cv::Mat &img, std::vector<std::vector<float>> vertices_2d)
{
    // draw 2D projected points on RGB image
    for (auto vertex : vertices_2d)
    {
        int x = std::max(0, std::min(img.cols, static_cast<int>(vertex.at(0))));
        int y = std::max(0, std::min(img.rows, static_cast<int>(vertex.at(1))));
        cv::circle(img, cv::Point(x,y), 1, cv::Scalar(255,0,0), cv::FILLED);
    }
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
// From the RoboCompObjectPoseEstimationRGBD you can call this methods:
// this->objectposeestimationrgbd_proxy->getObjectPose(...)

/**************************************/
// From the RoboCompObjectPoseEstimationRGBD you can use this types:
// RoboCompObjectPoseEstimationRGBD::ObjectPose
