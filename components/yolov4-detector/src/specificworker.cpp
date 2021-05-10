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
#include <cppitertools/product.hpp>
#include <cppitertools/zip.hpp>

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
//        connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::add_or_assign_edge_slot);

//        connect(G.get(), &DSR::DSRGraph::update_attrs_signal, this, &SpecificWorker::add_or_assign_attrs_slot);
//        connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
//        connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);

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
    const auto g_depth = depth_buffer.try_get();
    const auto g_image = rgb_buffer.try_get();
    auto robot = G->get_node(robot_name);
    if( g_image.has_value() and g_depth.has_value())
    {
        compute_visible_objects();
        cv::Mat imgyolo = g_image.value();
        std::vector<float> depth_array = g_depth.value();
        std::vector<Box> real_objects = process_image_with_yolo(imgyolo, depth_array);
        std::vector<Box> synth_objects = get_visible_objects_from_graph();
        auto lists_after_match = match_lists(real_objects, synth_objects, depth_array);
        auto lists_after_add = add_new_objects(lists_after_match);
        show_image(imgyolo, real_objects, synth_objects);
        //auto lists_after_delete = delete_unseen_objects(lists_after_add);
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::tuple<SpecificWorker::Boxes, SpecificWorker::Boxes> SpecificWorker::match_lists(Boxes &real_objects,
                                                                                     Boxes &synth_objects,
                                                                                     const std::vector<float> &depth_array)
{
    auto world_node = G->get_node(world_name);
    for (auto&& [b_real, b_synth] : iter::product(real_objects, synth_objects))
    {
        //std::cout << __FUNCTION__ << " trying match between " << b_synth.name << " and " << b_real.name << std::endl;
        if (b_synth.name.find(b_real.name, 0) == 0 or b_synth.name.find("glass", 0) == 0)
        {
//            std::cout << __FUNCTION__ << " potential match " << std::endl;
//            std::cout << "\t " << b_synth.top << " " << b_synth.left << " " << b_synth.right << " " << b_synth.bot << std::endl;
//            std::cout << "\t " << b_real.top << " " << b_real.left << " " << b_real.right << " " << b_real.bot << std::endl;
            if (both_boxes_match(b_synth, b_real))
            {
                std::cout << __FUNCTION__ << " success match between " << b_synth.name << " and " << b_real.name << std::endl;
                auto node = G->get_node(b_synth.name);
                auto parent = G->get_parent_node(node.value());
                auto edge = rt_api->get_edge_RT(parent.value(), node->id()).value();
                G->modify_attrib_local<rt_translation_att>(edge, std::vector<float>{b_real.Tx, b_real.Ty, b_real.Tz});
                G->insert_or_assign_edge(edge);
                b_real.print("Real Box");
                b_synth.print("Synth Box");
            }
        }
    }
    // remove matched elements
    real_objects.erase(std::remove_if(real_objects.begin(),real_objects.end(),
                          [](Box const &p) { return p.match == true; }), real_objects.end());
    synth_objects.erase(std::remove_if(synth_objects.begin(),synth_objects.end(),
                                      [](Box const &p) { return p.match == true; }), synth_objects.end());

    return(std::make_tuple(real_objects, synth_objects));
}
std::tuple<SpecificWorker::Boxes, SpecificWorker::Boxes>
        SpecificWorker::add_new_objects(const std::tuple<SpecificWorker::Boxes, SpecificWorker::Boxes> &lists_after_match)
{
    auto world_node = G->get_node(world_name);
    auto [real_objects, synth_objects] = lists_after_match;

    const std::map<std::string, std::vector<float>> known_object_types{{"glass",     {100, 100, 200}},
                                                                       {"cup",       {100, 100, 200}},
                                                                       {"microwave", {300, 250, 350}},
                                                                       {"plant",     {350, 350, 600}}};   // LIST OF KNOWN OBJECT TYPES

    for (auto&& [b_real, b_synth] : iter::product(real_objects, synth_objects))
    {
        for(const auto &[known_object, size] : known_object_types)
        {
            DSR::Node object_node;
            if (b_real.name.find(known_object, 0) == 0)
            {
                if(known_object == "glass")
                    object_node = DSR::Node::create<glass_node_type>(b_real.name);
                else if (known_object == "cup")
                    object_node = DSR::Node::create<cup_node_type>(b_real.name);

                // decide here who is going to be the parent
                auto parent_node = G->get_node(world_name);
                if(not parent_node.has_value()) { qWarning() << __FUNCTION__ << "No parent node " << QString::fromStdString(world_name) << " found "; continue; }
                G->add_or_modify_attrib_local<parent_att>(object_node, parent_node.value().id());
                G->add_or_modify_attrib_local<level_att>(object_node, G->get_node_level(parent_node.value()).value()+1);
                // object size in an object centered reference system: looking from the robot, center at roi center. x+ to the right, y+ upwards, z+ away from robot
                G->add_or_modify_attrib_local<obj_depth_att>(object_node, 1);  // get correct size
                G->add_or_modify_attrib_local<obj_height_att>(object_node, 1);  // get correct size
                G->add_or_modify_attrib_local<obj_width_att>(object_node, 1);  // get correct size

                if (std::optional<int> id = G->insert_node(object_node); id.has_value())
                {
                    std::cout << __FUNCTION__ << "object id " << object_node.id() << endl;
                    DSR::Edge edge = DSR::Edge::create<RT_edge_type>(world_node.value().id(), object_node.id());
                    G->add_or_modify_attrib_local<rt_translation_att>(edge, std::vector<float>{b_real.Tx, b_real.Ty, b_real.Tz});
                    G->add_or_modify_attrib_local<rt_rotation_euler_xyz_att>(edge, std::vector<float>{0., 0., 0.});
                    G->insert_or_assign_edge(edge);
                    G->update_node(world_node.value());
                    // mark real_box to remove from real_objects lists
                    b_real.marked_for_delete = true;
                    qInfo() << __FUNCTION__ << "Created node " << QString::fromStdString(b_real.name);
                }
                else
                    qWarning() << "Object " << QString::fromStdString(b_real.name) << " could NOT be created";
            }
        }
    }
    // remove marked objects
    real_objects.erase(std::remove_if(real_objects.begin(),real_objects.end(),[](Box const &p) { return p.marked_for_delete == true; }), real_objects.end());
    return std::make_tuple(real_objects, synth_objects);
}

std::tuple<SpecificWorker::Boxes, SpecificWorker::Boxes>
        SpecificWorker::delete_unseen_objects(const std::tuple<SpecificWorker::Boxes, SpecificWorker::Boxes> &lists_after_add)
{

}
std::vector<SpecificWorker::Box> SpecificWorker::get_visible_objects_from_graph()
{
    std::vector<Box> boxes;
    const auto visibles = G->get_edges_by_type("visible");  //Should taka a type instead of a string
    //qInfo() << __FUNCTION__ << visibles.size();
    for(const auto &edge: visibles)
    {
        if(auto att = G->get_attrib_by_name<projected_bounding_box_att>(edge); att.has_value())
        {
            auto object = G->get_node(edge.to());
            Box box;
            box.left = (int)att.value().get()[0]; box.top=(int)att.value().get()[1]; box.right=(int)att.value().get()[2]; box.bot=(int)att.value().get()[3];
            box.name = object.value().name();
            box.prob = 100;
            box.match = false;
            box.visible = true;
            if(auto t_world = inner_eigen->transform(world_name, object.value().name()); t_world.has_value())
            {
                if (auto t_camera = inner_eigen->transform(camera_name, object.value().name()); t_camera.has_value())
                {
                    box.depth = t_camera.value().norm();
                    box.Cx = t_camera.value().x();
                    box.Cy = t_camera.value().y();
                    box.Cz = t_camera.value().z();
                    box.Tx = t_world.value().x();
                    box.Ty = t_world.value().y();
                    box.Tz = t_world.value().z();
                    boxes.emplace_back(box);  //move
                }
            }
        }
    }
    return boxes;
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
std::vector<SpecificWorker::Box> SpecificWorker::process_image_with_yolo(const cv::Mat &img, const std::vector<float> &depth_array)
{
    // get detections from RGB image
    image_t yolo_img = createImage(img);
    std::vector<bbox_t> detections = ynets[0]->detect(yolo_img, 0.2, false);
    // process detected bounding boxes
    std::vector<Box> bboxes; bboxes.reserve(detections.size());
    for(const auto &d : detections)
    {
        Box box;
        int cls = d.obj_id;
        box.name = names.at(cls);
        box.left = d.x-yolo_img.w/2;
        box.right = d.x + d.w-yolo_img.w/2;
        box.top = d.y-yolo_img.h/2;
        box.bot = d.y + d.h-yolo_img.h/2;
        if (box.left < -yolo_img.w/2) box.left = -yolo_img.w/2;
        if (box.right > yolo_img.w/2 - 1) box.right = yolo_img.w/2 - 1;
        if (box.top < -yolo_img.h/2) box.top = -yolo_img.h/2;
        if (box.bot > yolo_img.h/2 - 1) box.bot = yolo_img.h/2 - 1;
        box.prob = d.prob*100;
        box.visible = true;
        box.match = false;
        int wDepth = cam_api->get_width();
        int hDepth = cam_api->get_height();
        int BDleft = box.left+wDepth/2;
        if (BDleft<0) BDleft = 0;
        int BDright = box.right+wDepth/2;
        if (BDright>=wDepth) BDright = wDepth-1;
        int BDtop = box.top+hDepth/2;
        if (BDtop<0) BDtop = 0;
        int BDbot = box.bot+hDepth/2;
        if (BDbot>=hDepth) BDbot = hDepth-1;


        auto tp = cam_api->get_roi_depth(depth_array, Eigen::AlignedBox<float, 2>(Eigen::Vector2f(BDleft, BDbot),
                                                                                  Eigen::Vector2f(BDright, BDtop)));
        if (tp.has_value())
        {
            auto[x, y, z] = tp.value();
            if (auto t_world = inner_eigen->transform(world_name, Mat::Vector3d(x, y, z), camera_name); t_world.has_value())
            {
                //std::cout << "POS 3D "<<names.at(cls)<<" "<<t_world[0]<<" "<<t_world[1]<<" "<<t_world[2] << std::endl;
                box.depth = (float) Mat::Vector3d(x, y, z).norm();  // center ROI
                box.Cx = x;
                box.Cy = y;
                box.Cz = z;
                box.Tx = t_world.value().x();
                box.Ty = t_world.value().y();
                box.Tz = t_world.value().z();
                bboxes.push_back(box);
            }
        }
    }
    //qInfo() << __FILE__ << __FUNCTION__ << "LABELS " << bboxes.size();
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
            cout<<"processing synthetic objects "<<object_name<<endl;
            
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
            box.match = false;
            if( box.left>=0 and box.right<YOLO_IMG_SIZE and box.top>=0 and box.bot<YOLO_IMG_SIZE)
                box.visible = true;
            else
                box.visible = false;
            if( auto d = rt_api->get_translation(cam_api->get_id(), object.value().id()); d.has_value())
                box.depth = d.value().norm();
            else box.depth = 0;
            synth_box.push_back(box);

            auto node = G->get_node(object_name);
            G->add_attrib_local<obj_visible_att>(node.value(), (int) box.visible);
            G->update_node(node.value());
            cout<<"end processing synthetic objects "<<endl;
        }
    }
    return synth_box;
}
bool SpecificWorker::both_boxes_match(Box &real_box, Box &synth_box)
{
    //A rectangle with the real object is created
    QRect r(QPoint(real_box.top, real_box.left), QSize(real_box.width(), real_box.height()));
    //A rectangle with the sythetic object is created
    QRect rs(QPoint(synth_box.top, synth_box.left), QSize(synth_box.width(), synth_box.height()));
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
        qInfo() << __FUNCTION__ << " area " << area << "error " << error.manhattanLength() << "rs widrh " << rs.width()*3;
        real_box.match = true; synth_box.match = true;
        real_box.area = area; real_box.match_error = error.manhattanLength();
        synth_box.area = area; synth_box.match_error = error.manhattanLength();
        return true;
    }
    else
        return false;
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
void SpecificWorker::compute_visible_objects()
{
    std::vector<Box> synth_box;
    auto object_nodes = G->get_nodes_by_type(glass_type_name);
    auto cup_nodes = G->get_nodes_by_type(cup_type_name);
    object_nodes.insert(object_nodes.end(), cup_nodes.begin(), cup_nodes.end());

    // computer room's delimting polygon that contains the robot
    auto robot_node = G->get_node(robot_name);
    QPolygonF room_polygon;
    if( auto in_edges = G->get_node_edges_by_type(robot_node.value(), "in"); not in_edges.empty())
        if( auto room_node = G->get_node(in_edges.front().to()); room_node.has_value() )
        {
            auto polygon_x = G->get_attrib_by_name<delimiting_polygon_x_att>(room_node.value());
            auto polygon_y = G->get_attrib_by_name<delimiting_polygon_y_att>(room_node.value());
            if (polygon_x.has_value() and polygon_y.has_value())
                for (auto &&[px, py] : iter::zip(polygon_x.value().get(), polygon_y.value().get()))
                    room_polygon << QPointF(px, py);
        }
        else { qWarning() << __FUNCTION__ << "No room node found for robot"; return; }
    else { qWarning() << __FUNCTION__ << "No IN edge from robot to room"; return; }

    // project object
    auto c = YOLO_IMG_SIZE/2;
    for(auto &object : object_nodes)
    {
        const std::string &object_name = object.name();

        // check if object is in the same room as the robot
        auto object_pos = inner_eigen->transform(world_name, object_name);
        if(not room_polygon.containsPoint(QPointF(object_pos.value().x(), object_pos.value().y()), Qt::WindingFill))
                continue;

        // project corners of object's bounding box in the camera image plane
        // get object's bounding box from object's node
        std::vector<Mat::Vector2d> bb_in_camera(8);
        const float h = 150;
        bb_in_camera[0] = cam_api->project(inner_eigen->transform(camera_name, Mat::Vector3d(40,40,0), object_name).value(),0,0);
        bb_in_camera[1] = cam_api->project(inner_eigen->transform(camera_name, Eigen::Vector3d(-40,40,0), object_name).value(),0,0);
        bb_in_camera[2] = cam_api->project(inner_eigen->transform(camera_name, Eigen::Vector3d(40,-40,0), object_name).value(),0,0);
        bb_in_camera[3] = cam_api->project(inner_eigen->transform(camera_name, Eigen::Vector3d(-40,-40,0), object_name).value(),0,0);
        bb_in_camera[4] = cam_api->project(inner_eigen->transform(camera_name, Eigen::Vector3d(40, 40, h), object_name).value(),0,0);
        bb_in_camera[5] = cam_api->project(inner_eigen->transform(camera_name, Eigen::Vector3d(-40,40, h), object_name).value(),0,0);
        bb_in_camera[6] = cam_api->project(inner_eigen->transform(camera_name, Eigen::Vector3d(40, -40, h), object_name).value(),0,0);
        bb_in_camera[7] = cam_api->project(inner_eigen->transform(camera_name, Eigen::Vector3d(-40, -40,h), object_name).value(),0,0);

        // Compute a 2D projected bounding box
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
        box.match = false;

        // check if is inside the image
        if( box.left>=-c and box.right<c and box.top>=-c and box.bot<c)
        {
            box.visible = true;
            if (auto d = rt_api->get_translation(cam_api->get_id(), object.id()); d.has_value())
                box.depth = d.value().norm();
            else box.depth = 0;
            synth_box.push_back(box);

            // update edges
            auto edges = G->get_edges_to_id(object.id());
            if (auto it = std::ranges::find_if(edges, [](const auto &e) { return e.type() == "visible"; }); it == edges.end())  //not found
            {
                //add edge
                DSR::Edge new_edge = DSR::Edge::create<visible_edge_type>(cam_api->get_id(), object.id());
                G->add_or_modify_attrib_local<projected_bounding_box_att>(new_edge, std::vector<float>{(float)box.left, (float)box.top, (float)box.right, (float)box.bot});
                G->insert_or_assign_edge(new_edge);
            }
            else    // already there. update projection bounding box
            {
                if( auto old_edge = G->get_edge(cam_api->get_id(), object.id(), "visible"); old_edge.has_value())
                    G->insert_or_assign_attrib<projected_bounding_box_att>(old_edge.value(), std::vector<float>{(float)box.left, (float)box.top, (float)box.right, (float)box.bot});
                else
                    qWarning() << __FUNCTION__ << "No VISIBLE edge going from camera to " << QString::fromStdString(object.name());
            }
        }
        else  // remove edge
        {
            G->delete_edge(cam_api->get_id(), object.id(), "visible");
        }
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
            auto left = box.left+YOLO_IMG_SIZE/2;
            auto right = box.right+YOLO_IMG_SIZE/2;
            auto top = box.top+YOLO_IMG_SIZE/2;
            auto bot = box.bot+YOLO_IMG_SIZE/2;
            auto p1 = cv::Point(left, top);
            auto p2 = cv::Point(right, bot);
            auto offset = int((bot - top) / 2);
            auto pt = cv::Point(left + offset, top + offset);
            cv::rectangle(imgdst, p1, p2, cv::Scalar(0, 0, 255), 4);
            auto font = cv::FONT_HERSHEY_SIMPLEX;
            cv::putText(imgdst, box.name + " " + std::to_string(int(box.prob)) + "%", pt, font, 0.8, cv::Scalar(0, 255, 255), 2);
        }
    }
    for(const auto &box : synth_boxes)
    {
        auto left = box.left+YOLO_IMG_SIZE/2;
        auto right = box.right+YOLO_IMG_SIZE/2;
        auto top = box.top+YOLO_IMG_SIZE/2;
        auto bot = box.bot+YOLO_IMG_SIZE/2;
        auto p1 = cv::Point(left, top);
        auto p2 = cv::Point(right, bot);
        auto offset = int((bot - top) / 2);
        auto pt = cv::Point(left + offset, top + offset);
        cv::rectangle(imgdst, p1, p2, cv::Scalar(0, 255, 0), 4);
        auto font = cv::FONT_HERSHEY_SIMPLEX;
        cv::putText(imgdst, box.name + " " + std::to_string(int(box.prob)) + "%", pt, font, 0.8, cv::Scalar(0, 255, 0), 2);
        //cout<<box.name<<" "<<"p1 "<<p1.x<<" "<<p1.y<<endl;
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
        else qWarning() << __FUNCTION__ << "No camera_node found in G";
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

void SpecificWorker::add_or_assign_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type)
{
    if(from == G->get_node(world_name).value().id() and to == G->get_node(robot_name).value().id() and type == "RT")
            compute_visible_objects();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
