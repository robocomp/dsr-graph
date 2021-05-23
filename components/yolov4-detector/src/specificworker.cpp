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
	//G->write_to_json_file("./"+agent_name+".json");
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
        // connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::add_or_assign_edge_slot);
        // connect(G.get(), &DSR::DSRGraph::update_attrs_signal, this, &SpecificWorker::add_or_assign_attrs_slot);
        // connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
        // connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);

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
        if (auto cam_node = G->get_node(viriato_head_camera_name); cam_node.has_value())
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
        initialize_combobox();
        connect(custom_widget.clearButton, SIGNAL(clicked()), this, SLOT(clear_button_slot()));
        connect(custom_widget.comboBox, SIGNAL(activated(int)), this, SLOT(change_attention_object_slot(int)));

        // clear al attention_action edges
        clear_all_attention_edges();

        this->Period = 60;
        timer.start(Period);
        READY_TO_GO = true;
	}
}
Detector* SpecificWorker::init_detector()
{
    // read objects yolo_names from file
    std::ifstream file(names_file);
    for(std::string line; getline(file, line);) yolo_names.push_back(line);
    // initialize YOLOv4 detector
    Detector* detector = new Detector(cfg_file, weights_file);

    // PROTO SEMANTIC MEMORY. Standard size of objects in object's reference frame.
    //          now: center at roi center. x,y,z as in world coordinate system
    //          should be: select face pointing at camera and assign Y+, X+ to the right and Z+ upwards
    //
    //          other info to be added here:
    //              - typical mesh as a path to an IVE, OSG or OBJ file
    //              - 2D shape to be shown in 2D view
    //              - estimated mass
    //              - meaning predicates related to:
    //                  position: "usually found in X"
    //                  function: "usualy used for Y"
    //                  changes of state after actions: "it brakes if it falls", "it fills if poured with a liquid"

    known_object_types.insert( {"glass",        {80, 100, 80}});
    known_object_types.insert( {"cup",          {80, 100, 80}});
    known_object_types.insert( {"microwave",    {450, 250, 350}});
    known_object_types.insert( {"plant",        {500, 900, 500}});
    known_object_types.insert( {"person",       {350, 1700, 350}});
    known_object_types.insert( {"vase",         {300, 300, 350}});
    known_object_types.insert( {"oven",         {400, 100, 400}});
    known_object_types.insert( {"refrigerator", {600, 1600, 600}});
    //known_object_types.insert( {"apple",        {80, 80, 80}});
    return detector;
}
////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::compute()
{
    auto begin = myclock::now();
    const auto g_depth = depth_buffer.try_get();
    const auto g_image = rgb_buffer.try_get();
    auto robot = G->get_node(robot_name);
    if( g_image.has_value() and g_depth.has_value())
    {
        compute_visible_objects();
        //std::cout << "Visible objects = " << std::chrono::duration_cast<std::chrono::milliseconds>(myclock::now() - begin).count() << "[ms]" << std::endl;
        cv::Mat imgyolo = g_image.value();
        std::vector<float> depth_array = g_depth.value();
        std::vector<Box> real_objects = process_image_with_yolo(imgyolo, depth_array);
        std::vector<Box> synth_objects = get_visible_objects_from_graph();
        show_image(imgyolo, real_objects, synth_objects);
        auto lists_after_match = match_lists(real_objects, synth_objects, depth_array);
        auto lists_after_add = add_new_objects(lists_after_match);
        auto lists_after_delete = delete_unseen_objects(lists_after_add);
        auto &[a,b] = lists_after_delete;
        //qInfo() << __FUNCTION__ << "real: " << a.size() << " synth:" << b.size();
    }
    //std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(myclock::now() - begin).count() << "[ms]" << std::endl;
    fps.print("FPS:");
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::tuple<SpecificWorker::Boxes, SpecificWorker::Boxes> SpecificWorker::match_lists(Boxes &real_objects,
                                                                                     Boxes &synth_objects,
                                                                                     const std::vector<float> &depth_array)
{
    int count = 0;
    auto world_node = G->get_node(world_name);
    for (auto&& [b_real, b_synth] : iter::product(real_objects, synth_objects))
    {
        //std::cout << __FUNCTION__ << " trying match between " << b_synth.name << " and " << b_real.name << std::endl;
        if (b_synth.name.find(b_real.name, 0) == 0) // or b_synth.name.find("glass", 0) == 0)
        {
            //std::cout << __FUNCTION__ << " potential match " << std::endl;
            //std::cout << "\t " << b_synth.top << " " << b_synth.left << " " << b_synth.right << " " << b_synth.bot << std::endl;
            //std::cout << "\t " << b_real.top << " " << b_real.left << " " << b_real.right << " " << b_real.bot << std::endl;
            if (both_boxes_match(b_synth, b_real))
            {
                //std::cout << __FUNCTION__ << " success match between " << b_synth.name << " and " << b_real.name << std::endl;
                b_synth.match = true; b_real.match = true;
                auto synth_node = G->get_node(b_synth.name);
                auto parent = G->get_parent_node(synth_node.value());
                auto edge = rt_api->get_edge_RT(parent.value(), synth_node->id()).value();
                G->add_or_modify_attrib_local<unseen_time_att>(synth_node.value(), 0);  // reset unseen counter
                G->modify_attrib_local<rt_translation_att>(edge, std::vector<float>{b_real.Tx, b_real.Ty, b_real.Tz});
                // const auto &[width, depth, height] = estimate_object_size_through_projection_optimization(b_synth, b_real);
                G->insert_or_assign_edge(edge);
                G->update_node(synth_node.value());
                //b_real.print("Real Box");
                //b_synth.print("Synth Box");
                //qInfo() << __FUNCTION__ << " Matched objects:" << ++count;
            }
        }
    }
    // remove matched elements from both lists
    real_objects.erase(std::remove_if(real_objects.begin(),real_objects.end(),
                           [](Box const &p) { return p.match == true; }), real_objects.end());
    synth_objects.erase(std::remove_if(synth_objects.begin(),synth_objects.end(),
                                       [](Box const &p) { return p.match == true; }), synth_objects.end());
    return(std::make_tuple(real_objects, synth_objects));
}
std::tuple<SpecificWorker::Boxes, SpecificWorker::Boxes>
        SpecificWorker::add_new_objects(std::tuple<SpecificWorker::Boxes, SpecificWorker::Boxes> &lists_after_match)
{
    int count = 0;
    auto world_node = G->get_node(world_name);
    auto &[real_objects, synth_objects] = lists_after_match;

    auto robot_node = G->get_node(robot_name);
    QPolygonF room_polygon;
    if( auto in_edges = G->get_node_edges_by_type(robot_node.value(), in_type_name); not in_edges.empty())
        if( auto room_node = G->get_node(in_edges.front().to()); room_node.has_value() )   // THIS CAN BE MOVED TO the SLOTs and create a list
        {
            auto polygon_x = G->get_attrib_by_name<delimiting_polygon_x_att>(room_node.value());
            auto polygon_y = G->get_attrib_by_name<delimiting_polygon_y_att>(room_node.value());
            if (polygon_x.has_value() and polygon_y.has_value())
                for (auto &&[px, py] : iter::zip(polygon_x.value().get(), polygon_y.value().get()))
                    room_polygon << QPointF(px, py);
        }
        else { qWarning() << __FUNCTION__ << "No room node found for robot";  return lists_after_match;}
    else { qWarning() << __FUNCTION__ << "No IN edge from robot to room";  return lists_after_match;}


    for (auto&& b_real : real_objects)
    {
        if (not room_polygon.containsPoint(QPointF(b_real.Tx, b_real.Ty), Qt::OddEvenFill))
                continue;

        DSR::Node object_node;
        // before actually creating the object, it has to be seen a minimun number of times at the same place
        if(real_object_is_stable(b_real, room_polygon))  // insert
        {
            object_node = create_node_with_type(b_real.type, b_real.name);
            auto size = known_object_types.at(b_real.type);

            // decide here who is going to be the parent
            auto parent_node = G->get_node(world_name);
            if (not parent_node.has_value())
            {
                qWarning() << __FUNCTION__ << "No parent node " << QString::fromStdString(world_name) << " found ";
                continue;
            }

            // complete attributes
            G->add_or_modify_attrib_local<parent_att>(object_node, parent_node.value().id());
            G->add_or_modify_attrib_local<level_att>(object_node, G->get_node_level(parent_node.value()).value() + 1);
            G->add_or_modify_attrib_local<obj_width_att>(object_node, size[0]);
            G->add_or_modify_attrib_local<obj_height_att>(object_node, size[1]);
            G->add_or_modify_attrib_local<obj_depth_att>(object_node, size[2]);
            const auto &[random_x, random_y] = get_random_position_to_draw_in_graph("object");
            G->add_or_modify_attrib_local<pos_x_att>(object_node, random_x);
            G->add_or_modify_attrib_local<pos_y_att>(object_node, random_y);

            if (std::optional<int> id = G->insert_node(object_node); id.has_value())
            {
                //std::cout << __FUNCTION__ << "object id " << object_node.id() << endl;
                //std::cout << __FUNCTION__ << "T " << b_real.Tx <<" "<< b_real.Ty << " " << b_real.Tz << endl;
                DSR::Edge edge = DSR::Edge::create<RT_edge_type>(world_node.value().id(), object_node.id());
                G->add_or_modify_attrib_local<rt_translation_att>(edge, std::vector<float>{b_real.Tx, b_real.Ty, b_real.Tz});
                G->add_or_modify_attrib_local<rt_rotation_euler_xyz_att>(edge, std::vector<float>{0., 0., 0.});
                G->insert_or_assign_edge(edge);
                G->update_node(world_node.value());
                b_real.marked_for_delete = true;
                qInfo() << __FUNCTION__ << "Created node " << QString::fromStdString(b_real.name);
                initialize_combobox();
            } else
                qWarning() << "Object " << QString::fromStdString(b_real.name) << " could NOT be created";
            qInfo() << __FUNCTION__ << " Added objects:" << ++count;
        }
    }
    // remove marked objects
    real_objects.erase(std::remove_if(real_objects.begin(),real_objects.end(),[](Box const &p) { return p.marked_for_delete == true; }), real_objects.end());
    return lists_after_match;
}

std::tuple<SpecificWorker::Boxes, SpecificWorker::Boxes>
        SpecificWorker::delete_unseen_objects(std::tuple<SpecificWorker::Boxes, SpecificWorker::Boxes> &lists_after_add)
{
    auto &[real_objects, synth_objects] = lists_after_add;
    for (auto&& b_synth : synth_objects)
    {
        if (not b_synth.match)   // objects in G not corroborated
        {
            if (auto object_node = G->get_node(b_synth.name); object_node.has_value())
            {
                if (auto unseen_time = G->get_attrib_by_name<unseen_time_att>(object_node.value()); unseen_time.has_value())
                {
                    if (unseen_time.value() > CONSTANTS.max_allowed_unseen_ticks)
                    {
                        G->delete_node(object_node->name());
                        b_synth.marked_for_delete = true;
                        initialize_combobox();
                    }
                    else  //update unseen time
                    {
                        G->add_or_modify_attrib_local<unseen_time_att>(object_node.value(), unseen_time.value() + 1);
                        G->update_node(object_node.value());
                    }
                }
                else
                {
                    G->add_or_modify_attrib_local<unseen_time_att>(object_node.value(), 1);
                    G->update_node(object_node.value());
                }
            }
        }
    }
    synth_objects.erase(std::remove_if(synth_objects.begin(),synth_objects.end(),[](Box const &p) { return p.marked_for_delete == true; }), synth_objects.end());
    return lists_after_add;
}
////////////////////////////////////////////////////////////////////////
std::tuple<float, float> SpecificWorker::get_random_position_to_draw_in_graph(const std::string &type)
{
    static std::random_device rd;
    static std::mt19937 mt(rd());

    float low_x_limit = -600, low_y_limit = -600, upper_x_limit = 600, upper_y_limit = 600;
    if(type == "object")
    {
        low_x_limit = -300;
        upper_x_limit = 0;
        low_y_limit = 0;
        upper_y_limit = 300;
    }
    std::uniform_real_distribution<double> dist_x(low_x_limit, upper_x_limit);
    std::uniform_real_distribution<double> dist_y(low_y_limit, upper_y_limit);

    return std::make_tuple(dist_x(mt), dist_y(mt));
}
bool SpecificWorker::real_object_is_stable(Box box, const QPolygonF &robot_room)     // a copy of Box
{
    static std::vector<Box> candidates;

    // remove those  in other room
    candidates.erase(std::remove_if(candidates.begin(), candidates.end(),[robot_room](Box const &b)
            { return not robot_room.containsPoint(QPointF(b.Tx, b.Ty), Qt::OddEvenFill);}), candidates.end());

    // remove those behind the camera
    candidates.erase(std::remove_if(candidates.begin(), candidates.end(),[this](Box const &b)
            {
                auto pos_wrt_camera = inner_eigen->transform(viriato_head_camera_name, Eigen::Vector3d(b.Tx,b.Ty,b.Tz), world_name);
                return not (pos_wrt_camera.has_value() and pos_wrt_camera.value().y() > 0);
            }), candidates.end());

    // check if new box type equals to one of the existing boxes and close enough to it
    if(auto r = std::ranges::find_if(candidates, [this, box](auto &b)mutable{ return box.type == b.type and box.distance_in_world_frame_to(b) < 300;}); r != candidates.end())
    {
        // if old enough, delete it from the list and return true
        auto now = std::chrono::steady_clock::now();
        if(r->creation_ticks > CONSTANTS.min_ticks_to_add_object_threshold
            and std::chrono::duration_cast<std::chrono::milliseconds>(now - r->creation_time).count() > CONSTANTS.min_time_to_add_object_threshold)
        {
            candidates.erase(r);
            qInfo() << __FUNCTION__ << "Candidate found. List size: " << candidates.size();
            return true;
        }
        else // increment
            r->creation_ticks++;
    }
    else // add
    {
        box.creation_ticks = 0;
        box.creation_time = std::chrono::steady_clock::now();
        candidates.push_back(box);
    }
    qInfo() << __FUNCTION__ << " Candidates:" << candidates.size();
    return false;
}
std::tuple<float, float, float> SpecificWorker::estimate_object_size_through_projection_optimization(const Box &b_synth, const Box &b_real)
{
    return std::make_tuple(0,0,0);
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
            box.type = object.value().type();
            if(auto t_world = inner_eigen->transform(world_name, object.value().name()); t_world.has_value())
            {
                if (auto t_camera = inner_eigen->transform(viriato_head_camera_name, object.value().name()); t_camera.has_value())
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
std::vector<SpecificWorker::Box> SpecificWorker::process_image_with_yolo(const cv::Mat &img, const std::vector<float> &depth_array)
{
    // get detections from RGB image
    image_t yolo_img = createImage(img);
    std::vector<bbox_t> detections = ynets[0]->detect(yolo_img, CONSTANTS.min_yolo_probability_threshold / 100.f, false);
    // process detected bounding boxes
    std::vector<Box> bboxes; bboxes.reserve(detections.size());
    int width = cam_api->get_width();
    int height = cam_api->get_height();

    for(const auto &d : detections)
    {
        if( auto &&[success, type] = contained_in_known_objects(yolo_names.at(d.obj_id)); success == true)
        {
            Box box;
            int cls = d.obj_id;
            box.name = yolo_names.at(cls);
            box.type = type;
            box.left = d.x - yolo_img.w / 2;
            box.right = d.x + d.w - yolo_img.w / 2;
            box.top = d.y - yolo_img.h / 2;
            box.bot = d.y + d.h - yolo_img.h / 2;
            if (box.left < -yolo_img.w / 2) box.left = -yolo_img.w / 2;
            if (box.right > yolo_img.w / 2 - 1) box.right = yolo_img.w / 2 - 1;
            if (box.top < -yolo_img.h / 2) box.top = -yolo_img.h / 2;
            if (box.bot > yolo_img.h / 2 - 1) box.bot = yolo_img.h / 2 - 1;
            box.left = (box.left * width) / YOLO_IMG_SIZE;
            box.right = (box.right * width) / YOLO_IMG_SIZE;
            box.top = (box.top * height) / YOLO_IMG_SIZE;
            box.bot = (box.bot * height) / YOLO_IMG_SIZE;
            box.prob = d.prob * 100;
            box.visible = true;
            box.match = false;
            int BDleft = box.left + width / 2;
            if (BDleft < 0) BDleft = 0;
            int BDright = box.right + width / 2;
            if (BDright >= width) BDright = width - 1;
            int BDtop = box.top + height / 2;
            if (BDtop < 0) BDtop = 0;
            int BDbot = box.bot + height / 2;
            if (BDbot >= height) BDbot = height - 1;

            auto tp = cam_api->get_roi_depth(depth_array, Eigen::AlignedBox<float, 2>(Eigen::Vector2f(BDleft, BDbot),
                                                                                      Eigen::Vector2f(BDright, BDtop)));
            if (tp.has_value())
            {
                auto[x, y, z] = tp.value();
                if (auto t_world = inner_eigen->transform(world_name, Mat::Vector3d(x, y, z), viriato_head_camera_name); t_world.has_value())
                {
                    //std::cout << "POS 3D "<<yolo_names.at(cls)<<" "<<t_world[0]<<" "<<t_world[1]<<" "<<t_world[2] << std::endl;
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
    }
    // qInfo() << __FILE__ << __FUNCTION__ << "LABELS " << bboxes.size();
    ynets[0]->free_image(yolo_img);
    return bboxes;
}
std::tuple<bool, std::string> SpecificWorker::contained_in_known_objects(const std::string &candidate)
{
    std::string type;
    for(const auto &[known_object, size] : this->known_object_types)
        if (candidate.find(known_object, 0) != string::npos)
            return std::make_tuple(true, known_object);
    return std::make_tuple(false, type);
}
DSR::Node SpecificWorker::create_node_with_type(const std::string &type, const std::string &name)
{
    DSR::Node object_node;

    if (type == "glass")
    object_node = DSR::Node::create<glass_node_type>(name);
    else if (type == "cup")
    object_node = DSR::Node::create<cup_node_type>(name);
    else if (type == "plant")
    object_node = DSR::Node::create<plant_node_type>(name);
    else if (type == "microwave")
    object_node = DSR::Node::create<microwave_node_type>(name);
    else if (type == "person")
    object_node = DSR::Node::create<person_node_type>(name);
    else if (type == "oven")
    object_node = DSR::Node::create<oven_node_type>(name);
    else if (type == "vase")
    object_node = DSR::Node::create<vase_node_type>(name);
    else if (type == "refrigerator")
    object_node = DSR::Node::create<refrigerator_node_type>(name);

    return object_node;
}
bool SpecificWorker::both_boxes_match(Box &real_box, Box &synth_box)
{
    //A rectangle with the real object is created
    QRect r(QPoint(real_box.left, real_box.top), QSize(real_box.width(), real_box.height()));
    //A rectangle with the sythetic object is created
    QRect rs(QPoint(synth_box.left, synth_box.top), QSize(synth_box.width(), synth_box.height()));
    //Compute intersection percentage between synthetic and real
    QRect i = rs.intersected(r);
    //The area is normalized between 0 and 1 dividing by the minimum between both areas of each object
    float area = (float)(i.width() * i.height()) / std::min(rs.width() * rs.height(), r.width() * r.height());
    //The displacement vector between the two images is calculated
    QPoint error = r.center() - rs.center();
    // If the area is 0 there is no intersection
    // If the error is less than three times width of the synthetic rectangle
    if(area > 0 or error.manhattanLength() < rs.width()*CONSTANTS.times_the_width_of_synth_box) //ADD DEPTH CHECK
    {
        //qInfo() << __FUNCTION__ << " area " << area << "error " << error.manhattanLength() << "rs widrh " << rs.width()*3;
        real_box.area = area; real_box.match_error = error.manhattanLength();
        synth_box.area = area; synth_box.match_error = error.manhattanLength();
        return true;
    }
    else
        return false;
}
// SHOULD BE LIMITED TO A MAXIMUM NUMBER OF OBJECTS
void SpecificWorker::compute_visible_objects()
{
    std::vector<Box> synth_box;
    std::vector<DSR::Node> object_nodes;
    // get all potentially visible objects
    for(const auto &[known_object, size] : known_object_types)
    {
        auto new_nodes = G->get_nodes_by_type(known_object); 
        object_nodes.insert(object_nodes.end(), new_nodes.begin(), new_nodes.end());
    }
    // computer room's delimiting polygon that contains the robot
    auto robot_node = G->get_node(robot_name);
    QPolygonF room_polygon;
    if( auto in_edges = G->get_node_edges_by_type(robot_node.value(), in_type_name); not in_edges.empty())
        if( auto room_node = G->get_node(in_edges.front().to()); room_node.has_value() )   // THIS CAN BE MOVED TO the SLOTs and create a list
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
    int width = cam_api->get_width();
    int height = cam_api->get_height();
    int final_counter = 0;
    for(auto &object : object_nodes)
    {
        const std::string &object_name = object.name();

        auto w_attr = G->get_attrib_by_name<obj_width_att>(object);
        float w = w_attr.value();
        auto h_attr = G->get_attrib_by_name<obj_height_att>(object);
        float h = h_attr.value();
        auto d_attr = G->get_attrib_by_name<obj_depth_att>(object);
        float d = d_attr.value();

        // check if object is in front of the robot. Its position relative to the camera must has positive Y value
        if(auto pos_wrt_camera = inner_eigen->transform(viriato_head_camera_name, object_name); pos_wrt_camera.has_value() and pos_wrt_camera.value().y() > 0)
        {
            // check if object is in the same room as the robot
            auto object_pos = inner_eigen->transform(world_name, object_name);
            if (not room_polygon.containsPoint(QPointF(object_pos.value().x(), object_pos.value().y()), Qt::OddEvenFill))
                continue;

            // project corners of object's bounding box in the camera image plane
            // get object's bounding box from object's node
            std::vector<Mat::Vector2d> bb_in_camera(8);
            bb_in_camera[0] = cam_api->project(inner_eigen->transform(viriato_head_camera_name, Mat::Vector3d(w / 2, d / 2, -h / 2), object_name).value(), 0, 0);
            bb_in_camera[1] = cam_api->project(inner_eigen->transform(viriato_head_camera_name, Eigen::Vector3d(-w / 2, d / 2, -h / 2), object_name).value(), 0, 0);
            bb_in_camera[2] = cam_api->project(inner_eigen->transform(viriato_head_camera_name, Eigen::Vector3d(w / 2, -d / 2, -h / 2), object_name).value(), 0, 0);
            bb_in_camera[3] = cam_api->project(inner_eigen->transform(viriato_head_camera_name, Eigen::Vector3d(-w / 2, -d / 2, -h / 2), object_name).value(), 0, 0);
            bb_in_camera[4] = cam_api->project(inner_eigen->transform(viriato_head_camera_name, Eigen::Vector3d(w / 2, d / 2, h / 2), object_name).value(), 0, 0);
            bb_in_camera[5] = cam_api->project(inner_eigen->transform(viriato_head_camera_name, Eigen::Vector3d(-w / 2, d / 2, h / 2), object_name).value(), 0, 0);
            bb_in_camera[6] = cam_api->project(inner_eigen->transform(viriato_head_camera_name, Eigen::Vector3d(w / 2, -d / 2, h / 2), object_name).value(), 0, 0);
            bb_in_camera[7] = cam_api->project(inner_eigen->transform(viriato_head_camera_name, Eigen::Vector3d(-w / 2, -d / 2, h / 2), object_name).value(), 0, 0);

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
            Box box;
            box.left = xExtremes.first->x();
            box.top = yExtremes.first->y();
            box.right = xExtremes.second->x();
            box.bot = yExtremes.second->y();
            box.prob = 100;
            box.name = object_name;
            box.match = false;
            box.type = object.type();

            auto bL = std::clamp(box.left, -width / 2, width / 2);
            auto bR = std::clamp(box.right, -width / 2, width / 2);
            auto bT = std::clamp(box.top, -height / 2, height / 2);
            auto bB = std::clamp(box.bot, -height / 2, height / 2);

            float areaV = (bR - bL) * (bB - bT); // clamped area
            float areaR = (box.right - box.left) * (box.bot - box.top);  // projected area

            // check if is inside the image
            //if( box.left>=-c and box.right<c and box.top>=-c and box.bot<c)
            if (areaV / areaR > CONSTANTS.percentage_of_visible_area_to_be_visible / 100.f) // ratio between clamped area and projected area
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
                    G->add_or_modify_attrib_local<projected_bounding_box_att>(new_edge, std::vector<float>{(float) box.left, (float) box.top, (float) box.right,
                                                                                                           (float) box.bot});
                    G->insert_or_assign_edge(new_edge);
                } else    // already there. update projection bounding box
                {
                    if (auto old_edge = G->get_edge(cam_api->get_id(), object.id(), "visible"); old_edge.has_value())
                        G->insert_or_assign_attrib<projected_bounding_box_att>(old_edge.value(),
                                                                               std::vector<float>{(float) box.left, (float) box.top, (float) box.right,
                                                                                                  (float) box.bot});
                    else
                        qWarning() << __FUNCTION__ << "No VISIBLE edge going from camera to " << QString::fromStdString(object.name());
                }
               final_counter++;
            }
            else  // remove edge
            {
                G->delete_edge(cam_api->get_id(), object.id(), "visible");
            }
        }
        else qWarning() << __FUNCTION__ << "Object  " << QString::fromStdString(object_name)  << " behind the camera";
    }
    qInfo() << __FUNCTION__ << "Total visible: " << final_counter << " Total: " << object_nodes.size();
}
void SpecificWorker::clear_all_attention_edges()
{
    if (const auto camera_node = G->get_node(viriato_head_camera_name); camera_node.has_value())
        for (auto edges = G->get_node_edges_by_type(camera_node.value(), attention_action_type_name); auto e : edges)
            G->delete_edge(e.from(), e.to(), attention_action_type_name);
}
////////////////////////////////////////////////////////////////////
/// Images
////////////////////////////////////////////////////////////////////
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
    int width = cam_api->get_width();
    int height = cam_api->get_height();

    for(const auto &box : real_boxes)
    {
        if(box.prob > CONSTANTS.min_yolo_probability_threshold)
        {
            auto left = box.left+width/2;
            auto right = box.right+width/2;
            auto top = box.top+height/2;
            auto bot = box.bot+height/2;
            auto p1 = cv::Point((left*YOLO_IMG_SIZE)/width, (top*YOLO_IMG_SIZE)/height);
            auto p2 = cv::Point((right*YOLO_IMG_SIZE)/width, (bot*YOLO_IMG_SIZE)/height);
            auto offset = int((((bot - top)*YOLO_IMG_SIZE)/height) / 2);
            auto pt = cv::Point(left + offset, top + offset);
            cv::rectangle(imgdst, p1, p2, cv::Scalar(0, 0, 255), 4);
            auto font = cv::FONT_HERSHEY_SIMPLEX;
            cv::putText(imgdst, box.name + " " + std::to_string(int(box.prob)) + "%", pt, font, 0.8, cv::Scalar(0, 255, 255), 2);
        }
    }
    for(const auto &box : synth_boxes)
    {
        auto left = box.left+width/2;
        auto right = box.right+width/2;
        auto top = box.top+height/2;
        auto bot = box.bot+height/2;
        auto p1 = cv::Point((left*YOLO_IMG_SIZE)/width, (top*YOLO_IMG_SIZE)/height);
        auto p2 = cv::Point((right*YOLO_IMG_SIZE)/width, (bot*YOLO_IMG_SIZE)/height);
        auto offset = int((((bot - top)*YOLO_IMG_SIZE)/height) / 2);
        auto pt = cv::Point(left + offset, top + offset);
        cv::rectangle(imgdst, p1, p2, cv::Scalar(0, 255, 0), 4);
        auto font = cv::FONT_HERSHEY_SIMPLEX;
        cv::putText(imgdst, box.name + " " + std::to_string(int(box.prob)) + "%", pt, font, 0.8, cv::Scalar(0, 255, 0), 2);
        //cout<<box.name<<" "<<"p1 "<<p1.x<<" "<<p1.y<<endl;
    }
    cv::drawMarker(imgdst, cv::Point(imgdst.cols/2, imgdst.rows/2),  cv::Scalar(0, 128, 128), cv::MARKER_CROSS, 60, 1);
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
                    rgb_buffer.put(std::vector<uint8_t>(g_image.value().get().begin(), g_image.value().get().end()),
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
            if( std::optional<std::string> plan = G->get_attrib_by_name<plan_att>(node.value()); plan.has_value())
                plan_buffer.put(std::move(plan.value()),[](const std::string& plan_text, Plan &plan){ plan.json_to_plan(plan_text);});
        }
    }
}

void SpecificWorker::add_or_assign_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type)
{
    if(from == G->get_node(world_name).value().id() and to == G->get_node(robot_name).value().id() and type == "RT")
            compute_visible_objects();
}
///////////////////////////////////////////////////////////////////
void SpecificWorker::clear_button_slot()
{
   clear_all_attention_edges();
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
void SpecificWorker::change_attention_object_slot(int index)
{
    std::string node_name = custom_widget.comboBox->itemText(index).toStdString();
    qInfo() << __FUNCTION__ << " " << index << " " << QString::fromStdString(node_name);
    if (auto object = G->get_node(node_name); object.has_value())
    {
        // remove current edge
        G->delete_edge(cam_api->get_id(), this->last_object_of_attention, attention_action_type_name);
        auto edge = DSR::Edge::create<attention_action_edge_type>(cam_api->get_id(), object.value().id());
        if (G->insert_or_assign_edge(edge))
                this->last_object_of_attention = object.value().id();
    }
    else
        std::cout << __FUNCTION__ << " WARNING: Error inserting new edge from camera: " << cam_api->get_id() << "->"
                  << node_name << std::endl;
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
