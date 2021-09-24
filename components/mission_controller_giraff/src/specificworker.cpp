/*
 *    Copyright (C) 2021 by YOUR NAME HERE
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
#include <cppitertools/enumerate.hpp>
#include <cppitertools/zip.hpp>
#include <cppitertools/filter.hpp>
#include <cppitertools/sliding_window.hpp>

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
		G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, ""); // Init nodes
		std::cout<< __FUNCTION__ << "Graph loaded" << std::endl;  

		//dsr update signals
		connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::add_or_assign_node_slot);
//		connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::add_or_assign_edge_slot);
//		connect(G.get(), &DSR::DSRGraph::update_attrs_signal, this, &SpecificWorker::add_or_assign_attrs_slot);
//		connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
//		connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);

		// Graph viewer
		using opts = DSR::DSRViewer::view;
		int current_opts = 0;
		opts main = opts::none;
		if(tree_view)
		    current_opts = current_opts | opts::tree;
		if(graph_view)
		{
		    current_opts = current_opts | opts::graph;
		    //main = opts::graph;
		}
		if(qscene_2d_view)
		    current_opts = current_opts | opts::scene;
		if(osg_3d_view)
		    current_opts = current_opts | opts::osg;

		graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
        setWindowTitle(QString::fromStdString(agent_name + "-") + QString::number(agent_id));

        // custom widget
        graph_viewer->add_custom_widget_to_dock("Giraff Plan Controller", &custom_widget);
        connect(custom_widget.pushButton_start_mission, SIGNAL(clicked()), this, SLOT(slot_start_mission()));
        connect(custom_widget.pushButton_stop_mission, SIGNAL(clicked()), this, SLOT(slot_stop_mission()));
        connect(custom_widget.pushButton_cancel_mission, SIGNAL(clicked()), this, SLOT(slot_cancel_mission()));
        connect(custom_widget.pushButton_save_coords, SIGNAL(clicked()), this, SLOT(slot_save_coords()));

        //List of missions
        custom_widget.list_plan->addItem("Misión punto");
        custom_widget.list_plan->addItem("Misión secuencia de puntos");
        custom_widget.list_plan->addItem("Misión choca-choca");

        // 2D widget
        widget_2d = qobject_cast<DSR::QScene2dViewer *>(graph_viewer->get_widget(opts::scene));
        widget_2d->set_draw_laser(true);
        connect(widget_2d, SIGNAL(mouse_right_click(int, int, std::uint64_t)), this, SLOT(new_target_from_mouse(int, int, std::uint64_t)));


        // get camera_api
        if(auto cam_node = G->get_node(giraff_camera_usb_name); cam_node.has_value())
            cam_api = G->get_camera_api(cam_node.value());
        else
        {
            std::cout << "Controller-DSR terminate: could not find a camera node named " << giraff_camera_usb_name << std::endl;
            std::terminate();
        }

        // Inner Api
        inner_eigen = G->get_inner_eigen_api();

        // Robot polygon
        if(auto robot_body = G->get_node(robot_body_name); robot_body.has_value())
        {
            auto width = G->get_attrib_by_name<width_att>(robot_body.value());
            auto height = G->get_attrib_by_name<depth_att>(robot_body.value());
            if (width.has_value() and height.has_value())
            {
                robot_polygon << QPointF(-width.value() / 2, -height.value() / 2)
                              << QPointF(-width.value() / 2, height.value() / 2)
                              << QPointF(width.value() / 2, height.value() / 2)
                              << QPointF(width.value() / 2, -height.value() / 2);
            } else
            {
                std::cout << __FUNCTION__ << " No robot body width or depth found. Terminating..." << std::endl;
                std::terminate();
            }
        }
        else
        {
            std::cout << __FUNCTION__ << " No robot body found. Terminating..." << std::endl;
            std::terminate();
        }

        // Eigen format
        OctaveFormat = Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
        CommaInitFmt = Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");

        this->Period = 100;
		timer.start(Period);
	}
}

void SpecificWorker::compute()
{
    static QPixmap pix;
    auto robot_node = get_robot_node();
    if (auto vframe_t = virtual_camera_buffer.try_get(); vframe_t.has_value())
    {
        auto vframe = cv::Mat(cam_api->get_height(), cam_api->get_width(), CV_8UC3, vframe_t.value().data());
        project_robot_on_image(robot_node, robot_polygon, vframe, cam_api->get_focal_x());
        if (auto laser_o = laser_buffer.try_get(); laser_o.has_value() and not vframe.empty())
        {
            const auto &[angles, dists, laser_poly_local, laser_cart_world] = laser_o.value();
            project_laser_on_image(robot_node, laser_poly_local, vframe, cam_api->get_focal_x());
        }
        project_path_on_image(path, robot_node, vframe, cam_api->get_focal_x());
        pix = QPixmap::fromImage(QImage(vframe.data, vframe.cols, vframe.rows, QImage::Format_RGB888));
        custom_widget.label_rgb->setPixmap(pix);
    }

    // check for existing missions
    if (auto plan_o = plan_buffer.try_get(); plan_o.has_value())
    {
        current_plan = plan_o.value();
        current_plan.print();
        custom_widget.current_plan->setPlainText(QString::fromStdString(current_plan.pprint()));
        // Create an interpretable version of the plan: m_plan
        // check that there is a computed path in G
        // get the path and divide it in N check points separated 1 sec
        // add to the plan the check points
    }
    // Compute next step of m_plan and check that it matches the current state

//    QPointF position( 2679,  -22367);
//    std::string name = custom_widget.list_plan->currentText().toStdString();
//    qInfo() << __FUNCTION__ << " New mission to " << QString::fromStdString(name);
//    auto node = G ->get_node(name);
//    qInfo() << __FUNCTION__ << " Prueba " << custom_widget.list_plan->itemText(2);
//    create_mission(position,node.value().id());
}


void SpecificWorker::create_mission(const QPointF &pos, std::uint64_t target_node_id)
{

}


/////////////////////////////////////////////////////////////////////////////////////////////

DSR::Node SpecificWorker::get_robot_node()
{
    if (auto robot_o = G->get_node(robot_name); robot_o.has_value())
        return robot_o.value();
    else
    {
        std::cout << __FUNCTION__ << " Terminating due to missing robot_node: " << robot_name << std::endl;
        std::terminate();
    }
}

void SpecificWorker::project_robot_on_image(const DSR::Node &robot_node, const QPolygonF &robot_polygon, cv::Mat virtual_frame, float focal)
{
    if (auto local_velocity = G->get_attrib_by_name<robot_local_linear_velocity_att>(robot_node); local_velocity.has_value())
    {
        float robot_adv_speed = local_velocity.value().get()[1];   // Y component of linear speed in robot's coordinate frame
        int delta_time = 1;  // 1 sec
        if (fabs(robot_adv_speed) < 50) return;    // only do if advance velocity is greater than 50
        // displace robot polygon by offset
        QPolygonF robot_polygon_projected(robot_polygon);
        robot_polygon_projected.translate(0, robot_adv_speed * delta_time);
        // transform projected polygon to virtual camera coordinate frame and project into virtual camera
        std::vector<cv::Point> cv_poly;
        for (const auto &p : robot_polygon_projected)
        {
            if(auto projected_point = inner_eigen->transform(giraff_camera_usb_name, Eigen::Vector3d(p.x(), p.y(), 0.f), robot_name); projected_point.has_value())
            {
                auto point = cam_api->project(projected_point.value());
                if( point.y() > virtual_frame.rows / 2)
                    cv_poly.emplace_back(cv::Point(point.x(), point.y()));
            }
        }
        // paint on image
        const cv::Point *pts = (const cv::Point *) cv::Mat(cv_poly).data;
        int npts = cv::Mat(cv_poly).rows;
        cv::polylines(virtual_frame, &pts, &npts, 1, true, cv::Scalar(0, 255, 0), 8);
    }
    else{ std::cout << __FUNCTION__ << " No robot_local_linear_velocity attribute in robot: " << robot_name << std::endl; }
}
void SpecificWorker::project_path_on_image(const std::vector<Eigen::Vector3d> &path, const DSR::Node robot_node, cv::Mat virtual_frame, float focal)
{
    int cont = 0;
    std::vector<cv::Point> cv_points;
    Eigen::Hyperplane<double, 3> robot_plane(Eigen::Vector3d(0.0, 1.0, 0.0), 1000);

    auto cmp = [](auto &a, auto &b) { return std::get<double>(a) < std::get<double>(b); };
    std::set<std::tuple<Eigen::Vector3d, double>, decltype(cmp)> ahead_of_robot;
    for (const auto &p : path)
    {
        auto apr = inner_eigen->transform(robot_name, p, world_name).value();
        auto d = robot_plane.signedDistance(apr);
        if (d > 0)
            ahead_of_robot.insert(std::make_tuple(p, d));
    }
    std::transform(ahead_of_robot.cbegin(), ahead_of_robot.cend(), std::back_inserter(cv_points), [this, virtual_frame](auto &p) {
        if (auto projected_point = inner_eigen->transform(giraff_camera_usb_name, std::get<0>(p), world_name); projected_point.has_value())
        {
            auto point = cam_api->project(projected_point.value());
            if (point.y() > virtual_frame.rows / 2)
                return cv::Point(point.x(), point.y());
        } else
            return cv::Point();
    });
    for (const auto &p : cv_points)
        cv::circle(virtual_frame, p, 6, cv::Scalar(51, 165, 50), cv::FILLED);
    for (auto &&p: iter::sliding_window(cv_points, 2))
        cv::line(virtual_frame, p[0], p[1], cv::Scalar(190, 234, 182), 2);
}
void SpecificWorker::project_laser_on_image(const DSR::Node &robot_node, const QPolygonF &laser_poly_local, cv::Mat virtual_frame, float focal)
{
        std::vector<cv::Point> cv_poly;
        // transform laser polygon to virtual camera coordinate frame
        for(const auto &p : laser_poly_local)
        {
            if(auto projected_point = inner_eigen->transform(giraff_camera_usb_name, Eigen::Vector3d(p.x(), p.y(), 0.f), robot_name); projected_point.has_value())
            {
                auto point = cam_api->project(projected_point.value());
                if( point.x() < virtual_frame.cols and point.y() < virtual_frame.rows and point.x() >= 0 and point.y()>=0 )
                    cv_poly.emplace_back(cv::Point(point.x(), point.y()));
            }
        }
        // add two points to close de polygon
        cv_poly.emplace_back(cv::Point(virtual_frame.cols/2,virtual_frame.rows));
        cv_poly.insert(cv_poly.begin(), cv::Point(virtual_frame.cols/2, virtual_frame.rows));
        // paint on image
        cv::Mat overlay;  // declaring overlay matrix, we'll copy source image to this matrix
        double alpha = 0.3;  // defining opacity value, 0 means fully transparent, 1 means fully opaque
        virtual_frame.copyTo(overlay);
        const cv::Point *pts = (const cv::Point*) cv::Mat(cv_poly).data;
        int npts = cv::Mat(cv_poly).rows;
        cv::polylines(overlay, &pts, &npts, 1, true, cv::Scalar(255,192,203), 3);
        cv::fillPoly(overlay, &pts, &npts, 1, cv::Scalar(255,182,193));
        cv::addWeighted(overlay, alpha, virtual_frame, 1 - alpha, 0, virtual_frame);  // blending the overlay (with alpha opacity) with the source image (with 1-alpha opacity)
}
void SpecificWorker::draw_path(std::vector<Eigen::Vector3d> &path, QGraphicsScene* viewer_2d)
{
    static std::vector<QGraphicsLineItem *> scene_road_points;

    //clear previous points
    for (QGraphicsLineItem* item : scene_road_points)
        viewer_2d->removeItem((QGraphicsItem*)item);
    scene_road_points.clear();

    /// Draw all points
    QGraphicsLineItem *line1, *line2;
    std::string color;
    for (unsigned int i = 1; i < path.size(); i++)
        for(auto &&p_pair : iter::sliding_window(path, 2))
        {
            if(p_pair.size() < 2)
                continue;
            Mat::Vector2d a_point(p_pair[0].x(), p_pair[0].y());
            Mat::Vector2d b_point(p_pair[1].x(), p_pair[1].y());
            Mat::Vector2d dir = a_point - b_point;
            Mat::Vector2d dir_perp = dir.unitOrthogonal();
            Eigen::ParametrizedLine segment = Eigen::ParametrizedLine<double, 2>::Through(a_point, b_point);
            Eigen::ParametrizedLine<double, 2> segment_perp((a_point+b_point)/2, dir_perp);
            auto left = segment_perp.pointAt(50);
            auto right = segment_perp.pointAt(-50);
            QLineF qsegment(QPointF(a_point.x(), a_point.y()), QPointF(b_point.x(), b_point.y()));
            QLineF qsegment_perp(QPointF(left.x(), left.y()), QPointF(right.x(), right.y()));

            if(i == 1 or i == path.size()-1)
                color = "#00FF00"; //Green

            line1 = viewer_2d->addLine(qsegment, QPen(QBrush(QColor(QString::fromStdString(color))), 20));
            line2 = viewer_2d->addLine(qsegment_perp, QPen(QBrush(QColor(QString::fromStdString("#F0FF00"))), 20));
            line1->setZValue(2000);
            line2->setZValue(2000);
            scene_road_points.push_back(line1);
            scene_road_points.push_back(line2);
        }
}
void SpecificWorker::send_command_to_robot(const std::tuple<float, float, float> &speeds)   //adv, side, rot
{
    auto &[adv_, side_, rot_] = speeds;
    auto robot_node = G->get_node(robot_name);
    G->add_or_modify_attrib_local<robot_ref_adv_speed_att>(robot_node.value(),  (float)adv_);
    G->add_or_modify_attrib_local<robot_ref_rot_speed_att>(robot_node.value(), (float)rot_);
    G->add_or_modify_attrib_local<robot_ref_side_speed_att>(robot_node.value(),  (float)side_);
    G->update_node(robot_node.value());
}
/////////////////////////////////////////////////////////////////////////////////////////////
/// Asynchronous changes on G nodes from G signals
////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::add_or_assign_node_slot(const std::uint64_t id, const std::string &type)
{
    if(type == rgbd_type_name and id == cam_api->get_id())
    {
        if (auto cam_node = G->get_node(id); cam_node.has_value())
            if (const auto g_image = G->get_attrib_by_name<cam_rgb_att>(cam_node.value()); g_image.has_value())
                virtual_camera_buffer.put(std::vector<uint8_t>(g_image.value().get().begin(), g_image.value().get().end()));
    }
    else if (type == laser_type_name)    // Laser node updated
    {
        //qInfo() << __FUNCTION__ << " laser node change";
        if( auto node = G->get_node(id); node.has_value())
        {
            auto angles = G->get_attrib_by_name<laser_angles_att>(node.value());
            auto dists = G->get_attrib_by_name<laser_dists_att>(node.value());
            if(dists.has_value() and angles.has_value())
            {
                if(dists.value().get().empty() or angles.value().get().empty()) return;
                laser_buffer.put(std::move(std::make_tuple(angles.value().get(), dists.value().get())),
                                 [this](const LaserData &in, std::tuple<std::vector<float>, std::vector<float>, QPolygonF,std::vector<QPointF>> &out) {
                                     QPolygonF laser_poly_local;
                                     //laser_poly_local << QPointF(0.f, 200.f);
                                     std::vector<QPointF> laser_cart_world;
                                     const auto &[angles, dists] = in;
                                     for (const auto &[angle, dist] : iter::zip(angles, dists))
                                     {
                                         //convert laser polar coordinates to cartesian in the robot's coordinate frame
                                         float x = dist * sin(angle);
                                         float y = dist * cos(angle);
                                         Mat::Vector3d laserWorld = inner_eigen->transform(world_name,Mat::Vector3d(x, y, 0), laser_name).value();
                                         laser_poly_local << QPointF(x, y);
                                         laser_cart_world.emplace_back(QPointF(laserWorld.x(), laserWorld.y()));
                                     }
                                     //laser_poly_local << QPointF(0.f, 200.f);
                                     out = std::make_tuple(angles, dists, laser_poly_local, laser_cart_world);
                                 });
            }
        }
    }
    else if (type == path_to_target_type_name)
    {
        if( auto path_to_target_node = G->get_node(id); path_to_target_node.has_value())
        {
            auto x_values_o = G->get_attrib_by_name<path_x_values_att>(path_to_target_node.value());
            auto y_values_o = G->get_attrib_by_name<path_y_values_att >(path_to_target_node.value());
            if(x_values_o.has_value() and y_values_o.has_value())
            {
                path.clear();
                auto &x_values = x_values_o.value().get();
                auto &y_values = y_values_o.value().get();
                path.reserve(x_values.size());
                for(auto &&[p, q] : iter::zip(x_values,y_values))
                    path.emplace_back(Eigen::Vector3d(p, q, 0.f));
                draw_path(path, &widget_2d->scene);
            }
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

// connect to signal and show virtual image
///////////////////////////////////////////////////////
//// Check new target from mouse
///////////////////////////////////////////////////////
void SpecificWorker::new_target_from_mouse(int pos_x, int pos_y, std::uint64_t id)
{
    qInfo() << __FUNCTION__ << " Creating GOTO mission to " << pos_x << pos_y;
    qInfo() << __FUNCTION__ << "[" << pos_x << " " << pos_y << "] Id:" << id;
    create_mission(QPointF(pos_x, pos_y), id);
    Plan plan;
    std::string plan_string;
    // we get the id of the object clicked from the 2D representation
    if (auto target_node = G->get_node(id); target_node.has_value())
    {
        //const std::string location = "[" + std::to_string(pos_x) + "," + std::to_string(pos_y) + "," + std::to_string(0) + "]";
        std::stringstream location;
        location <<"[" << std::to_string(pos_x) << "," << std::to_string(pos_y) << "," + std::to_string(0) << "]";
        plan_string = R"({"plan":[{"action":"goto","params":{"location":)" + location.str() + R"(,"object":")" + target_node.value().name() + "\"}}]}";
        plan = Plan(plan_string);
        plan.print();
        insert_intention_node(plan);
        plan_buffer.put(std::move(plan));
    }
    else
        qWarning() << __FUNCTION__ << " No target node  " << QString::number(id) << " found";
}
void SpecificWorker::insert_intention_node(const Plan &plan)
{
    // Check if there is not 'intention' node yet in G
    if(auto mind = G->get_node(robot_mind_name); mind.has_value())
    {
        if (auto intention = G->get_node(current_intention_name); not intention.has_value())
        {
            DSR::Node intention_node = DSR::Node::create<intention_node_type>(current_intention_name);
            G->add_or_modify_attrib_local<parent_att>(intention_node, mind.value().id());
            G->add_or_modify_attrib_local<level_att>(intention_node, G->get_node_level(mind.value()).value() + 1);
            G->add_or_modify_attrib_local<pos_x_att>(intention_node, (float) -290);
            G->add_or_modify_attrib_local<pos_y_att>(intention_node, (float) -344);
            G->add_or_modify_attrib_local<current_intention_att>(intention_node, plan.to_string());
            if (std::optional<int> intention_node_id = G->insert_node(intention_node); intention_node_id.has_value())
            {
                DSR::Edge edge = DSR::Edge::create<has_edge_type>(mind.value().id(), intention_node.id());
                if (G->insert_or_assign_edge(edge))
                {
                    std::cout << __FUNCTION__ << " Edge successfully inserted: " << mind.value().id() << "->" << intention_node.id()
                              << " type: has" << std::endl;
                    G->add_or_modify_attrib_local<current_intention_att>(intention_node, plan.to_string());
                    G->update_node(intention_node);
                }
                else
                {
                    std::cout << __FUNCTION__ << ": Fatal error inserting new edge: " << mind.value().id() << "->" << intention_node_id.value()
                              << " type: has" << std::endl;
                    std::terminate();
                }
            } else
            {
                std::cout << __FUNCTION__ << ": Fatal error inserting_new 'intention' node" << std::endl;
                std::terminate();
            }
        }
        else // there is one intention node
        {
            std::cout << __FUNCTION__ << ": Updating existing intention node with Id: " << intention.value().id() << std::endl;
            G->add_or_modify_attrib_local<current_intention_att>(intention.value(), plan.to_string());
            G->update_node(intention.value());
        }
    }
    else
    {
        std::cout  << __FUNCTION__ << " No node " << robot_mind_name << " found in G" << std::endl;
        std::terminate();
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// UI
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void SpecificWorker::slot_start_mission()
{
    if(not current_plan.is_empty)
    {
        insert_intention_node(current_plan);
        auto temp_plan = current_plan;
        plan_buffer.put(std::move(temp_plan));
    }
    else
        qWarning() << __FUNCTION__ << "No valid plan available";
}

void SpecificWorker::slot_stop_mission()
{
    qInfo() << __FUNCTION__  ;
    send_command_to_robot(std::make_tuple(0.f,0.f,0.f));   //adv, side, rot

    // Check if there is intention node in G
    if(auto intention = G->get_node(current_intention_name); intention.has_value())
    {
        if (auto path = G->get_node(current_path_name); path.has_value())
        {
            if (G->delete_node(path.value().id()))
                qInfo() << __FUNCTION__ << "Node " << QString::fromStdString(current_path_name) << " deleted ";
            else
                qInfo() << __FUNCTION__ << "Error deleting node " << QString::fromStdString(current_path_name);
        }
        if( auto target_room_edges = G->get_node_edges_by_type(intention.value(), "goto_action"); not target_room_edges.empty())
        {
            for (const auto &tr_edge : target_room_edges)
                G->delete_edge(tr_edge.from(), tr_edge.to(), "goto_action");
        }
        if (G->delete_node(intention.value().id()))
            qInfo() << __FUNCTION__ << "Node " << QString::fromStdString(current_intention_name) << " deleted ";
        else
            qInfo() << __FUNCTION__ << "Error deleting node " << QString::fromStdString(current_intention_name);
        send_command_to_robot(std::make_tuple(0.f,0.f,0.f));   //adv, side, rot
    }
    else
        qWarning() << __FUNCTION__ << "No intention node found";
}

void SpecificWorker::slot_cancel_mission()
{
    slot_stop_mission();
    current_plan.is_empty = true;
}

void SpecificWorker::slot_save_coords()
{
    //cuadros detexto --> nodo intención

    int coordX = custom_widget.coordX->value();
    int coordY = custom_widget.coordY->value();

    cout<< "Coordenada X: " << coordX << " | Coordenada Y: " << coordY << endl;

    Plan plan;
    std::string plan_string;
    std::stringstream location;

    location <<"[" << std::to_string(coordX) << "," << std::to_string(coordY) << "," + std::to_string(0) << "]";
    plan_string = R"({"plan":[{"action":"goto","params":{"location":)" + location.str() + R"(,"object":")" + "\"}}]}";
    current_plan = Plan(plan_string);
    current_plan.print();
    //insert_intention_node(plan);
    //plan_buffer.put(std::move(plan));

    //Actualizar el currentplan
}


//    using namespace std::placeholders;
//    if (auto target_node = G->get_node(id); target_node.has_value())
//    {
//        const std::string location =
//                "[" + std::to_string(pos_x) + "," + std::to_string(pos_y) + "," + std::to_string(0) + "]";
//        const std::string plan =
//                "{\"plan\":[{\"action\":\"goto\",\"params\":{\"location\":" + location + ",\"object\":\"" +
//                target_node.value().name() + "\"}}]}";
//        std::cout << plan << std::endl;
//        plan_buffer.put(plan, std::bind(&SpecificWorker::json_to_plan, this, _1, _2));
//    } else
//        qWarning() << __FILE__ << __FUNCTION__ << " No target node  " << QString::number(id) << " found";