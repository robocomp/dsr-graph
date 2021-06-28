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
#include <cppitertools/zip.hpp>
#include <cppitertools/range.hpp>



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
	//G->write_to_json_file("./"+agent_name+".json");
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

//    consts.max_adv_speed = std::stof(params["MaxAdvanceSpeed"].value);
//    consts.max_rot_speed = std::stof(params["MaxRotationSpeed"].value);
//    consts.max_side_speed = std::stof(params["MaxSideSpeed"].value);
//    consts.robot_radius = std::stof(params["RobotRadius"].value);
//    consts.robot_width = std::stof(params["RobotXWidth"].value);
//    consts.robot_length = std::stof(params["RobotZLong"].value);

    robot_rect = QRectF(QPointF(-consts.robot_width / 2.0, consts.robot_length / 2.0),
                        QPointF(consts.robot_width / 2.0, -consts.robot_length / 2.0));
    robot_rect_extended = robot_rect;
    robot_rect_extended.adjust(-consts.offset, consts.offset, consts.offset, -consts.offset);
    line_bottom = QLineF(robot_rect_extended.bottomLeft(), robot_rect_extended.bottomRight());
    line_left = QLineF(robot_rect_extended.topLeft(), robot_rect_extended.bottomLeft());
    line_top = QLineF(robot_rect_extended.topLeft(), robot_rect_extended.topRight());
    line_right = QLineF(robot_rect_extended.topRight(), robot_rect_extended.bottomRight());

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
		opts main = opts::scene;
		if(tree_view)
		    current_opts = current_opts | opts::tree;
		if(graph_view)
		    current_opts = current_opts | opts::graph;
		if(qscene_2d_view)
		    current_opts = current_opts | opts::scene;
		if(osg_3d_view)
		    current_opts = current_opts | opts::osg;
		graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
		setWindowTitle(QString::fromStdString(agent_name + "-") + QString::number(agent_id));

        //Inner Api
        inner_eigen = G->get_inner_eigen_api();

        // Ignore attributes from G
        G->set_ignored_attributes<cam_rgb_att, cam_depth_att >();

        widget_2d = qobject_cast<DSR::QScene2dViewer *>(graph_viewer->get_widget(opts::scene));
        if (widget_2d != nullptr)
            widget_2d->set_draw_laser(true);

        robot_extended_points = get_points_along_extended_robot_polygon(200, 40);
        
		this->Period = period;
		timer.start(Period);
	}
}
void SpecificWorker::compute()
{
    if( auto laser_pol = laser_buffer.try_get(); laser_pol.has_value())
    {
        // get extended robot polygon in worlds's coordinate frame

        std::vector<QPointF> contacts;
        QVector2D total(0, 0);
        for (const auto &p : robot_extended_points)
            if(not laser_pol.value().containsPoint(p, Qt::OddEvenFill))
            {
                total = total + QVector2D(p);
                contacts.push_back(p);
            }

        // project the resultant on the speed axes
        total = -total;
        float advVel = std::clamp(total.x(), -consts.max_adv_speed, consts.max_adv_speed);
        float sideVel = std::clamp(-total.y(), -consts.max_side_speed, consts.max_side_speed);
        send_command_to_robot(std::make_tuple(advVel, sideVel, 0.f));
        draw_bumper(contacts, total);

        if(total != QVector2D(0, 0))
            qInfo() << __FUNCTION__ << advVel << sideVel;

    }
    //fps.print("FPS: ", [this](auto x){ graph_viewer->set_external_hz(x);});
}
void SpecificWorker::draw_bumper(const std::vector<QPointF> &contacts, const QVector2D &total)
{
    static QGraphicsRectItem *robot_draw = nullptr;
    static std::vector<QGraphicsEllipseItem *> contacts_draw;
    static QGraphicsLineItem *total_line = nullptr;

    if(robot_draw != nullptr)
    {
        widget_2d->scene.removeItem(robot_draw);
        delete robot_draw;
    }
    if(total_line != nullptr)
    {
        widget_2d->scene.removeItem(total_line);
        delete total_line;
    }
    if(not contacts_draw.empty())
    {
        for(auto &p: contacts_draw)
            widget_2d->scene.removeItem(p);
        contacts_draw.clear();
    }

    auto robot = widget_2d->get_robot_polygon();
    auto robot_rect_in_world = robot->mapRectFromItem(robot,  robot_rect_extended);
    robot_draw = widget_2d->scene.addRect(robot_rect_in_world, QPen(QColor("blue"), 10));
    robot_draw->setRotation(widget_2d->get_robot_polygon()->rotation());
    robot_draw->setPos(widget_2d->get_robot_polygon()->pos());

    auto pen = QPen(QColor("red"), 10);
    auto brush = QBrush(QColor("red"));
    for(const auto &p: contacts)
    {
        auto contact_in_world = robot->mapToScene(p);
        contacts_draw.emplace_back(widget_2d->scene.addEllipse(contact_in_world.x() - 50, contact_in_world.y() - 50, 100, 100, pen, brush));
    }

    auto total_line_in_world = robot->mapToScene(total.toPointF());
    total_line = widget_2d->scene.addLine(QLineF(robot->pos(), total_line_in_world * 6), pen);
}
std::vector<QPointF> SpecificWorker::get_points_along_extended_robot_polygon(int offset, int chunck)
{
    std::vector<QPointF> points;
    
    for(auto i : iter::range(0.f, 1.f, (float)(1.f/(line_top.length()/chunck))))
    {
        points.push_back(line_top.pointAt(i));
        points.push_back(line_bottom.pointAt(i));
    }
    for(auto i : iter::range(0.f, 1.f, (float)(1.f/(line_left.length()/chunck))))
    {
        points.push_back(line_left.pointAt(i));
        points.push_back(line_right.pointAt(i));
    }
    return points;
}
std::tuple<float, float, float> SpecificWorker::send_command_to_robot(const std::tuple<float, float, float> &speeds) //adv, side, rot
{
    auto &[adv_, side_, rot_] = speeds;
    if(auto robot_node = G->get_node(robot_name); robot_node.has_value())
    {
        G->add_or_modify_attrib_local<robot_ref_adv_speed_att>(robot_node.value(), (float) adv_);
        G->add_or_modify_attrib_local<robot_ref_rot_speed_att>(robot_node.value(), (float) rot_);
        G->add_or_modify_attrib_local<robot_ref_side_speed_att>(robot_node.value(), (float) side_);
        G->update_node(robot_node.value());
    }
    else qWarning() << __FUNCTION__ << "No robot node found";
    return std::make_tuple(adv_, side_, rot_);
}
///////////////////////////////////////////////////////////////////////////////////7
void SpecificWorker::add_or_assign_node_slot(std::uint64_t id, const string &type)
{
    if(type==laser_name)
    {
        if (auto node = G->get_node(id); node.has_value())
        {
            auto angles = G->get_attrib_by_name<laser_angles_att>(node.value());
            auto dists = G->get_attrib_by_name<laser_dists_att>(node.value());
            if (dists.has_value() and angles.has_value())
            {
                const auto &d = dists.value().get();
                const auto &a = angles.value().get();
                if (d.empty() or a.empty()) return;
                laser_buffer.put(std::make_tuple(a, d),
                                 [this](const LaserData &in, QPolygonF &laser_poly)
                                 {
                                     std::vector<QPointF> laser_cart;
                                     const auto &[angles, dists] = in;
                                     laser_cart.reserve(angles.size());
                                     auto inner = G->get_inner_eigen_api();
                                     for (const auto &[angle, dist] : iter::zip(angles, dists))
                                     {
                                         //convert laser polar coordinates to cartesian
                                         if (dist == 0) continue;
                                         float x = dist * sin(angle);
                                         float y = dist * cos(angle);
                                         laser_poly << QPointF(x, y);
                                     }
                                 });
            }
        }
    }
}
/////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}