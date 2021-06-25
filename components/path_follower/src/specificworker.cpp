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
#include <cppitertools/zip.hpp>
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/range.hpp>
#include <algorithm>
#include <QPointF>
#include <ranges>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
    qRegisterMetaType<std::uint64_t>("std::uint64_t");
	this->startup_check_flag = startup_check;
    QLoggingCategory::setFilterRules("*.debug=false\n");
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	G.reset();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	conf_params  = std::make_shared<RoboCompCommonBehavior::ParameterList>(params);
    try
    {
        agent_name = params["agent_name"].value;
        agent_id = stoi(params["agent_id"].value);
        tree_view = params["tree_view"].value == "true";
        graph_view = params["graph_view"].value == "true";
        qscene_2d_view = params["2d_view"].value == "true";
        osg_3d_view = params["3d_view"].value == "true";
    }
    catch(...){std::cout << __FUNCTION__ << "Problem reading params" << std::endl;};
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << __FUNCTION__ << std::endl;
	this->Period = period;
	if(this->startup_check_flag)
		this->startup_check();
	else
	{
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
            //main = opts::graph;
		if(qscene_2d_view)
			current_opts = current_opts | opts::scene;
		if(osg_3d_view)
			current_opts = current_opts | opts::osg;
		dsr_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
        setWindowTitle(QString::fromStdString(agent_name + "-" + std::to_string(agent_id)));

        //dsr update signals
        connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::add_or_assign_node_slot, Qt::QueuedConnection);
        //connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::add_or_assign_edge_slot);
        //connect(G.get(), &DSR::DSRGraph::update_attrs_signal, this, &SpecificWorker::add_or_assign_attrs_slot);
        //connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
        connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot, Qt::QueuedConnection);

        //Inner Api
        inner_eigen = G->get_inner_eigen_api();

        // Ignore attributes from G
        G->set_ignored_attributes<cam_rgb_att, cam_depth_att >();

        // Custom widget
        dsr_viewer->add_custom_widget_to_dock("Path follower", &custom_widget);
        connect(custom_widget.startButton, &QPushButton::clicked, [this](){
            if(robot_is_active)
            {
                robot_is_active = false;
                send_command_to_robot((std::make_tuple(0, 0, 0)));
                custom_widget.startButton->setText("Start");
            }
            else
            {
                robot_is_active = true;
                custom_widget.startButton->setText("Stop");
            }
        });

        widget_2d = qobject_cast<DSR::QScene2dViewer *>(dsr_viewer->get_widget(opts::scene));
        if (widget_2d != nullptr)
            widget_2d->set_draw_laser(false);

        // path planner
        path_follower_initialize();

        // check for existing path node
        if(auto paths = G->get_nodes_by_type(path_to_target_type_name); not paths.empty())
            this->add_or_assign_node_slot(paths.front().id(), path_to_target_type_name);

        this->Period = 200;
        std::cout<< __FUNCTION__ << "Initialization finished" << std::endl;
        timer.start(Period);
	}
}

void SpecificWorker::compute()
{

    static std::vector<Eigen::Vector2f> path;

    // Check for existing path_to_target_nodes
    if (auto path_o = path_buffer.try_get(); path_o.has_value()) // NEW PATH!
    {
        qInfo() << __FUNCTION__ << "New path";
        path.clear();
        path = path_o.value();
        if(widget_2d != nullptr)
            draw_path(path, &widget_2d->scene);
    }
    // if no path node, stop controlling
    if(auto node_path = G->get_node(current_path_name); node_path.has_value())
    {
        if (const auto laser_data = laser_buffer.try_get(); laser_data.has_value())
        {
            const auto &[angles, dists, laser_poly, laser_cart] = laser_data.value();
            auto nose_3d = inner_eigen->transform(world_name, Mat::Vector3d(0, 500, 0), robot_name).value();
            auto robot_pose_3d = inner_eigen->transform(world_name, robot_name).value();
            auto robot_nose = Eigen::Vector2f(nose_3d.x(), nose_3d.y());
            auto robot_pose = Eigen::Vector2f(robot_pose_3d.x(), robot_pose_3d.y());
            auto speeds = update(path, laser_poly, robot_pose, robot_nose, current_target);
            auto[adv, side, rot] =  send_command_to_robot(speeds);
            remove_trailing_path(path, robot_pose);

            std::cout << "---------------------------" << std::endl;
            std::cout << "Robot position: " << std::endl;
            std::cout << "\t " << robot_pose.x() << ", " << robot_pose.y() << std::endl;
            std::cout << "Target position: " << std::endl;
            std::cout << "\t " << current_target.x() << ", " << current_target.y() << std::endl;
            std::cout << "Dist to target: " << std::endl;
            std::cout << "\t " << (robot_pose - current_target).norm() << std::endl;
            std::cout << "Ref speeds:  " << std::endl;
            std::cout << "\t Advance-> " << adv << std::endl;
            std::cout << "\t Side -> " << side << std::endl;
            std::cout << "\t Rotate -> " << rot << std::endl;
            std::cout << "\tRobot_is_active -> " << std::boolalpha << robot_is_active << std::endl;
        }
        else
        {} // check elapsed time since last reading. Stop the robot if too long
    }
    else
        qDebug() << __FUNCTION__ << "No path_node found in G";
    fps.print("FPS: ", [this](auto x){ dsr_viewer->set_external_hz(x);});
}
void SpecificWorker::path_follower_initialize()
{
    qDebug()<< "Controller - " << __FUNCTION__;
    try
    {
        consts.max_adv_speed = QString::fromStdString(conf_params->at("MaxAdvanceSpeed").value).toFloat();
        consts.max_rot_speed = QString::fromStdString(conf_params->at("MaxRotationSpeed").value).toFloat();
        consts.max_side_speed = QString::fromStdString(conf_params->at("MaxSideSpeed").value).toFloat();
        consts.max_lag = std::stof(conf_params->at("MinControllerPeriod").value);
        consts.robot_radius =  QString::fromStdString(conf_params->at("RobotRadius").value).toFloat();
        qDebug() << __FUNCTION__ << "CONTROLLER: Params from config:"  <<  consts.max_adv_speed <<  consts.max_rot_speed <<  consts.max_side_speed <<  consts.max_lag << consts.robot_radius;
    }
    catch (const std::out_of_range& oor)
    {
        std::cout << "CONTROLLER. Out of Range error reading parameters: " << oor.what() << '\n';
        std::terminate();
    }
        robotXWidth = std::stof(conf_params->at("RobotXWidth").value);
        robotZLong = std::stof(conf_params->at("RobotZLong").value);
        robotBottomLeft     = Mat::Vector3d ( -robotXWidth / 2, -robotZLong / 2, 0);
        robotBottomRight    = Mat::Vector3d ( robotXWidth / 2, -robotZLong / 2, 0);
        robotTopRight       = Mat::Vector3d ( robotXWidth / 2, robotZLong / 2, 0);
        robotTopLeft        = Mat::Vector3d ( -robotXWidth / 2, robotZLong / 2, 0);
}
void SpecificWorker::remove_trailing_path(const std::vector<Eigen::Vector2f> &path, const Eigen::Vector2f &robot_pose)
{
    // closest point to robot nose in path
    if(path.size() == 1) return;

    auto closest_point_to_robot = std::ranges::min_element(path, [robot_pose](auto &a, auto &b){ return (robot_pose - a).norm() < (robot_pose - b).norm();});
    std::vector<float> x_values;  x_values.reserve(path.size());
    std::transform(closest_point_to_robot, path.cend(), std::back_inserter(x_values),
                   [](const auto &value) { return value.x(); });
    std::vector<float> y_values;  y_values.reserve(path.size());
    std::transform(closest_point_to_robot, path.cend(), std::back_inserter(y_values),
                   [](const auto &value) { return value.y(); });
    if (auto node_path = G->get_node(current_path_name); node_path.has_value())
    //if (auto node_paths = G->get_nodes_by_type(path_to_target_type_name); not node_paths.empty())
    {
        //auto path_to_target_node = node_paths.front();
        G->add_or_modify_attrib_local<path_x_values_att>(node_path.value(), x_values);
        G->add_or_modify_attrib_local<path_y_values_att>(node_path.value(), y_values);
        G->update_node(node_path.value());
    }
    else
        std::cout << __FUNCTION__ << "No path target " << std::endl;
}
std::tuple<float, float, float> SpecificWorker::update(const std::vector<Eigen::Vector2f> &path, const QPolygonF &laser_poly, const Eigen::Vector2f &robot_pose,
                                                       const Eigen::Vector2f &robot_nose, const Eigen::Vector2f &target)
{
    qDebug() << "Controller - "<< __FUNCTION__;
    if(path.size() < 2)
    {
        qWarning() << __FUNCTION__ << " Path with less than 2 elements. Returning";
        return std::make_tuple(0, 0, 0);
    }

    // now y is forward direction and x is pointing rightwards
    float advVel = 0.f, sideVel = 0.f, rotVel = 0.f;
    //auto firstPointInPath = points.front();
    bool active = true;
    bool blocked = false;
    // Compute euclidean distance to target
    float euc_dist_to_target = (robot_pose - target).norm();

    auto is_increasing = [](float new_val)
    { static float ant_value = 0.f;
        bool res = false;
        if( new_val - ant_value > 0 ) res = true;
        ant_value = new_val;
        return res;
    };

    // Target achieved
    std::cout << std::boolalpha << __FUNCTION__ << " Conditions: n points < 2 " << (path.size() < 2)
              << " dist < 200 " << (euc_dist_to_target < consts.final_distance_to_target);

    if ( (path.size() < 2) and (euc_dist_to_target < consts.final_distance_to_target))// or is_increasing(euc_dist_to_target))
    {
        qInfo() << __FUNCTION__ << "Target achieved";
        advVel = 0;  sideVel= 0; rotVel = 0;
        active = false;
        robot_is_active = false;
        custom_widget.startButton->setText("Start");
        return std::make_tuple(0,0,0);  //adv, side, rot
    }

    // lambda to convert from Eigen to QPointF
    auto to_QPointF = [](const Eigen::Vector2f &a){ return QPointF(a.x(), a.y());};

    /// Compute rotational speed
    // closest point to robot nose in path
    auto closest_point_to_nose = std::ranges::min_element(path, [robot_nose](auto &a, auto &b){ return (robot_nose - a).norm() < (robot_nose - b).norm();});

    // compute angle between robot-to-nose line and  tangent to closest point in path
    QLineF tangent;
    if(std::distance(path.cbegin(), closest_point_to_nose) == 0)
        tangent = QLineF(to_QPointF(*closest_point_to_nose), to_QPointF(*std::next(path.cbegin(),1)));
    else if(std::distance(closest_point_to_nose, path.cend()) == 0)
        tangent = QLineF(to_QPointF(*closest_point_to_nose), to_QPointF(*path.cend()));
    else tangent = QLineF(to_QPointF(*(closest_point_to_nose - 1)), to_QPointF(*(closest_point_to_nose + 1)));
    QLineF robot_to_nose(to_QPointF(robot_pose), to_QPointF(robot_nose));
    float angle = rewrapAngleRestricted(qDegreesToRadians(robot_to_nose.angleTo(tangent)));

    // compute distance to path to cancel stationary error
    auto e_tangent = Eigen::Hyperplane<float, 2>::Through(Eigen::Vector2f(tangent.p1().x(), tangent.p1().y()),
                                                          Eigen::Vector2f(tangent.p2().x(), tangent.p2().y()));
    float signed_distance = e_tangent.signedDistance(robot_nose);
    float correction = consts.lateral_correction_gain * tanh(signed_distance);
    qInfo() << __FUNCTION__  << " angle error: " << angle << "correction: " << correction;
    //angle +=  correction;
    sideVel = consts.lateral_correction_for_side_velocity * correction;

    // rot speed gain
    rotVel = 2*angle;  // pioneer
    rotVel = consts.rotation_gain * angle;  // viriato

    // limit angular  values to physical limits
    rotVel = std::clamp(rotVel, -consts.max_rot_speed, consts.max_rot_speed);
    // cancel final rotation
    if(euc_dist_to_target < consts.times_final_distance_to_target_before_zero_rotation * consts.final_distance_to_target)
          rotVel = 0.f;

    /// Compute advance speed
    advVel = std::min(consts.max_adv_speed * exponentialFunction(rotVel, consts.advance_gaussian_cut_x, consts.advance_gaussian_cut_y, 0),
                      euc_dist_to_target);

    /// Compute bumper-away speed
    QVector2D total{0, 0};
//    const auto &[angles, dists] = laser_data;
//    for (const auto &[angle, dist] : iter::zip(angles, dists))
//    {
//        float limit = (fabs(ROBOT_LENGTH / 2.f * sin(angle)) + fabs(ROBOT_LENGTH / 2.f * cos(angle))) + 200;
//        float diff = limit - dist;
//        if (diff >= 0)
//            total = total + QVector2D(-diff * cos(angle), -diff * sin(angle));
//    }

    /// Compute bumper away speed for rectangular shape
    // get extendedrobot polygon in worlds's coordinate frame
//    std::vector<QPointF> rp = get_points_along_extended_robot_polygon(200, 40);
//    for (const auto &p : rp)
//        if(not laser_poly.containsPoint(p, Qt::OddEvenFill))
//            total = total + QVector2D(p);
//    qInfo() << __FUNCTION__ << total;
//    sideVel = std::clamp(total.y(), -MAX_SIDE_SPEED, MAX_SIDE_SPEED);
    return std::make_tuple(advVel, sideVel, rotVel);
}

std::vector<QPointF> SpecificWorker::get_points_along_extended_robot_polygon(int offset, int chunck)
{
    static QGraphicsRectItem *poly_draw = nullptr;
    std::vector<QPointF> poly;
    QRectF rp(QPointF(robotTopLeft.x(),robotTopLeft.y()), QPointF(robotBottomRight.x(), robotBottomRight.y()));
    rp.adjust(-offset, offset, offset, -offset);
    QLineF bottom(rp.bottomLeft(), rp.bottomRight());
    QLineF left(rp.topLeft(), rp.bottomLeft());
    QLineF top(rp.topLeft(), rp.topRight());
    QLineF right(rp.topRight(), rp.bottomRight());
    for(auto i : iter::range(0.f, 1.f, (float)(1.f/(top.length()/chunck))))
    {
        poly.push_back(top.pointAt(i));
        poly.push_back(bottom.pointAt(i));
        //auto aux = inner_eigen->transform(world_name, Eigen::Vector3d{point.x(), point.y(), 0.f}, robot_name);
        //poly << QPointF(aux.value().x(), aux.value().y());
        //point =  bottom.pointAt(i);
        //aux = inner_eigen->transform(world_name, Eigen::Vector3d{point.x(), point.y(), 0.f}, robot_name);
        //poly << QPointF(aux.value().x(), aux.value().y());
    }
    for(auto i : iter::range(0.f, 1.f, (float)(1.f/(left.length()/chunck))))
    {
        poly.push_back(left.pointAt(i));
        poly.push_back(right.pointAt(i));
        //        auto point =  left.pointAt(i);
        //        auto aux = inner_eigen->transform(world_name, Eigen::Vector3d{point.x(), point.y(), 0.f}, robot_name);
        //        poly << QPointF(aux.value().x(), aux.value().y());
        //        point =  right.pointAt(i);
        //        aux = inner_eigen->transform(world_name, Eigen::Vector3d{point.x(), point.y(), 0.f}, robot_name);
        //        poly << QPointF(aux.value().x(), aux.value().y());
    }
    if(poly_draw != nullptr)
    {
        widget_2d->scene.removeItem(poly_draw);
        delete poly_draw;
    }
    poly_draw = widget_2d->scene.addRect(rp, QPen(QColor("blue"), 10));
    auto robot_pos = inner_eigen->transform_axis(world_name, robot_name).value();
    poly_draw->setRotation(qRadiansToDegrees(robot_pos[5]));
    poly_draw->setPos(QPointF(robot_pos[0], robot_pos[1]));
    return poly;
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

//std::tuple<float, float, float> SpecificWorker::send_command_to_robot(const std::tuple<float, float, float> &speeds) //adv, side, rot
//{
//    static QMat adv_conv = QMat::afinTransformFromIntervals(QList<QPair<QPointF,QPointF>>{QPair<QPointF,QPointF>{QPointF{-MAX_ADV_SPEED,MAX_ADV_SPEED}, QPointF{-20,20}}});
//    static QMat rot_conv = QMat::afinTransformFromIntervals(QList<QPair<QPointF,QPointF>>{QPair<QPointF,QPointF>{QPointF{-MAX_ROT_SPEED,MAX_ROT_SPEED}, QPointF{-15,15}}});
//    static QMat side_conv = QMat::afinTransformFromIntervals(QList<QPair<QPointF,QPointF>>{QPair<QPointF,QPointF>{QPointF{-MAX_SIDE_SPEED,MAX_SIDE_SPEED}, QPointF{-15,15}}});
//
//    auto &[adv_, side_, rot_] = speeds;
//    auto adv = (adv_conv * QVec::vec2(adv_,1.0))[0];
//    auto rot = (rot_conv * QVec::vec2(rot_,1.0))[0];
//    auto side = (side_conv * QVec::vec2(side_, 1.0))[0];
//    auto robot_node = G->get_node(robot_name);
//    G->add_or_modify_attrib_local<robot_ref_adv_speed_att>(robot_node.value(),  (float)adv);
//    G->add_or_modify_attrib_local<robot_ref_rot_speed_att>(robot_node.value(), (float)rot);
//    G->add_or_modify_attrib_local<robot_ref_side_speed_att>(robot_node.value(),  (float)side);
//    G->update_node(robot_node.value());
//    return std::make_tuple(adv, side, rot);
//}

// compute max de gauss(value) where gauss(x)=y  y min
float SpecificWorker::exponentialFunction(float value, float xValue, float yValue, float min)
{
    if (yValue <= 0)
        return 1.f;
    float landa = -fabs(xValue) / log(yValue);
    float res = exp(-fabs(value) / landa);
    return std::max(res, min);
}

float SpecificWorker::rewrapAngleRestricted(const float angle)
{
    if (angle > M_PI)
        return angle - M_PI * 2;
    else if (angle < -M_PI)
        return angle + M_PI * 2;
    else
        return angle;
}

///////////////////////////////////////////////////////
//// Check new target from mouse
///////////////////////////////////////////////////////
void SpecificWorker::new_target_from_mouse(int pos_x, int pos_y, int id)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
/// Asynchronous changes on G nodes from G signals
/////////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::add_or_assign_node_slot(const std::uint64_t id, const std::string &type)
{
    //check node type
    if (type == path_to_target_type_name)
    {
        if( auto node = G->get_node(id); node.has_value())
        {
            auto x_values = G->get_attrib_by_name<path_x_values_att>(node.value());
            auto y_values = G->get_attrib_by_name<path_y_values_att>(node.value());
            if(x_values.has_value() and y_values.has_value())
            {
                std::vector<Eigen::Vector2f> path;
                auto x = x_values.value().get();
                auto y = y_values.value().get();
                for (const auto &[x, y] : iter::zip(x, y))
                    path.emplace_back(Eigen::Vector2f(x, y));
                path_buffer.put(std::move(path));
                auto t_x = G->get_attrib_by_name<path_target_x_att>(node.value());
                auto t_y = G->get_attrib_by_name<path_target_y_att>(node.value());
                if(t_x.has_value() and t_y.has_value())
                    current_target = Eigen::Vector2f(t_x.value(), t_y.value());
            }
        }
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
                //if(dists.value().get().empty() or angles.value().get().empty()) return;
                //qInfo() << __FUNCTION__ << dists->get().size();

                std::tuple<std::vector<float>, std::vector<float>> && tuple = std::make_tuple(angles.value().get(), dists.value().get());
                laser_buffer.put(std::move(tuple),
                                 [this](LaserData &&in, std::tuple<std::vector<float>, std::vector<float>, QPolygonF, std::vector<QPointF>> &out)
                                 {
                                     QPolygonF laser_poly;
                                     std::vector<QPointF> laser_cart;
                                     auto &&[angles, dists] = in;
                                     for (auto &&[angle, dist] : iter::zip(std::move(angles), std::move(dists)))
                                     {
                                         //convert laser polar coordinates to cartesian
                                         float x = dist * sin(angle);
                                         float y = dist * cos(angle);
                                         Mat::Vector3d laserWorld = inner_eigen->transform(world_name,Mat::Vector3d(x, y, 0), laser_name).value();
                                         laser_poly << QPointF(x, y);
                                         laser_cart.emplace_back(QPointF(laserWorld.x(), laserWorld.y()));
                                     }
                                     out = std::make_tuple(angles, dists, laser_poly, laser_cart);
                                 });
            }
        }
    }
}

void SpecificWorker::del_node_slot(std::uint64_t from)
{
        if( auto node = G->get_node(current_path_name); not node.has_value())
            qInfo() << __FUNCTION__  << "Path node deleter. Aborting control";
}

///////////////////////////////////////////////////
/// GUI
//////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}
/**************************************/

void SpecificWorker::draw_path(std::vector<Eigen::Vector2f> &path, QGraphicsScene* viewer_2d)
{
    static std::vector<QGraphicsLineItem *> scene_road_points;

    //clear previous points

    for (QGraphicsLineItem* item : scene_road_points)
    {
        viewer_2d->removeItem(item);
        delete item;
    }
    scene_road_points.clear();

    /// Draw all points
    QGraphicsLineItem *line1, *line2;
    std::string color;
    for(auto &&p_pair : iter::sliding_window(path, 2))
    {
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

        line1 = viewer_2d->addLine(qsegment, QPen(QBrush(QColor(QString::fromStdString(color))), 20));
        line2 = viewer_2d->addLine(qsegment_perp, QPen(QBrush(QColor(QString::fromStdString("#F0FF00"))), 20));

        scene_road_points.push_back(line1);
        scene_road_points.push_back(line2);
    }
}
