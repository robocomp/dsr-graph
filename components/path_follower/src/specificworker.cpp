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
#include <algorithm>
#include <QPointF>

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
	G.reset();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	conf_params  = std::make_shared<RoboCompCommonBehavior::ParameterList>(params);
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
	std::cout << __FUNCTION__ << std::endl;
	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		// create graph
		G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, "", dsrgetid_proxy); // Init nodes
		std::cout<< __FUNCTION__ << "Graph loaded" << std::endl;  

        // Graph viewer
		using opts = DSR::DSRViewer::view;
		int current_opts = 0;
		//opts main = opts::none;
		if(tree_view)
			current_opts = current_opts | opts::tree;
		if(graph_view)
			current_opts = current_opts | opts::graph;
		if(qscene_2d_view)
			current_opts = current_opts | opts::scene;
		if(osg_3d_view)
			current_opts = current_opts | opts::osg;
		dsr_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts);
        setWindowTitle(QString::fromStdString(agent_name + "-" + std::to_string(agent_id)));

		connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::update_node_slot);
        //connect(G.get(), &DSR::DSRGraph::update_attrs_signal, this, &SpecificWorker::update_attrs_slot);

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
            widget_2d->set_draw_laser(true);

        // path planner
        path_follower_initialize();

        // check for existing intention node
        if(auto paths = G->get_nodes_by_type(path_to_target_type); not paths.empty())
            this->update_node_slot(paths.front().id(), path_to_target_type);

        this->Period = 200;
        std::cout<< __FUNCTION__ << "Initialization finished" << std::endl;
        timer.start(Period);
	}
}

void SpecificWorker::compute()
{
    static std::vector<QPointF> path;

    if(not robot_is_active) return;

    // Check for existing path_to_target_nodes
    if (auto path_o = path_buffer.try_get(); path_o.has_value()) // NEW PATH!
    {
        path.clear();
        path = path_o.value();
    }
    // keep controlling
    if( const auto laser_data = laser_buffer.try_get(); laser_data.has_value())
    {
        const auto &[angles, dists, laser_poly, laser_cart] = laser_data.value();
        // qInfo() << "Path: " << path.size()  << " Laser size:" << laser_poly.size();
        // for (auto &&p:path) qInfo() << p;
        auto nose_3d = inner_eigen->transform(world_name, Mat::Vector3d(0, 360, 0), robot_name).value();
        auto robot_pose_3d = inner_eigen->transform(world_name, robot_name).value();
        auto robot_nose = QPointF(nose_3d.x(), nose_3d.y());
        auto robot_pose = QPointF(robot_pose_3d.x(), robot_pose_3d.y());
        auto speeds = update(path, LaserData{angles, dists}, robot_pose, robot_nose, current_target);
        auto [adv,side,rot] =  send_command_to_robot(speeds);
        std::cout << "Dist to target: " << std::endl;
        std::cout << "\t " << QVector2D(robot_pose - current_target).length() << std::endl;
        std::cout << "Ref speeds:  " << std::endl;
        std::cout << "\t Adv-> " << adv << std::endl;
        std::cout << "\t Side -> " << side << std::endl;
        std::cout << "\tRot -> " << rot << std::endl;
    }
}

void SpecificWorker::path_follower_initialize()
{
    qDebug()<< "Controller - " << __FUNCTION__;
    try
    {
        MAX_ADV_SPEED = QString::fromStdString(conf_params->at("MaxAdvanceSpeed").value).toFloat();
        MAX_ROT_SPEED = QString::fromStdString(conf_params->at("MaxRotationSpeed").value).toFloat();
        MAX_SIDE_SPEED = QString::fromStdString(conf_params->at("MaxSideSpeed").value).toFloat();
        MAX_LAG = std::stof(conf_params->at("MinControllerPeriod").value);
        ROBOT_RADIUS_MM =  QString::fromStdString(conf_params->at("RobotRadius").value).toFloat();
        qDebug()<< __FUNCTION__ << "CONTROLLER: Params from config:"  << MAX_ADV_SPEED << MAX_ROT_SPEED << MAX_SIDE_SPEED << MAX_LAG << ROBOT_RADIUS_MM;
    }
    catch (const std::out_of_range& oor)
    {
        std::cout << "CONTROLLER. Out of Range error reading parameters: " << oor.what() << '\n';
        std::terminate();
    }
    //    robotXWidth = std::stof(conf_params->at("RobotXWidth").value);
    //    robotZLong = std::stof(conf_params->at("RobotZLong").value);
    //    robotBottomLeft     = Mat::Vector3d ( -robotXWidth / 2, robotZLong / 2, 0);
    //    robotBottomRight    = Mat::Vector3d ( - robotXWidth / 2,- robotZLong / 2, 0);
    //    robotTopRight       = Mat::Vector3d ( + robotXWidth / 2, - robotZLong / 2, 0);
    //    robotTopLeft        = Mat::Vector3d ( + robotXWidth / 2, + robotZLong / 2, 0);
}

std::tuple<float, float, float> SpecificWorker::update(const std::vector<QPointF> &path, const LaserData &laser_data, const QPointF &robot_pose, const QPointF &robot_nose, const QPointF &target)
{
    qDebug() << "Controller - "<< __FUNCTION__;
    if(path.size() < 2)
        return std::make_tuple(0,0,0);

    // now y is forward direction and x is pointing rightwards
    float advVel = 0.f, sideVel = 0.f, rotVel = 0.f;
    //auto firstPointInPath = points.front();
    bool active = true;
    bool blocked = false;
    QPointF robot = QPointF(robot_pose.x(), robot_pose.y());
    // Compute euclidean distance to target
    float euc_dist_to_target = QVector2D(robot - target).length();

    auto is_increasing = [](float new_val)
    { static float ant_value = 0.f;
        bool res = false;
        if( new_val - ant_value > 0 ) res = true;
        ant_value = new_val;
        return res;
    };

    // Target achieved
    if ( (path.size() < 3) and (euc_dist_to_target < FINAL_DISTANCE_TO_TARGET or is_increasing(euc_dist_to_target)))
    {
        advVel = 0;  sideVel= 0; rotVel = 0;
        active = false;
        std::cout << std::boolalpha << __FUNCTION__ << " Target achieved. Conditions: n points < 3 " << (path.size() < 3)
        << " dist < 100 " << (euc_dist_to_target < FINAL_DISTANCE_TO_TARGET)
        << " der_dist > 0 " << is_increasing(euc_dist_to_target)  << std::endl;
        return std::make_tuple(0,0,0);  //adv, side, rot
    }

    /// Compute rotational speed
    QLineF robot_to_nose(robot, robot_nose);
    float angle = rewrapAngleRestricted(qDegreesToRadians(robot_to_nose.angleTo(QLineF(robot_nose, path[1]))));
    if(angle >= 0)
        rotVel = std::clamp(angle, 0.f, MAX_ROT_SPEED);
    else
        rotVel = std::clamp(angle, -MAX_ROT_SPEED, 0.f);
    if(euc_dist_to_target < 4*FINAL_DISTANCE_TO_TARGET)
        rotVel = 0.f;

    /// Compute advance speed
    std::min(advVel = MAX_ADV_SPEED * exponentialFunction(rotVel, 1.5, 0.1, 0), euc_dist_to_target);

    /// Compute bumper-away speed
    QVector2D total{0, 0};
    const auto &[angles, dists] = laser_data;
    for (const auto &[angle, dist] : iter::zip(angles, dists))
    {
        float limit = (fabs(ROBOT_LENGTH / 2.f * sin(angle)) + fabs(ROBOT_LENGTH / 2.f * cos(angle))) + 200;
        float diff = limit - dist;
        if (diff >= 0)
            total = total + QVector2D(-diff * cos(angle), -diff * sin(angle));
    }
    QVector2D bumperVel = total * KB;  // Parameter set in slidebar
    if (abs(bumperVel.y()) < MAX_SIDE_SPEED)
    sideVel = bumperVel.y();

    //qInfo() << advVelz << advVelx << rotVel;
    return std::make_tuple(advVel, sideVel, rotVel);
    //return std::make_tuple (true, blocked, active, advVelz, advVelx, rotVel); //side, adv, rot
}

std::tuple<float, float, float> SpecificWorker::send_command_to_robot(const std::tuple<float, float, float> &speeds) //adv, side, rot
{
    static QMat adv_conv = QMat::afinTransformFromIntervals(QList<QPair<QPointF,QPointF>>{QPair<QPointF,QPointF>{QPointF{-MAX_ADV_SPEED,MAX_ADV_SPEED}, QPointF{-20,20}}});
    static QMat rot_conv = QMat::afinTransformFromIntervals(QList<QPair<QPointF,QPointF>>{QPair<QPointF,QPointF>{QPointF{-MAX_ROT_SPEED,MAX_ROT_SPEED}, QPointF{-15,15}}});
    static QMat side_conv = QMat::afinTransformFromIntervals(QList<QPair<QPointF,QPointF>>{QPair<QPointF,QPointF>{QPointF{-MAX_SIDE_SPEED,MAX_SIDE_SPEED}, QPointF{-15,15}}});

    auto &[adv_, side_, rot_] = speeds;

    auto adv = (adv_conv * QVec::vec2(adv_,1.0))[0];
    auto rot = (rot_conv * QVec::vec2(rot_,1.0))[0];
    auto side = (side_conv * QVec::vec2(side_, 1.0))[0];
    auto robot_node = G->get_node(robot_name);
    G->add_or_modify_attrib_local<ref_adv_speed_att>(robot_node.value(),  (float)adv);
    G->add_or_modify_attrib_local<ref_rot_speed_att>(robot_node.value(), (float)rot);
    G->add_or_modify_attrib_local<ref_side_speed_att>(robot_node.value(),  (float)side);
    G->update_node(robot_node.value());
    return std::make_tuple(adv, side, rot);
}

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

///////////////////////////////////////////////////
/// Asynchronous changes on G nodes from G signals
///////////////////////////////////////////////////
void SpecificWorker::update_node_slot(const std::int32_t id, const std::string &type)
{
    //check node type
    if (type == path_to_target_type)
    {
        if( auto node = G->get_node(id); node.has_value())
        {
            auto x_values = G->get_attrib_by_name<path_x_values_att>(node.value());
            auto y_values = G->get_attrib_by_name<path_y_values_att>(node.value());
            if(x_values.has_value() and y_values.has_value())
            {
                std::vector<QPointF> path;
                for (const auto &[x, y] : iter::zip(x_values.value().get(), y_values.value().get()))
                    path.emplace_back(QPointF(x, y));
                path_buffer.put(path);
                auto t_x = G->get_attrib_by_name<path_target_x_att>(node.value());
                auto t_y = G->get_attrib_by_name<path_target_y_att>(node.value());
                if(t_x.has_value() and t_y.has_value())
                    current_target = QPointF(t_x.value(), t_y.value());
            }
        }
    }
    else if (type == laser_type)    // Laser node updated
    {
        //qInfo() << __FUNCTION__ << " laser node change";
        if( auto node = G->get_node(id); node.has_value())
        {
            auto angles = G->get_attrib_by_name<laser_angles_att>(node.value());
            auto dists = G->get_attrib_by_name<laser_dists_att>(node.value());
            if(dists.has_value() and angles.has_value())
            {
                if(dists.value().get().empty() or angles.value().get().empty()) return;
                //qInfo() << __FUNCTION__ << dists->get().size();
                laser_buffer.put(std::make_tuple(angles.value().get(), dists.value().get()),
                                 [this](const LaserData &in, std::tuple<std::vector<float>, std::vector<float>, QPolygonF,std::vector<QPointF>> &out) {
                                     QPolygonF laser_poly;
                                     std::vector<QPointF> laser_cart;
                                     const auto &[angles, dists] = in;
                                     for (const auto &[angle, dist] : iter::zip(angles, dists))
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

void SpecificWorker::update_attrs_slot(const std::int32_t id, const std::map<string, DSR::Attribute> &attribs)
{
    //qInfo() << "Update attr " << id;
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