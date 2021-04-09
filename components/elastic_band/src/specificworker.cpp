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
		G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id); // Init nodes
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
        G->set_ignored_attributes<cam_rgb_att , cam_depth_att>();

        // Custom widget
        dsr_viewer->add_custom_widget_to_dock("Elastic band", &custom_widget);
		widget_2d = qobject_cast<DSR::QScene2dViewer*> (dsr_viewer->get_widget(opts::scene));
        if(widget_2d)
            widget_2d->set_draw_laser(true);

		// path planner
		elastic_band_initialize();

		// check for existing intention node
		if(auto paths = G->get_nodes_by_type(path_to_target_type); not paths.empty())
            this->update_node_slot(paths.front().id(), path_to_target_type);

		this->Period = 100;
        std::cout<< __FUNCTION__ << "Initialization finished" << std::endl;
        timer.start(Period);
	}
}

void SpecificWorker::compute()
{
    static std::vector<QPointF> path;

    // Check for existing path_to_target_nodes
    if (auto path_o = path_buffer.try_get(); path_o.has_value())
    {
        path.clear();
        path = path_o.value();
    }
    else
    {
        if( const auto laser_data = laser_buffer.try_get(); laser_data.has_value())
        {
            const auto &[laser_poly, laser_cart] = laser_data.value();
            auto nose_3d = inner_eigen->transform(world_name, Mat::Vector3d(0, 360, 0), robot_name).value();
            auto current_robot_nose = QPointF(nose_3d.x(), nose_3d.y());
            auto current_robot_polygon = get_robot_polygon();  //in world coordinates. Think of a transform_multi
            compute_forces(path, laser_cart, laser_poly, current_robot_polygon, current_robot_nose);
            clean_points(path, laser_poly, current_robot_polygon);
            add_points(path, laser_poly, current_robot_polygon);
            draw_path(path, &widget_2d->scene, laser_poly);
            save_path_in_G(path);
        }
    }
}

void SpecificWorker::elastic_band_initialize()
{
    robotXWidth = std::stof(conf_params->at("RobotXWidth").value);
    robotZLong = std::stof(conf_params->at("RobotZLong").value);
    robotBottomLeft     = Mat::Vector3d ( -robotXWidth / 2, robotZLong / 2, 0);
    robotBottomRight    = Mat::Vector3d ( - robotXWidth / 2,- robotZLong / 2, 0);
    robotTopRight       = Mat::Vector3d ( + robotXWidth / 2, - robotZLong / 2, 0);
    robotTopLeft        = Mat::Vector3d ( + robotXWidth / 2, + robotZLong / 2, 0);
}

void SpecificWorker::compute_forces(std::vector<QPointF> &path,
                                    const vector<QPointF> &laser_cart,
                                    const QPolygonF &laser_poly,
                                    const QPolygonF &current_robot_polygon,
                                    const QPointF &current_robot_nose)
{
    if (path.size() < 3)
        return;
    int nonVisiblePointsComputed = 0;

    // Go through points using a sliding windows of 3
    for (auto &&[i, group] : iter::enumerate(iter::sliding_window(path, 3)))
    {
        if (group.size() < 3)
            continue; // break if too short

        auto p1 = QVector2D(group[0]);
        auto p2 = QVector2D(group[1]);
        auto p3 = QVector2D(group[2]);
        if(p1==p2 or p2==p3)
            continue;
        QPointF p = group[1];
        int index_of_p_in_path = i+1;  //index of p in path

        ////////////////////////////////
        /// INTERNAL curvature forces on p2. Stretches the path locally
        /// Approximates the angle between adjacent segments: p2->p1, p2->p3
        ////////////////////////////////
        QVector2D iforce = ((p1 - p2) / (p1 - p2).length() + (p3 - p2) / (p3 - p2).length());

        ////////////////////////////////////////////
        /// External forces caused by obstacles repulsion field
        ///////////////////////////////////////////7
        float min_dist;
        QVector2D eforce;

        qDebug() << __FUNCTION__  << nonVisiblePointsComputed;

//        // compute forces from map on not visible points
        if ( not is_visible(p, laser_poly ))
            continue;
//        {
//            auto [obstacleFound, vectorForce] = grid.vectorToClosestObstacle(p);
//            if (( not obstacleFound) or (nonVisiblePointsComputed > 10))
//            {
//                qDebug ()  << __FUNCTION__ << "No obstacles found in map for not visible point or it is more than 10 not visible points away";
//                nonVisiblePointsComputed++;
//                continue;
//            }
//            else
//            {
//                qDebug()  << __FUNCTION__  << "--- Obstacle found in grid ---";
//                min_dist = vectorForce.length() - (ROBOT_LENGTH / 2);   // subtract robot semi-width
//                if (min_dist <= 0)    // hard limit to close obstables
//                    min_dist = 0.01;
//                eforce = vectorForce;
//            }
//            nonVisiblePointsComputed++;
//        }

        // compute forces from laser on visible point
//       else
       // {
            // vector holding a) distance from laser tip to p, vector from laser tip to p, laser tip plane coordinates
            std::vector<std::tuple<float, QVector2D, QPointF>> distances;
            // Apply to all laser points a functor to compute the distances to point p2. laser_cart must be up to date
            std::transform(std::begin(laser_cart), std::end(laser_cart), std::back_inserter(distances), [p, RL=ROBOT_LENGTH](const QPointF &laser)
            {   // compute distance from laser measure to point minus RLENGTH/2 or 0 and keep it positive
                float dist = (QVector2D(p) - QVector2D(laser)).length() - (RL / 2);
                if (dist <= 0)
                    dist = 0.01;
                return std::make_tuple(dist,  QVector2D(p)-QVector2D(laser), laser);
            });
            // compute min of all laser to p distances
            auto min = std::min_element(std::begin(distances), std::end(distances), [](auto &a, auto &b)
            {
                return std::get<float>(a) < std::get<float>(b);
            });
            min_dist = std::get<float>(*min);
            eforce = std::get<QVector2D>(*min);
        //}
        /// Note: instead of min, we could compute the resultant of all forces acting on the point, i.e. inside a given radius.
        /// a logarithmic law can be used to compute de force from the distance.
        /// To avoid constants, we need to compute de Jacobian of the sum of forces wrt the (x,y) coordinates of the point

        // rescale min_dist so 1 is ROBOT_LENGTH
        float magnitude = (1.f / ROBOT_LENGTH) * min_dist;
        // compute inverse square law
        magnitude = 10.f / (magnitude * magnitude);
        //if(magnitude > 25) magnitude = 25.;
        QVector2D f_force = magnitude * eforce.normalized();

        // Remove tangential component of repulsion force by projecting on line tangent to path (base_line)
//        QVector2D base_line = (p1 - p3).normalized();
//        const QVector2D itangential = QVector2D::dotProduct(f_force, base_line) * base_line;
//        f_force = f_force - itangential;

        // update node pos. KI and KE are approximating inverse Jacobians modules. This should be CHANGED
        // Directions are taken as the vector going from p to closest obstacle.
        auto total = (KI * iforce) + (KE * f_force);
        //
        // limiters CHECK!!!!!!!!!!!!!!!!!!!!!!
        if (total.length() > 30)
            total = 8 * total.normalized();
        if (total.length() < -30)
            total = -8 * total.normalized();

        /// Compute additional restrictions to be forced in the minimization process
        // A) Check boundaries for final displacements
        // A.1) Move nodes only if it does not move inside objects
        // A.2) Does not move underneath the robot.
        // A.3) Does not exit the laser polygon
        QPointF temp_p = p + total.toPointF();
        //qInfo()  << __FUNCTION__  << "Total force "<< total.toPointF()<< " New Point "<< temp_p;
        if (is_point_visitable(temp_p) and (not current_robot_polygon.containsPoint(temp_p, Qt::OddEvenFill))
            //and (std::none_of(std::begin(intimateSpaces), std::end(intimateSpaces),[temp_p](const auto &poly) { return poly.containsPoint(temp_p, Qt::OddEvenFill);}))
            //and (std::none_of(std::begin(personalSpaces), std::end(personalSpaces),[temp_p](const auto &poly) { return poly.containsPoint(temp_p, Qt::OddEvenFill);}))
                )
        {
            path[index_of_p_in_path] = temp_p;
        }
//            if( auto it = find_if(pathPoints.begin(), pathPoints.end(), [p] (auto & s){ return (s.x() == p.x() and s.y() == p.y() );}); it != pathPoints.end())
//            {
//                int index = std::distance(pathPoints.begin(), it);
//                pathPoints[index] = temp_p;
//            }
    }
    // Check if robot nose is inside the laser polygon
    if(is_visible(current_robot_nose, laser_poly))
        path[0] = current_robot_nose;
    else
        qWarning() << __FUNCTION__  << "Robot Nose not visible -- NEEDS REPLANNING ";
}

void SpecificWorker::clean_points(std::vector<QPointF> &path, const QPolygonF &laser_poly,  const QPolygonF &current_robot_polygon)
{
    qDebug() << __FUNCTION__;
    std::vector<QPointF> points_to_remove;
    for (const auto &group : iter::sliding_window(path, 2))
    {
        const auto &p1 = group[0];
        const auto &p2 = group[1];

        if (not is_visible(p1, laser_poly) or not is_visible(p2, laser_poly)) //not visible
            continue;

        if (p2 == path.back())
            break;
        // check if p1 was marked to erase in the previous iteration
        if (std::find(std::begin(points_to_remove), std::end(points_to_remove), p1) != std::end(points_to_remove))
            continue;

        float dist = QVector2D(p1 - p2).length();
        qDebug() << __FUNCTION__ << " dist <" << dist << 0.5 * ROAD_STEP_SEPARATION;
        if (dist < 0.5 * ROAD_STEP_SEPARATION)
            points_to_remove.push_back(p2);

        else if(current_robot_polygon.containsPoint(p2, Qt::OddEvenFill))
        {
            qDebug()<<"-------------" << __FUNCTION__ << "------------- Removing point inside robot ";
            points_to_remove.push_back(p2);
        }
    }
    qDebug() << __FUNCTION__ << "Removed: " << points_to_remove.size();
    for (auto &&p : points_to_remove)
        path.erase(std::remove_if(path.begin(), path.end(), [p](auto &r) { return p == r; }), path.end());
}

void SpecificWorker::add_points(std::vector<QPointF> &path, const QPolygonF &laser_poly, const QPolygonF &current_robot_polygon)
{
    // qDebug()<<"Navigation - "<< __FUNCTION__;
    std::vector<std::tuple<int, QPointF>> points_to_insert;
    for (auto &&[k, group] : iter::enumerate(iter::sliding_window(path, 2)))
    {
        auto &p1 = group[0];
        auto &p2 = group[1];

        if ( not is_visible(p1, laser_poly) or not is_visible(p2, laser_poly)) //not visible
            continue;

        float dist = QVector2D(p1 - p2).length();
        qDebug() << __FUNCTION__ << " dist >" << dist << ROAD_STEP_SEPARATION;
        if (dist > ROAD_STEP_SEPARATION)
        {
            //Crucial que el punto se ponga mas cerca que la condición de entrada
            float l = 0.9 * ROAD_STEP_SEPARATION / dist;
            QLineF line(p1, p2);
            points_to_insert.emplace_back(k + 1, QPointF{line.pointAt(l)});
        }
    }
    qDebug() << __FUNCTION__ << "Added: " << points_to_insert.size();
    for (const auto &[l, p] : iter::enumerate(points_to_insert))
    {
        if ( not current_robot_polygon.containsPoint(std::get<QPointF>(p), Qt::OddEvenFill))
            path.insert(path.begin() + std::get<int>(p) + l, std::get<QPointF>(p));
    }
}

bool SpecificWorker::is_point_visitable(QPointF point)
{
    return true;  //// NEEDS the GRID
}

QPolygonF SpecificWorker::get_robot_polygon()
{
    QPolygonF robotP;
    auto bLWorld = inner_eigen->transform(world_name, robotBottomLeft, robot_name);
    auto bRWorld = inner_eigen->transform(world_name, robotBottomRight, robot_name);
    auto tRWorld = inner_eigen->transform(world_name, robotTopRight, robot_name);
    auto tLWorld = inner_eigen->transform(world_name, robotTopLeft, robot_name);
    robotP << QPointF(bLWorld.value().x(),bLWorld.value().y());
    robotP << QPointF(bRWorld.value().x(),bRWorld.value().y());
    robotP << QPointF(tRWorld.value().x(),tRWorld.value().y());
    robotP << QPointF(tLWorld.value().x(),tLWorld.value().y());
    return robotP;
}

bool SpecificWorker::is_visible(QPointF p, const QPolygonF &laser_poly)
{
    std::optional<Mat::Vector3d> pointInLaser = inner_eigen->transform(laser_name, Mat::Vector3d (p.x(),p.y(), 0), world_name);
    return laser_poly.containsPoint(QPointF(pointInLaser.value().x(), pointInLaser.value().y()), Qt::OddEvenFill);
}

void SpecificWorker::save_path_in_G(const std::vector<QPointF> &path)
{
    if( auto node_path = G->get_node(last_path_id); node_path.has_value())
    {
        std::vector<float> x_points, y_points;
        for(const auto &p : path)
        { x_points.emplace_back(p.x()); y_points.emplace_back(p.y());  }
        G->add_or_modify_attrib_local<path_x_values_att>(node_path.value(), x_points);
        G->add_or_modify_attrib_local<path_y_values_att>(node_path.value(), y_points);
        G->update_node(node_path.value());
    }
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
void SpecificWorker::update_node_slot(const std::uint64_t id, const std::string &type)
{
    // PATH_TO_TARGET
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
                last_path_id = id;
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
                                 [this](const LaserData &in, std::tuple<QPolygonF,std::vector<QPointF>> &out) {
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
                                     out = std::make_tuple(laser_poly, laser_cart);
                                 });
            }
        }
    }
}

void SpecificWorker::update_attrs_slot(const std::uint64_t id, const std::map<string, DSR::Attribute> &attribs)
{
    //qInfo() << "Update attr " << id;
}
///////////////////////////////////////////////////
/// GUI
//////////////////////////////////////////////////
void SpecificWorker::draw_path(std::vector<QPointF> &path, QGraphicsScene* viewer_2d, const QPolygonF &laser_poly)
{
    static std::vector<QGraphicsLineItem *> scene_road_points;
    qDebug() << __FUNCTION__;
    ///////////////////////
    // Preconditions
    ///////////////////////
    if (path.size() == 0)
        return;

    //clear previous points
    for (QGraphicsLineItem* item : scene_road_points)
        viewer_2d->removeItem((QGraphicsItem*)item);
    scene_road_points.clear();

    /// Draw all points
    QGraphicsLineItem *line1, *line2;
    std::string color;
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

//        if(i == 1 or i == path.size()-1)
//            color = "#00FF00"; //Green

        if(is_visible(QPointF(b_point.x(), b_point.y()), laser_poly))
            color = "#F0FF00";
        else
            color = "#FF0000";
        line1 = viewer_2d->addLine(qsegment, QPen(QBrush(QColor(QString::fromStdString(color))), 20));
        line2 = viewer_2d->addLine(qsegment_perp, QPen(QBrush(QColor(QString::fromStdString("#F0FF00"))), 20));

        line1->setZValue(2000);
        line2->setZValue(2000);
        scene_road_points.push_back(line1);
        scene_road_points.push_back(line2);
    }
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

/**************************************/