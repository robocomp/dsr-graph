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

        //dsr update signals
        connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::add_or_assign_node_slot);
        //connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::add_or_assign_edge_slot);
        //connect(G.get(), &DSR::DSRGraph::update_attrs_signal, this, &SpecificWorker::add_or_assign_attrs_slot);
        //connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
        //connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);

        // Graph viewer
		using opts = DSR::DSRViewer::view;
		int current_opts = 0;
		opts main = opts::graph;
		if(tree_view)
			current_opts = current_opts | opts::tree;
		if(graph_view)
			current_opts = current_opts | opts::graph;
		if(qscene_2d_view)
			current_opts = current_opts | opts::scene;
		if(osg_3d_view)
			current_opts = current_opts | opts::osg;
		graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
        setWindowTitle(QString::fromStdString(agent_name + "-" + std::to_string(agent_id)));

        //Inner Api
        inner_eigen = G->get_inner_eigen_api();
        rt_api = G->get_rt_api();

        // Ignore attributes from G
        G->set_ignored_attributes<cam_rgb_att , cam_depth_att>();

        // Custom widget
        graph_viewer->add_custom_widget_to_dock("Elastic band", &custom_widget);
		widget_2d = qobject_cast<DSR::QScene2dViewer*> (graph_viewer->get_widget(opts::scene));

		if(widget_2d != nullptr)
            widget_2d->set_draw_laser(false);

        //widget_2d->set_draw_axis(bool draw);
        connect(custom_widget.ke_slider, &QSlider::valueChanged, [this](auto v){ KE = v;});
        connect(custom_widget.ki_slider, &QSlider::valueChanged, [this](auto v){ KI = v;});;

		// path planner
		elastic_band_initialize();

		// check for existing intention node
		if(auto paths = G->get_nodes_by_type(path_to_target_type_name); not paths.empty())
            this->add_or_assign_node_slot(paths.front().id(), path_to_target_type_name);

		// grid
        QRectF outerRegion;
        auto world_node = G->get_node(world_name).value();
        outerRegion.setLeft(G->get_attrib_by_name<OuterRegionLeft_att>(world_node).value());
        outerRegion.setRight(G->get_attrib_by_name<OuterRegionRight_att>(world_node).value());
        outerRegion.setBottom(G->get_attrib_by_name<OuterRegionBottom_att>(world_node).value());
        outerRegion.setTop(G->get_attrib_by_name<OuterRegionTop_att>(world_node).value());
        if(outerRegion.isNull())
        {
            qWarning() << __FILE__ << __FUNCTION__ << "Outer region of the scene could not be found in G. Aborting";
            std::terminate();
        }
        grid.dim.setCoords(outerRegion.left(), outerRegion.top(), outerRegion.right(), outerRegion.bottom());
        grid.TILE_SIZE = stoi(conf_params->at("TileSize").value);
        collisions =  std::make_shared<Collisions>();
        collisions->initialize(G, conf_params);
        grid.initialize(G, collisions, false);
        if( auto grid_node = G->get_node("current_grid"); grid_node.has_value())
        {
            if (auto grid_as_string = G->get_attrib_by_name<grid_as_string_att>(grid_node.value()); grid_as_string.has_value())
                grid.readFromString(grid_as_string.value());
        }
        qInfo() << "SIZE " << grid.size();
		this->Period = 60;
        std::cout<< __FUNCTION__ << "Initialization finished" << std::endl;
        timer.start(Period);
	}
}

void SpecificWorker::compute()
{
    static std::vector<QPointF> path;   // CHANGE TO LIST to reduce insertion times
    static std::tuple<QPolygonF, std::vector<QPointF>> laser_data;

    // Check for existing path_to_target_nodes
    if (auto path_o = path_buffer.try_get(); path_o.has_value())
    {
        path.clear();
        path = path_o.value();
    }
    else
    {
        // path_node has been deleted, stop processing
        if(auto node_path = G->get_node(current_path_name);node_path.has_value())
        {
            //if( const auto ldata = get_laser_data(); ldata.has_value())
            if( const auto ldata = laser_buffer.try_get(); ldata.has_value())
            {
                std::get<1>(laser_data).clear(); std::get<0>(laser_data).clear();
                laser_data = ldata.value();
            }
            const auto &[laser_poly, laser_cart] = laser_data;
            if(laser_poly.isEmpty() or laser_cart.empty()) return;

            auto nose_3d = inner_eigen->transform(world_name, Mat::Vector3d(0, 360, 0), robot_name).value();
            auto current_robot_nose = QPointF(nose_3d.x(), nose_3d.y());
            auto current_robot_polygon = get_robot_polygon();  //in world coordinates. Think of a transform_multi
            compute_forces(path, laser_cart, laser_poly, current_robot_polygon, current_robot_nose);
            clean_points(path, laser_poly, current_robot_polygon);
            add_points(path, laser_poly, current_robot_polygon);
            if(widget_2d != nullptr)
                draw_path(path, &widget_2d->scene, laser_poly);
            save_path_in_G(path);
        }
        else
            qDebug() << __FUNCTION__ << "No path node";
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
    forces_vector.clear();  // drawing only

    // Go through points using a sliding windows of 3
    for (auto &&[i, group] : iter::enumerate(iter::sliding_window(path, 3)))
    {
        const auto &p1 = QVector2D(group[0]);
        const auto &p2 = QVector2D(group[1]);
        const auto &p3 = QVector2D(group[2]);
        if(p1==p2 or p2==p3)  continue;

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
        QVector2D laser_min_element;

        qDebug() << __FUNCTION__  << nonVisiblePointsComputed;

        // compute forces from G on not visible points
        if ( not is_visible(p, laser_poly ) and grid.size() > 0)
        {
            auto closest_obstacle = grid.closest_obstacle(p);
            if (not closest_obstacle.has_value() or (nonVisiblePointsComputed > 6))
            {
                qDebug ()  << __FUNCTION__ << "No obstacles found in map for not visible point or it is more than 10 not visible points away";
                nonVisiblePointsComputed++;
                continue;
            }
            else
            {
                //qInfo() << __FUNCTION__  << closest_obstacle.value();
                QVector2D vector_to_obs(QVector2D(p) - QVector2D(closest_obstacle.value()));
                min_dist = vector_to_obs.length() - (ROBOT_LENGTH / 2);   // subtract robot semi-width
                min_dist = std::clamp(min_dist, 0.01f, 2000.f);
                eforce = vector_to_obs;
                laser_min_element = QVector2D(closest_obstacle.value());  // for drawing
            }
            nonVisiblePointsComputed++;
        }
        // compute forces from laser on visible point
       else
       {
            // vector holding a) distance from laser tip to p, vector from laser tip to p, laser tip plane coordinates
            std::vector<std::tuple<float, QVector2D, QPointF>> distances;
            // Apply to all laser points a functor to compute the distances to point p2. laser_cart must be up to date
            std::transform(std::begin(laser_cart), std::end(laser_cart), std::back_inserter(distances), [p, RL=ROBOT_LENGTH](const QPointF &laser)
            {   // compute distance from laser measure to point minus RLENGTH/2 or 0 and keep it positive

                //if p is MAX_LASER_RANGE distance should be INF
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
            laser_min_element = QVector2D(std::get<QPointF>(*min));
        }
        // Note: instead of min, we could compute the resultant of all forces acting on the point, i.e. inside a given radius.
        // a logarithmic law can be used to compute de force from the distance.
        // To avoid constants, we need to compute de Jacobian of the sum of forces wrt the (x,y) coordinates of the point

        // rescale min_dist so 1 is ROBOT_LENGTH
        float magnitude = (1.f / ROBOT_LENGTH) * min_dist;
        // compute inverse square law
        magnitude = 10.f / (magnitude * magnitude);
        //if(magnitude > 25) magnitude = 25.;
        QVector2D f_force = magnitude * eforce.normalized();

        // Remove tangential component of repulsion force by projecting on line tangent to path (base_line)
        QVector2D base_line = (p1 - p3).normalized();
        const QVector2D itangential = QVector2D::dotProduct(f_force, base_line) * base_line;
        f_force = f_force - itangential;

        // update node pos. KI and KE are approximating inverse Jacobians modules. This should be CHANGED
        // Directions are taken as the vector going from p to closest obstacle.
        forces_vector.push_back(std::make_tuple(QVector2D(path[index_of_p_in_path]), laser_min_element));
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

        // check if translated point is accepted
        if ( grid.isFree(grid.pointToGrid(temp_p)) and (not current_robot_polygon.containsPoint(temp_p, Qt::OddEvenFill)))
           path[index_of_p_in_path] = temp_p;  //METER UN TRY
    }
    // Check if robot nose is inside the laser polygon
//    if(is_visible(current_robot_nose, laser_poly))
//        path[0] = current_robot_nose;
//    else
//        qWarning() << __FUNCTION__  << "Robot Nose not visible -- NEEDS REPLANNING ";
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
        const auto &p1 = group[0];
        const auto &p2 = group[1];
        if ( not is_visible(p1, laser_poly)) // or not is_visible(p2, laser_poly)) //not visible
            continue;
        if ( k < path.size()-1) // do not insert just before last
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
    return laser_poly.containsPoint(QPointF(pointInLaser.value().x(), pointInLaser.value().y()), Qt::WindingFill);
}
void SpecificWorker::save_path_in_G(const std::vector<QPointF> &path)
{
    if( auto node_path = G->get_node(last_path_id); node_path.has_value())
    {
        std::vector<float> x_points, y_points;
        x_points.reserve(path.size()); y_points.reserve(path.size());
        for(const auto &p : path)
            { x_points.emplace_back(p.x()); y_points.emplace_back(p.y());  }
        G->add_or_modify_attrib_local<path_x_values_att>(node_path.value(), x_points);
        G->add_or_modify_attrib_local<path_y_values_att>(node_path.value(), y_points);
        G->update_node(node_path.value());
    }
}

/////////////////
std::optional<std::tuple<QPolygonF, std::vector<QPointF>>> SpecificWorker::get_laser_data()
{
    if( auto node = G->get_node(laser_name); node.has_value())
    {
        auto angles = G->get_attrib_by_name<laser_angles_att>(node.value());
        auto dists = G->get_attrib_by_name<laser_dists_att>(node.value());
        if (dists.has_value() and angles.has_value())
        {
            const auto &d = dists.value().get();
            const auto &a = angles.value().get();
            if (d.empty() or a.empty()) return {};
            QPolygonF laser_poly;
            std::vector<QPointF> laser_cart;
            laser_cart.reserve(a.size());
            auto robot = widget_2d->get_robot_polygon();
            //auto inner_eigen = G->get_inner_eigen_api();
            for (const auto &[angle, dist] : iter::zip(a, d))
            {
                //convert laser polar coordinates to cartesian
                if (dist == 0) continue;
                float x = dist * sin(angle);
                float y = dist * cos(angle);
                //QPointF p = robot->mapToScene(x, y);
                Mat::Vector3d laserWorld = inner_eigen->transform(world_name,Mat::Vector3d(x, y, 0), robot_name).value();
                //auto diff = Eigen::Vector3d(p.x(),p.y(),0) - laserWorld;
                //qInfo() << p << " - [" << laserWorld.x() << "," << laserWorld.y() << "] = " << "[" << diff.x() << "," << diff.y() << "]";
                //auto r = inner_eigen->transform_axis(world_name, robot_name);
                //auto po = rt_api->get_translation(G->get_node(world_name).value(), G->get_node(robot_name).value().id());
                // if (angle < 0.05 and angle > -0.05)
                //  qInfo() << "q_robot:" << robot->pos() << " g_robot:" << r.value().x() << r.value().y() << r.value()[5];
                //qInfo() << "q_robot:" << robot->pos() << " g_robot:" << po.value().x() << po.value().y() << " g2_robot:" << r.value().x() << r.value().y()  ;
                laser_poly << QPointF(x, y);
                laser_cart.emplace_back(QPointF(laserWorld.x(), laserWorld.y()));
                //laser_cart.emplace_back(QPointF(p.x(), p.y()));
                //laser_cart.emplace_back(p);
            }
            return std::make_tuple(laser_poly, laser_cart);
        }
    }
    return {};
}

///////////////////////////////////////////////////
/// Asynchronous changes on G nodes from G signals
///////////////////////////////////////////////////
void SpecificWorker::add_or_assign_node_slot(const std::uint64_t id, const std::string &type)
{
    // PATH_TO_TARGET
    if (type == path_to_target_type_name)
    {
        if (auto node = G->get_node(id); node.has_value())
        {
            auto x_values = G->get_attrib_by_name<path_x_values_att>(node.value());
            auto y_values = G->get_attrib_by_name<path_y_values_att>(node.value());
            if (x_values.has_value() and y_values.has_value())
            {
                std::vector<QPointF> path;
                for (const auto &[x, y] : iter::zip(x_values.value().get(), y_values.value().get()))
                    path.emplace_back(QPointF(x, y));
                path_buffer.put(std::move(path));
                last_path_id = id;
            }
        }
    } else if (type == grid_type_name)  // grid
    {
        if (auto node = G->get_node(id); node.has_value())
        {
            if (auto grid_as_string = G->get_attrib_by_name<grid_as_string_att>(node.value()); grid_as_string.has_value())
                grid.readFromString(grid_as_string.value());
        }
    }
    else if (type == laser_type_name)    // Laser node updated
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
                                 [this](const LaserData &in, std::tuple<QPolygonF, std::vector<QPointF>> &out) {
                                     QPolygonF laser_poly;
                                     std::vector<QPointF> laser_cart;
                                     const auto &[angles, dists] = in;
                                     laser_cart.reserve(angles.size());
                                     auto robot = widget_2d->get_robot_polygon();
                                     for (const auto &[angle, dist] : iter::zip(angles, dists))
                                     {
                                         //convert laser polar coordinates to cartesian
                                         if (dist == 0) continue;
                                         float x = dist * sin(angle);
                                         float y = dist * cos(angle);
                                         // QPointF p = robot->mapToScene(x, y);
                                         Mat::Vector3d laserWorld = inner_eigen->transform(world_name, Mat::Vector3d(x, y, 0), robot_name).value();
                                         laser_poly << QPointF(x, y);
                                         QPointF p = robot->mapToScene(x, y);
                                         //laser_cart.emplace_back(QPointF(laserWorld.x(), laserWorld.y()));
                                         laser_cart.emplace_back(QPointF(p.x(), p.y()));
                                     }
                                     out = std::make_tuple(laser_poly, laser_cart);
                                 });
            }
        }
    }
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

    //clear previous points
    for (QGraphicsLineItem* item : scene_road_points)
    {
        viewer_2d->removeItem(item);
        delete item;
    }
    scene_road_points.clear();

    /// Draw all points
    QGraphicsLineItem *line1, *line2;
    QColor green_color("Green");
    QColor red_color("Red");
    QPen pen; pen.setWidth(20);
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

        if(is_visible(QPointF(b_point.x(), b_point.y()), laser_poly))
            pen.setColor(green_color);
        else
            pen.setColor(red_color);
        line1 = viewer_2d->addLine(qsegment, pen);
        line2 = viewer_2d->addLine(qsegment_perp, pen);

        line1->setZValue(2000);
        line2->setZValue(2000);
        scene_road_points.push_back(line1);
        scene_road_points.push_back(line2);
    }
    QColor fcolor("Orange");
    pen.setColor(fcolor);
    for(auto &&[orig, dest] : forces_vector)
    {
        //qInfo() << orig << dest;
        //dest = dest + (dest-orig) * 50;
        auto l = viewer_2d->addLine(QLineF(orig.toPointF(), dest.toPointF()), pen);
        l->setZValue(2000);
        scene_road_points.push_back(l);
    }
    //qInfo() << "----------";
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

/**************************************/