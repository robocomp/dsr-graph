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
    std::setlocale(LC_NUMERIC, "C"); // without this, decimal dots are ignored
	conf_params  = std::make_shared<RoboCompCommonBehavior::ParameterList>(params);
	agent_name = params["agent_name"].value;
	agent_id = stoi(params["agent_id"].value);
    tree_view = params["tree_view"].value == "true";
	graph_view = params["graph_view"].value == "true";
	qscene_2d_view = params["2d_view"].value == "true";
	osg_3d_view = params["3d_view"].value == "true";

    constants.number_of_not_visible_points = stoi(params.at("number_of_not_visible_points").value);
    constants.robot_length = stof(params["robot_length"].value);
    constants.robot_width = stof(params["robot_width"].value);
    constants.robot_radius = stof(params["robot_radius"].value);
    constants.KE = stof(params["external_forces_gain"].value);
    constants.KI = stof(params["internal_forces_gain"].value);
    constants.KS = stof(params["social_forces_gain"].value); //elastic band social
    constants.delta = stof(params["delta_derivation_step"].value);
    constants.max_laser_range = stof(params["max_laser_range"].value);
    constants.max_free_energy_iterations = stoi(params["max_free_energy_iterations"].value);
    constants.max_total_energy_ratio = stof(params["max_total_energy_ratio"].value);
    constants.update();  // to compute local relations

	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << __FUNCTION__ << std::endl;
	this->Period = Period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		// create graph
		G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id); // Init nodes
		std::cout<< __FUNCTION__ << " Graph loaded" << std::endl;

        //dsr update signals
        connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::add_or_assign_node_slot, Qt::QueuedConnection);
        //connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::add_or_assign_edge_slot);
        //connect(G.get(), &DSR::DSRGraph::update_attrs_signal, this, &SpecificWorker::add_or_assign_attrs_slot);
        //connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
        //connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);
        rt = G->get_rt_api();
        // Graph viewer
		using opts = DSR::DSRViewer::view;
		int current_opts = 0;
		opts main = opts::none;
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

        // self agent api
        agent_info_api = std::make_unique<DSR::AgentInfoAPI>(G.get());

        // Ignore attributes from G
        G->set_ignored_attributes<cam_rgb_att , cam_depth_att>();

        // Custom widget
        graph_viewer->add_custom_widget_to_dock("Elastic band", &custom_widget);
		widget_2d = qobject_cast<DSR::QScene2dViewer*> (graph_viewer->get_widget(opts::scene));

		if(widget_2d != nullptr)
            widget_2d->set_draw_laser(true);

        //widget_2d->set_draw_axis(bool draw);
        //connect(custom_widget.ke_slider, &QSlider::valueChanged, [this](auto v){ KE = v;});
        //connect(custom_widget.ki_slider, &QSlider::valueChanged, [this](auto v){ KI = v;});;

		// robot polygon
        robotBottomLeft = Mat::Vector3d(-    constants.robot_width / 2,     constants.robot_length / 2, 0);
        robotBottomRight = Mat::Vector3d(-    constants.robot_width / 2, -    constants.robot_length / 2, 0);
        robotTopRight = Mat::Vector3d(+    constants.robot_width / 2, -    constants.robot_length / 2, 0);
        robotTopLeft = Mat::Vector3d(+    constants.robot_width / 2, +    constants.robot_length / 2, 0);

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
        grid.TILE_SIZE = stoi(conf_params->at("tile_size").value);
        collisions =  std::make_shared<Collisions>();
        collisions->initialize(G, conf_params);
        grid.initialize(G, collisions, false);

        if( auto grid_node = G->get_node(current_grid_name); grid_node.has_value())
        {
            if (auto grid_as_string = G->get_attrib_by_name<grid_as_string_att>(grid_node.value()); grid_as_string.has_value())
                grid.readFromString(grid_as_string.value());
        }
        qInfo() << " SIZE " << grid.size();

		this->Period = 60;
        std::cout<< __FUNCTION__ << "Initialization finished" << std::endl;
        timer.start(Period);

        cout << "KE: " << constants.KE << endl;
        cout << "KI: " << constants.KI << endl;
        cout << "KS: " << constants.KS << endl;
	}
  //  hide();
}

void SpecificWorker::compute()
{
    static std::vector<QPointF> path;   // CHANGE TO LIST to reduce insertion times
    static std::tuple<QPolygonF, std::vector<QPointF>> laser_data;
    DSR::Node node;
    // Check for existing path_to_target_nodes
    if (auto path_o = path_buffer.try_get(); path_o.has_value())
    {

//        /**
//         * PARTE DE OPTIMIZACIÓN DE FUERZAS --- CREAR NODO
//         */º
        if (auto forces_opt = G->get_node("optimized_params"); forces_opt.has_value())
        {
            constants.KE = G->get_attrib_by_name<robot_external_forces_att>(forces_opt.value()).value();
            constants.KI = G->get_attrib_by_name<robot_internal_forces_att>(forces_opt.value()).value();
            constants.KS = G->get_attrib_by_name<robot_social_forces_att>(forces_opt.value()).value(); //elastic band social
        }

        /*cout << "KE: " << constants.KE << endl;
        cout << "KI: " << constants.KI << endl;
        cout << "KS: " << constants.KS << endl;*/

        path.clear();
        path = path_o.value();
        qDebug() << __FUNCTION__ << " New path detected";
    }
    else // continue operation on current path
    {
        if(auto node_path = G->get_node(current_path_name) ; node_path.has_value())
        {
            if( auto node_laser = G->get_node(laser_social_name); node_laser.has_value())
                node=node_laser.value();
            else
            {
                if( auto node_laser = G->get_node(laser_name); node_laser.has_value())
                if( auto node_laser = G->get_node(laser_name); node_laser.has_value())
                    node=node_laser.value();
            }
            if( const auto ldata = get_laser_data(node); ldata.has_value())
            //if( const auto ldata = laser_buffer.try_get(); ldata.has_value())
                {
                    std::get<1>(laser_data).clear(); std::get<0>(laser_data).clear();
                    laser_data = ldata.value();
                }
            const auto &[laser_poly, laser_cart] = laser_data;
            if(laser_poly.isEmpty() or laser_cart.empty()) return;

            auto nose_3d = inner_eigen->transform(world_name, Mat::Vector3d(0, 360, 0), robot_name).value();
            auto current_robot_nose = QPointF(nose_3d.x(), nose_3d.y());
            auto current_robot_polygon = get_robot_polygon();  //in world coordinates. Think of a transform_multi
//            std::cout << "1" << endl;
            compute_forces(path, laser_cart, laser_poly, current_robot_polygon, current_robot_nose);
            std::cout << "2" << endl;
            clean_points(path, laser_poly, current_robot_polygon);
            std::cout << "3" << endl;
            add_points(path, laser_poly, current_robot_polygon);
            std::cout << "4" << endl;
            if(widget_2d != nullptr) {
                std::cout << "5" << endl;
                draw_path(path, &widget_2d->scene, laser_poly);
            }
            std::cout << "6" << endl;
            save_path_in_G(path);
            std::cout << "7" << endl;
        }
        else
            qDebug() << __FUNCTION__ << "No path node";
    }
    fps.print("FPS: ", [this](auto x){ graph_viewer->set_external_hz(x);});
}
void SpecificWorker::compute_forces(std::vector<QPointF> &path,
                                    const vector<QPointF> &laser_cart,
                                    const QPolygonF &laser_poly,
                                    const QPolygonF &current_robot_polygon,
                                    const QPointF &current_robot_nose)
{
    if (path.size() < 2) return;;
    if (grid.size() == 0)
    {
        qWarning() << __FUNCTION__ << " Empty grid";
        return;
    };

    int nonVisiblePointsComputed = 0;
    forces_vector.clear();  // drawing only
    float total_energy_ratio = 0.f; // sum of gradients modules
    int iterations = 0;

    while( total_energy_ratio > constants.max_total_energy_ratio or iterations < constants.max_free_energy_iterations)
    {
        std::cout << "ITERATIONS: " << iterations << std::endl;
        std::cout << "TOTAL ENERGY RATIO: " << total_energy_ratio << std::endl;
        total_energy_ratio = 0.f;
        float total_energy = 0.f;
        // Go through all points using a sliding window of 3
        for (auto &&[i, group] : iter::enumerate(iter::sliding_window(path, 3)))
        {
            const auto &p1 = QVector2D(group[0]);
            const auto &p2 = QVector2D(group[1]);
            const auto &p3 = QVector2D(group[2]);
            if (p1 == p2 or p2 == p3) continue;

            QPointF p = group[1];
            int index_of_p_in_path = i + 1;  //index of p in path

            /////////////////////////////////////////////////////////////
            /// INTERNAL curvature forces on p2. Stretches the path locally
            /// Approximates the angle between adjacent segments: p2->p1, p2->p3
            ////////////////////////////////////////////////////////////
            QVector2D i_force = ((p1 - p2) / (p1 - p2).length() + (p3 - p2) / (p3 - p2).length());

            //////////////////////////////////////////////////////////
            /// External forces caused by obstacles repulsion field
            //////////////////////////////////////////////////////////
            QVector2D e_force;
            QVector2D laser_min_element;

            qDebug() << __FUNCTION__ << nonVisiblePointsComputed;
            /// Compute forces from G on not visible points using the grid (memory of free space)
            const float delta = grid.TILE_SIZE;
            float px, mx, py, my;
            if (not is_visible(p, laser_poly))
            {
                if (nonVisiblePointsComputed < constants.number_of_not_visible_points)
                {
                    if (auto closest_obstacle = grid.closest_obstacle(p); closest_obstacle.has_value())
                    {
                        float p_dist = QVector2D(p - closest_obstacle.value()).length();
                        px = mx = py = my = p_dist;  // all deltas initially equal to p distance to closest obstacle
                        if (auto closest_obstacle = grid.closest_obstacle(p + QPointF(delta, 0.f)); closest_obstacle.has_value())
                            px = QVector2D(p + QPointF(delta, 0.f) - closest_obstacle.value()).length();
                        if (auto closest_obstacle = grid.closest_obstacle(p + QPointF(-delta, 0.f)); closest_obstacle.has_value())
                            mx = QVector2D(p + QPointF(-delta, 0.f) - closest_obstacle.value()).length();
                        if (auto closest_obstacle = grid.closest_obstacle(p + QPointF(0.f, delta)); closest_obstacle.has_value())
                            py = QVector2D(p + QPointF(0.f, delta) - closest_obstacle.value()).length();
                        if (auto closest_obstacle = grid.closest_obstacle(p + QPointF(0.f, -delta)); closest_obstacle.has_value())
                            my = QVector2D(p + QPointF(0.f, -delta) - closest_obstacle.value()).length();

                        QVector2D grad((px - mx) / delta, (py - my) / delta);
                        //e_force = grad;
                        if( p_dist < constants.max_distance_range)
                            e_force = grad * (constants.max_distance_range - p_dist) * constants.KE;
                        else
                            e_force = QVector2D(0.f, 0.f);
                    } else
                    {
                        qDebug() << __FUNCTION__ << "No valid closest obstacle found in Grid";
                        nonVisiblePointsComputed++;
                        continue;
                    }
                    nonVisiblePointsComputed++;
                } else
                {
                    qDebug() << __FUNCTION__ << " Max number of not visible points processed";
                    //e_force = QVector2D(0.f, 0.f);
                    break;
                }
            }
                /// compute forces from laser on visible points
            else
            {
                // compute Jacobian of current point. as +- min_dist deltas in both axis for current point
                std::vector<float> pdx(laser_cart.size(), 0.f);
                std::vector<float> mdx(laser_cart.size(), 0.f);
                std::vector<float> pdy(laser_cart.size(), 0.f);
                std::vector<float> mdy(laser_cart.size(), 0.f);
                std::vector<float> center(laser_cart.size(), 0.f);

                for (size_t i = 0; const auto &&[lc, lr]: iter::zip(laser_cart, laser_poly))
                {   // compute distance from laser measure to delta points minus RLENGTH/2 or 0 and keep it positive

                    if (QVector2D(lr).length() >= constants.max_laser_range)  // if laser at MAX_RANGE, discard
                        pdx[i] = mdx[i] = pdy[i] = mdy[i] = center[i] = constants.max_distance_range + 1;
                    else
                    {
                        pdx[i] = QVector2D(p + QPointF(constants.delta, 0.f) - lc).length() - (constants.robot_radius);
                        mdx[i] = QVector2D(p + QPointF(-constants.delta, 0.f) - lc).length() - (constants.robot_radius);
                        pdy[i] = QVector2D(p + QPointF(0.f, constants.delta) - lc).length() - (constants.robot_radius);
                        mdy[i] = QVector2D(p + QPointF(0.f, -constants.delta) - lc).length() - (constants.robot_radius);
                        center[i] = QVector2D(p - lc).length() - (constants.robot_radius);
                    }
                    i++;
                };
                // compute min of the four vectors of distances
                auto px_min = std::ranges::min(pdx);
                auto mx_min = std::ranges::min(mdx);
                auto py_min = std::ranges::min(pdy);
                auto my_min = std::ranges::min(mdy);
                auto center_min = std::ranges::min(center);

                // compute gradient as [ pD / dx, pD / dy ]
                QVector2D grad((px_min - mx_min) / delta, (py_min - my_min) / delta);
                if( center_min < constants.max_distance_range)
                    e_force = grad * (constants.max_distance_range - center_min) * constants.KE;
                else
                    e_force = QVector2D(0.f, 0.f);
                //qInfo() << px_min << mx_min << py_min << my_min << center_min;
            }

            // Remove tangential component of repulsion force by projecting on line tangent to path (base_line)
            QVector2D base_line = (p1 - p3).normalized();
            const QVector2D itangential = QVector2D::dotProduct(e_force, base_line) * base_line;
            e_force = e_force - itangential;

            ///////////////////////////////////////
            /// SOCIAL FORCES ///

            QVector2D s_force_total;


            // people exists?
            if(auto person_list = G->get_nodes_by_type(person_type_name); not person_list.empty()) {
                if (auto space = G->get_nodes_by_type(personal_space_type_name); not space.empty()) {

                    s_force_total = calculateSocialForce(s_force_total, person_list, p);
                    // Remove tangential component of repulsion force by projecting on line tangent to path (base_line)
                    QVector2D base_line_social = (p1 - p3).normalized();
                    const QVector2D itangential_social =
                            QVector2D::dotProduct(s_force_total, base_line_social) * base_line_social;
                    s_force_total = s_force_total - itangential_social;
                }
            }


            ///////////////////////////////////////

            QVector2D total = (constants.KI * i_force) + (e_force) + (constants.KS * s_force_total);

//            std::cout << " I_FORCE: " << (constants.KI * i_force).length() << " E_FORCE: " << e_force.length() << " S_FORCE: " << (constants.KS * s_force_total).length() << std::endl;
            std::cout << " TOTAL FORCE: " << total.length() << std::endl;
            //std::cout<<"KE: "<< constants.KE << endl;
            //std::cout<<"KS: "<< constants.KS << endl;
            total_energy += total.length();

            // for drawing
            if( total.length() > 0)
                try
                {
                    auto p_draw = QVector2D(path.at(index_of_p_in_path));
                    forces_vector.push_back(std::make_tuple(p_draw, (total + p_draw)));
                }
                catch (const std::exception &e)
                { std::cout << e.what() << std::endl; };

            /// Compute additional restrictions to be forced in the minimization process
            QPointF temp_p = p + total.toPointF();
            // Move nodes only if it does not move inside objects and Does not move underneath the robot.
            if (grid.isFree(grid.pointToGrid(temp_p)) and (not current_robot_polygon.containsPoint(temp_p, Qt::OddEvenFill)))
                try
                { path.at(index_of_p_in_path) = temp_p; }
                catch (const std::exception &e)
                { std::cout << e.what() << std::endl; };
        }
        total_energy_ratio = total_energy / path.size();
        //qInfo() << __FUNCTION__ << " Remaining energy: " << total_energy << " Length: " << path.size() << " Ratio: " << total_energy / path.size();
        iterations++;
    }
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
        qDebug() << __FUNCTION__ << " dist <" << dist << 0.5 * constants.road_step_separation;
        if (dist < 0.5 * constants.road_step_separation)
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
        qDebug() << __FUNCTION__ << " dist >" << dist << constants.road_step_separation;
        if (dist > constants.road_step_separation)
        {
            //Crucial que el punto se ponga mas cerca que la condición de entrada
            float l = 0.9 * constants.road_step_separation / dist;
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
    try{
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
    catch (const std::exception &e) {
        std::cout << e.what() << std::endl;
    }
}
//////////////////////////////////////////////////////////////////////////////////////////
std::optional<std::tuple<QPolygonF, std::vector<QPointF>>> SpecificWorker::get_laser_data(const DSR::Node &node)
{
        auto angles = G->get_attrib_by_name<laser_angles_att>(node);
        auto dists = G->get_attrib_by_name<laser_dists_att>(node);
        if (dists.has_value() and angles.has_value())
        {
            const auto &d = dists.value().get();
            const auto &a = angles.value().get();
            if (d.empty() or a.empty()) return {};
            QPolygonF laser_poly;
            laser_poly << QPointF(0, 100);
            std::vector<QPointF> laser_cart;
            laser_cart.reserve(a.size());
            //auto robot = widget_2d->get_robot_polygon();
            //auto inner_eigen = G->get_inner_eigen_api();
            for (const auto &[angle, dist] : iter::zip(a, d))
            {
                //convert laser polar coordinates to cartesian
                if (dist == 0) continue;
                float x = dist * sin(angle);
                float y = dist * cos(angle);
                Mat::Vector3d laserWorld = inner_eigen->transform(world_name,Mat::Vector3d(x, y, 0), robot_name).value();
                laser_poly << QPointF(x, y);
                laser_cart.emplace_back(QPointF(laserWorld.x(), laserWorld.y()));
            }
            laser_poly << QPointF(0, 100);
            return std::make_tuple(laser_poly, laser_cart);
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
            {
                std::cout << "ACT GRID" << std::endl;

                grid.readFromString(grid_as_string.value());
                std::cout << "ACT GRID FINISH" << std::endl;
            }

        }
    }
//    else if (type == laser_type_name)    // Laser node updated
//    {
//        if (auto node = G->get_node(id); node.has_value())
//        {
//            auto angles = G->get_attrib_by_name<laser_angles_att>(node.value());
//            auto dists = G->get_attrib_by_name<laser_dists_att>(node.value());
//            if (dists.has_value() and angles.has_value())
//            {
//                const auto &d = dists.value().get();
//                const auto &a = angles.value().get();
//                if (d.empty() or a.empty()) return;
//                laser_buffer.put(std::make_tuple(a, d),
//                                 [this](const LaserData &in, std::tuple<QPolygonF, std::vector<QPointF>> &out) {
//                                     QPolygonF laser_poly;
//                                     std::vector<QPointF> laser_cart;
//                                     const auto &[angles, dists] = in;
//                                     laser_cart.reserve(angles.size());
//                                     auto inner = G->get_inner_eigen_api();
//                                     for (const auto &[angle, dist] : iter::zip(angles, dists))
//                                     {
//                                         //convert laser polar coordinates to cartesian
//                                         if (dist == 0) continue;
//                                         float x = dist * sin(angle);
//                                         float y = dist * cos(angle);
//                                         // QPointF p = robot->mapToScene(x, y);
//                                         Mat::Vector3d laserWorld = inner->transform(world_name, Mat::Vector3d(x, y, 0), robot_name).value();
//                                         laser_poly << QPointF(x, y);
//                                         //QPointF p = robot->mapToScene(x, y);
//                                         laser_cart.emplace_back(QPointF(laserWorld.x(), laserWorld.y()));
//                                         //laser_cart.emplace_back(QPointF(p.x(), p.y()));
//                                     }
//                                     out = std::make_tuple(laser_poly, laser_cart);
//                                 });
//            }
//        }
//    }
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

QPointF SpecificWorker:: obtener_gaussianas (const QPointF robot_point)
{
    QVector<QLineF> lines;
    QPointF intersectmin;
    if(auto personal_space = G->get_nodes_by_type(personal_space_type_name); not personal_space.empty()) {
        for( const auto &p_space : personal_space)
        {
            const auto gauss_x_o = G->get_attrib_by_name<ps_intimate_x_pos_att>(p_space);
            const auto gauss_y_o = G->get_attrib_by_name<ps_intimate_y_pos_att>(p_space);
            if( gauss_x_o.has_value() and gauss_y_o.has_value())
            {
                const std::vector<float> &gauss_x = gauss_x_o.value().get();
                const std::vector<float> &gauss_y = gauss_y_o.value().get();
                for(auto &&pos : iter::sliding_window(iter::zip(gauss_x, gauss_y),2))
                {
                    const auto &[x1, y1] = pos[0];
                    const auto &[x2, y2] = pos[1];
                    lines.append(QLineF(QPointF(x1, y1), QPointF(x2, y2)));
                }
                lines.append(QLineF(QPointF(gauss_x.front(), gauss_y.front()), QPointF(gauss_x.back(), gauss_y.back())));
            }
            else  qWarning() << __FUNCTION__ << " No attributes gauss_x or gauss_y available in G";
        }
        //const auto robot_in_w = inner_eigen->transform(world_name, robot_name);
        QPointF intersectPnt;
        //QPointF intersectmin;
        //QPointF robot_point(robot_in_w.value().x(),robot_in_w.value().y());
        float dist;
        float distance;
        distancePointLine(robot_point,lines[0],distance,intersectPnt);
        for(auto &&line: lines)
        {
            distancePointLine(robot_point,line,dist,intersectPnt);
            if (dist<=distance)
            {
                distance=dist;
                intersectmin=intersectPnt;
            }


        }

    }
    return intersectmin;
}

void SpecificWorker::distancePointLine( const QPointF &p, const QLineF &line, float &distance, QPointF &intersectPnt)
{
    float lineMag;
    float u;
    lineMag=line.length();
    QPointF l1 = line.p1() ;
    QPointF l2 = line.p2() ;
    u = ( ( ( p.x() - l1.x() ) * ( l2.x() - l1.x() ) ) +
            ( ( p.y() - l1.y() ) * ( l2.y() - l1.y() ) ) ) /
                    ( lineMag * lineMag );
    intersectPnt.setX( int( l1.x() + u * ( l2.x() - l1.x() ) ) );
    intersectPnt.setY( int( l1.y() + u * ( l2.y() - l1.y() ) ) );
    distance = QVector2D(p-intersectPnt).length();
}

QVector2D SpecificWorker::calculateSocialForce(QVector2D s_force_total, vector<Node> person_list, QPointF p)
{
    auto distance = 99999;
    int closest_person_id;
    Eigen::Matrix<double, 3, 1> person_pose;
    bool close_people = false;
    //Find closest person to p
    for( const auto &person : person_list){
        if(auto followed_person = G->get_attrib_by_name<is_followed_att>(person); followed_person.has_value() && followed_person.value() == false)
        {
            if(auto edge_robot_person = rt->get_edge_RT(G->get_node("world").value(), person.id()); edge_robot_person.has_value())
            {
                auto pose_edge = G->get_attrib_by_name<rt_translation_att>(edge_robot_person.value()).value().get();
                Eigen::Matrix<double, 3, 1> pose_aux(pose_edge[0], pose_edge[1], 0);
//        auto pose_aux = inner_eigen->transform(world_name, person.name()).value();
                Eigen::Matrix<double, 3, 1> point (p.x(),p.y(),0);
                if ( (point - pose_aux).norm() < distance && (point - pose_aux).norm() < 1500) {
                    close_people = true;
                    distance = (point - pose_aux).norm();// Distance point - person
                    closest_person_id = G->get_attrib_by_name<person_id_att>(person).value();
                    person_pose = pose_aux;
                }
            }
        }
    }

    if (close_people){

        size_t mod = 50; // milímetros
        //A vector s_force for person
        QVector2D s_force((p.x() - person_pose.x()), (p.y() - person_pose.y()));
        s_force = s_force/distance; //normalize s_force with the distance

        for( const auto &person : person_list)
        {
            //Get Personal spaces
            if(auto polylines_list = G->get_nodes_by_type(personal_space_type_name); not polylines_list.empty()) {
                for( const auto &polyline : polylines_list)
                {
                    // Get parent's personid
                    const auto padre = G->get_attrib_by_name<person_id_att>(polyline);

                    if(padre == closest_person_id && distance < 1500){
                        //VECTOR GAUSIANAS, SOCIAL ZONE
                        QPolygonF social; //For each gaussian

                        // Get social zone polygon
                        const auto social_x_o = G->get_attrib_by_name<ps_social_x_pos_att>(polyline);
                        const auto social_y_o = G->get_attrib_by_name<ps_social_y_pos_att>(polyline);
                        if( social_x_o.has_value() and social_y_o.has_value())
                        {
                            const std::vector<float> &social_x = social_x_o.value().get();
                            const std::vector<float> &social_y = social_y_o.value().get();
                            for(auto &&[point_x,point_y] : iter::zip(social_x, social_y))
                            {
                                QPointF point(point_x,point_y);
                                social.append(point);
                            }
                        }
                        // if p is located inside the social zone, the social force will be bigger
                        if (social.containsPoint(p, Qt::OddEvenFill)) {
                            mod *= 2;

//                            qInfo() << "INVASION ZONA SOCIAL" ;
                            QPolygonF personal;

                            const auto personal_x_o = G->get_attrib_by_name<ps_personal_x_pos_att>(polyline);
                            const auto personal_y_o = G->get_attrib_by_name<ps_personal_y_pos_att>(polyline);
                            if( personal_x_o.has_value() and personal_y_o.has_value())
                            {
                                const std::vector<float> &personal_x = personal_x_o.value().get();
                                const std::vector<float> &personal_y = personal_y_o.value().get();
                                for(auto &&[point_x,point_y] : iter::zip(personal_x, personal_y))
                                {
                                    QPointF point(point_x,point_y);
                                    personal.append(point);
                                }
                            }

                            if (personal.containsPoint(p, Qt::OddEvenFill)) { //Zona intima
                                mod *= 2;

//                                qInfo() << "INVASION ZONA INTIMA" ;

                                //VECTOR GAUSIANAS, SOCIAL ZONE
                                QPolygonF intimate; //For each gaussian

                                // Get social zone polygon
                                const auto intimate_x_o = G->get_attrib_by_name<ps_intimate_x_pos_att>(polyline);
                                const auto intimate_y_o = G->get_attrib_by_name<ps_intimate_y_pos_att>(polyline);
                                if( intimate_x_o.has_value() and intimate_y_o.has_value())
                                {

                                    const std::vector<float> &intimate_x = intimate_x_o.value().get();
                                    const std::vector<float> &intimate_y = intimate_y_o.value().get();
                                    for(auto &&[point_x,point_y] : iter::zip(intimate_x, intimate_y))
                                    {
                                        QPointF point(point_x,point_y);
                                        intimate.append(point);
                                    }
                                }
                                // if p is located inside the social zone, the social force will be bigger
                                if (intimate.containsPoint(p, Qt::OddEvenFill)) {
                                    mod *= 2;
                                }
                            }
                        }
                    }
                }
            }
        }
        //s_force_total acumula los vectores (s_force) si hay varias gausianas (personas)
//        mod = (1200-distance);
        return s_force * mod;
//        s_force = s_force * mod;
//        s_force_total = s_force_total + s_force;
//
//        qInfo() << "s_force" << s_force;
//        //}
//        qInfo() << " s_force_total " << s_force_total;
//        return s_force_total;
    }
    else
    {
//        qInfo() << " NO CLOSE PEOPLE " << s_force_total;
        return QVector2D (0.0 , 0.0);
    }


//    for( const auto &person : person_list)
//    {
//        auto person_pose = inner_eigen->transform(world_name, person.name()).value();
//        Eigen::Matrix<double, 3, 1> point (p.x(),p.y(),0);
//
//        auto distance = (point - person_pose).norm();// Distance point - person
//
//        //if(distance <= 3600.0)
//        //{
//
//            // get personid
//            const auto personid = G->get_attrib_by_name<person_id_att>(person);
//            //get person pose
//            //auto person_pose = inner_eigen->transform(world_name, person.name()).value();
//
//            size_t mod = 1;
//
//            //A vector s_force for person
//            QVector2D s_force((p.x() - person_pose.x()), (p.y() - person_pose.y()));
//            s_force = s_force/distance; //normalize s_force with the distance
//            //Get Personal spaces
//            if(auto polylines_list = G->get_nodes_by_type(personal_space_type_name); not polylines_list.empty()) {
//                for( const auto &polyline : polylines_list)
//                {
//                    // Get parent's personid
//                    const auto padre = G->get_attrib_by_name<person_id_att>(polyline);
//
//                    if(padre == personid){
//
//                        //VECTOR GAUSIANAS, SOCIAL ZONE
//                        QPolygonF social; //For each gaussian
//
//                        // Get social zone polygon
//                        const auto social_x_o = G->get_attrib_by_name<ps_social_x_pos_att>(polyline);
//                        const auto social_y_o = G->get_attrib_by_name<ps_social_y_pos_att>(polyline);
//                        if( social_x_o.has_value() and social_y_o.has_value())
//                        {
//                            const std::vector<float> &social_x = social_x_o.value().get();
//                            const std::vector<float> &social_y = social_y_o.value().get();
//                            for(auto &&[point_x,point_y] : iter::zip(social_x, social_y))
//                            {
//                                QPointF point(point_x,point_y);
//                                social.append(point);
//                            }
//                        }
//                        // if p is located inside the social zone, the social force will be bigger
//                        if (social.containsPoint(p, Qt::OddEvenFill)) {
//                            mod *= 2;
//
//                            qInfo() << "INVASION ZONA SOCIAL" ;
//                            QPolygonF personal;
//
//                            const auto personal_x_o = G->get_attrib_by_name<ps_personal_x_pos_att>(polyline);
//                            const auto personal_y_o = G->get_attrib_by_name<ps_personal_y_pos_att>(polyline);
//                            if( personal_x_o.has_value() and personal_y_o.has_value())
//                            {
//                                const std::vector<float> &personal_x = personal_x_o.value().get();
//                                const std::vector<float> &personal_y = personal_y_o.value().get();
//                                for(auto &&[point_x,point_y] : iter::zip(personal_x, personal_y))
//                                {
//                                    QPointF point(point_x,point_y);
//                                    personal.append(point);
//                                }
//                            }
//
//                            if (personal.containsPoint(p, Qt::OddEvenFill)) { //Zona intima
//                                mod *= 2;
//
//                                qInfo() << "INVASION ZONA INTIMA" ;
//
//                                //VECTOR GAUSIANAS, SOCIAL ZONE
//                                QPolygonF intimate; //For each gaussian
//
//                                // Get social zone polygon
//                                const auto intimate_x_o = G->get_attrib_by_name<ps_intimate_x_pos_att>(polyline);
//                                const auto intimate_y_o = G->get_attrib_by_name<ps_intimate_y_pos_att>(polyline);
//                                if( intimate_x_o.has_value() and intimate_y_o.has_value())
//                                {
//
//                                    const std::vector<float> &intimate_x = intimate_x_o.value().get();
//                                    const std::vector<float> &intimate_y = intimate_y_o.value().get();
//                                    for(auto &&[point_x,point_y] : iter::zip(intimate_x, intimate_y))
//                                    {
//                                        QPointF point(point_x,point_y);
//                                        intimate.append(point);
//                                    }
//                                }
//                                // if p is located inside the social zone, the social force will be bigger
//                                if (intimate.containsPoint(p, Qt::OddEvenFill)) {
//                                    mod *= 2;
//                                }
//                            }
//                        }
//                    }
//                }
//            }
//            //s_force_total acumula los vectores (s_force) si hay varias gausianas (personas)
//            s_force = s_force * mod;
//            s_force_total = s_force_total + s_force;
//
//        qInfo() << "s_force" << s_force;
//        //}
//
//
//    }


}
