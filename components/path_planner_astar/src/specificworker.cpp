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
#include <algorithm>
#include <ranges>

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
	grid_file_name = params["grid_file_name"].value;
	read_from_file = params["read_from_file"].value == "true";
	num_threads_for_grid_occupancy = stoi(params["num_threads_for_grid_occupancy"].value);
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
        //connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::add_or_assign_edge_slot);
        //connect(G.get(), &DSR::DSRGraph::update_attrs_signal, this, &SpecificWorker::add_or_assign_attrs_slot);
        //connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
        //connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);

        // Graph viewer
        using opts = DSR::DSRViewer::view;
        int current_opts = 0;
        opts main = opts::none;
        if (tree_view)
            current_opts = current_opts | opts::tree;
        if (graph_view)
            current_opts = current_opts | opts::graph;
        if (qscene_2d_view)
            current_opts = current_opts | opts::scene;
        if (osg_3d_view)
            current_opts = current_opts | opts::osg;

        graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
        setWindowTitle(QString::fromStdString(agent_name + "-") + QString::number(agent_id));

        //Inner Api
        inner_eigen = G->get_inner_eigen_api();

        // Ignore attributes from G
        G->set_ignored_attributes<cam_rgb_att, cam_depth_att>();

        // 2D widget
        widget_2d = qobject_cast<DSR::QScene2dViewer *>(graph_viewer->get_widget(opts::scene));
        widget_2d->set_draw_laser(false);
        connect(widget_2d, SIGNAL(mouse_right_click(int, int, std::uint64_t)), this, SLOT(new_target_from_mouse(int, int, std::uint64_t)));

        // path planner
        path_planner_initialize(&widget_2d->scene, read_from_file, grid_file_name);
        qInfo() << __FUNCTION__ << "Grid created with size " << grid.size() * sizeof(Grid::T) << "bytes";
        inject_grid_in_G(grid);
        qInfo() << __FUNCTION__ << "Grid injected in G";

        // check for existing intention node
        if (auto intentions = G->get_nodes_by_type(intention_type_name); not intentions.empty())
            this->add_or_assign_node_slot(intentions.front().id(), "intention");

        this->Period = 200;
        std::cout << __FUNCTION__ << " Initialization complete. Starting 'compute' at " << 1/(this->Period/1000.f) << " Hz"<< std::endl;
        timer.start(Period);
    }
}

void SpecificWorker::compute()
{
    // Update grid
    // Check for new plan
    if (auto plan_o = plan_buffer.try_get(); plan_o.has_value())
    {
        current_plan = plan_o.value();
        current_plan.print();
        if( auto robot_o = G->get_node(robot_name); robot_o.has_value())
        {
           auto robot = robot_o.value();
            if( auto intention_nodes = G->get_nodes_by_type(intention_type_name); not intention_nodes.empty())
            {
                auto intention_node = intention_nodes.front();
                if (auto target_o = G->get_node(current_plan.target_place); target_o.has_value())
                {
                    auto target = target_o.value();
                    const auto &[search_state, candidate] = search_a_feasible_target(target, current_plan.params, robot);
                    std::list<QPointF> path;
                    QPointF currentRobotNose;
                    Mat::Vector3d nose_3d;
                    switch (search_state)
                    {
                        case SearchState::NEW_TARGET:
                            break;
                        case SearchState::NEW_FLOOR_TARGET:
                            std::cout << __FUNCTION__ << " Candidate found on floor: " << std::endl;
                            nose_3d = inner_eigen->transform(world_name, Mat::Vector3d(0, 380, 0), robot_name).value();
                            currentRobotNose = QPointF(nose_3d.x(), nose_3d.y());
                            path = grid.computePath(currentRobotNose, QPointF(candidate.x(), candidate.y()));
                            qInfo() << __FUNCTION__ << " Path size: " << path.size();
                            if (not path.empty())
                            {
                                draw_path(path, &widget_2d->scene);

                                std::vector<float> x_values;
                                x_values.reserve(path.size());
                                std::transform(path.cbegin(), path.cend(), std::back_inserter(x_values),
                                               [](const auto &value) { return value.x(); });
                                std::vector<float> y_values;
                                y_values.reserve(path.size());
                                std::transform(path.cbegin(), path.cend(), std::back_inserter(y_values),
                                               [](const auto &value) { return value.y(); });
                                if (auto node_paths = G->get_nodes_by_type(path_to_target_type_name); not node_paths.empty())
                                {
                                    auto path_to_target_node = node_paths.front();
                                    G->add_or_modify_attrib_local<path_x_values_att>(path_to_target_node, x_values);
                                    G->add_or_modify_attrib_local<path_y_values_att>(path_to_target_node, y_values);
                                    G->add_or_modify_attrib_local<path_target_x_att>(path_to_target_node,
                                                                                     (float) candidate.x());
                                    G->add_or_modify_attrib_local<path_target_y_att>(path_to_target_node,
                                                                                     (float) candidate.y());
                                    G->update_node(path_to_target_node);
                                } else // create path_to_target_node with the solution path
                                {
                                    auto path_to_target_node = DSR::Node::create<path_to_target_node_type>(current_intention_name);
                                    G->add_or_modify_attrib_local<path_x_values_att>(path_to_target_node, x_values);
                                    G->add_or_modify_attrib_local<path_y_values_att>(path_to_target_node, y_values);
                                    G->add_or_modify_attrib_local<pos_x_att>(path_to_target_node, (float) -150);
                                    G->add_or_modify_attrib_local<pos_y_att>(path_to_target_node, (float) -400);
                                    G->add_or_modify_attrib_local<parent_att>(path_to_target_node, intention_node.id());
                                    G->add_or_modify_attrib_local<level_att>(path_to_target_node, 3);
                                    G->add_or_modify_attrib_local<level_att>(path_to_target_node, 3);
                                    G->add_or_modify_attrib_local<path_target_x_att>(path_to_target_node, (float) candidate.x());
                                    G->add_or_modify_attrib_local<path_target_y_att>(path_to_target_node, (float) candidate.y());

                                    auto id = G->insert_node(path_to_target_node);
                                    auto edge_to_intention = Edge(id.value(), intention_node.id(), think_type,
                                                                  agent_id);
                                    G->insert_or_assign_edge(edge_to_intention);
                                }
                           }
                            break;
                        case SearchState::AT_TARGET:
                            std::cout << __FUNCTION__ << " At target " << std::endl;
                            break;
                        case SearchState::NO_TARGET_FOUND:
                            std::cout << __FUNCTION__ << " No target found: " << std::endl;
                            break;
                    }
                }
            }
        }
        else // do whatever you do without a plan
        {}
    }
}

void SpecificWorker::path_planner_initialize(QGraphicsScene *scene, bool read_from_file, const std::string file_name)
{
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
    // if read_from_file is true we should read the parameters from the file to guarantee consistency
    grid.dim.setCoords(outerRegion.left(), outerRegion.top(), outerRegion.right(), outerRegion.bottom());

    std::cout << __FUNCTION__ << " - TileSize is " << conf_params->at("TileSize").value << std::endl;
    qInfo() <<  __FUNCTION__  << grid.dim.left()  << grid.dim.bottom() << grid.dim.top() << grid.dim.right() << grid.dim.width() << grid.dim.height();
    grid.TILE_SIZE = stoi(conf_params->at("TileSize").value);

    collisions =  std::make_shared<Collisions>();
    collisions->initialize(G, conf_params);

    grid.initialize(G, collisions, read_from_file, file_name, num_threads_for_grid_occupancy);
    grid.draw(&widget_2d->scene);

    robotXWidth = std::stof(conf_params->at("RobotXWidth").value);
    robotZLong = std::stof(conf_params->at("RobotZLong").value);
    robotBottomLeft     = Mat::Vector3d ( -robotXWidth / 2, robotZLong / 2, 0);
    robotBottomRight    = Mat::Vector3d ( - robotXWidth / 2,- robotZLong / 2, 0);
    robotTopRight       = Mat::Vector3d ( + robotXWidth / 2, - robotZLong / 2, 0);
    robotTopLeft        = Mat::Vector3d ( + robotXWidth / 2, + robotZLong / 2, 0);
}

//
// Search
// sample radially from target floor projection and create path to all acceptable places
// merge paths into a tree and wait to see if the decision can be visually solved at the knot
// if not select one. One option is to analyze the local orientation of the barrier
//

std::tuple<SpecificWorker::SearchState, Mat::Vector2d> SpecificWorker::search_a_feasible_target(const Node &target, const std::map<std::string, double> &params, const Node &robot)
{
    Mat::Vector3d tc = inner_eigen->transform(world_name, target.name()).value();
    Mat::Vector3d rc = inner_eigen->transform(world_name, robot_name).value();
    std::cout << __FUNCTION__ << " Target id: " << target.id() << " located at: " << tc.x() << ", " << tc.y() << " and coor: " << params.at("x") << ", " << params.at("y") << std::endl;
    std::cout << __FUNCTION__ << " Robot located at: " << rc.x() << ", " << rc.y() << std::endl;

    // if x,y not empty
    if(not params.empty())
    {
        // if already in current_target return
        if (this->current_plan.is_location(Mat::Vector2d(rc.x(), rc.y())))
            return std::make_tuple(SearchState::AT_TARGET, Mat::Vector2d());
        // if node is floor_plane take coordinates directly
        if (target.name() == floor_name) // floor
            return std::make_tuple(SearchState::NEW_FLOOR_TARGET, Mat::Vector2d(params.at("x"), params.at("y")));
    }
    std::string target_name = target.name();
    Mat::Vector2d target_center(tc.x(), tc.y());
    Mat::Vector2d robot_center(rc.x(), rc.y());
    std::vector<Mat::Vector2d> candidates;

    // search in a spiral pattern away from object for the first free cell
    long int x_pos = target_center.x();
    long int y_pos = target_center.y();
    int d = grid.TILE_SIZE;
    for(int i : iter::range(grid.size()))
    {
        while (2 * x_pos * d < i)
        {
            const auto &k = Grid::Key(x_pos, y_pos);
            if (grid.isFree(k))
                candidates.emplace_back(Mat::Vector2d (x_pos, y_pos));
            x_pos = x_pos + d;
            if(candidates.size() > 0)   // arbitrary number hard to fix
                break;
        }
        while (2 * y_pos * d < i)
        {
            const auto &k = Grid::Key(x_pos, y_pos);
            if (grid.isFree(k))
                candidates.emplace_back(Mat::Vector2d(x_pos, y_pos));
            y_pos = y_pos + d;
            if(candidates.size() > 0)   // arbitrary number hard to fix
                break;
        }
        d = -1 * d;
    }

    // sort by distances to target
    if(candidates.size() > 0)
    {
        std::sort(std::begin(candidates), std::end(candidates), [robot_center](const auto &p1, const auto &p2) {
            return (robot_center - p1).norm() < (robot_center - p2).norm();
        });
        return std::make_tuple(SearchState::NEW_TARGET, candidates.front());
    }
    else
        return std::make_tuple(SearchState::NO_TARGET_FOUND, Mat::Vector2d());
}

void SpecificWorker::inject_grid_in_G(const Grid &grid)
{
  std::string grid_as_string = grid.saveToString();
  if (auto current_grid_node_o = G->get_node(current_grid_name); current_grid_node_o.has_value())
    G->add_or_modify_attrib_local<grid_as_string_att>(current_grid_node_o.value(), grid_as_string);
  else
  {
      if (auto robot = G->get_node(robot_name); robot.has_value())
      {
          DSR::Node current_grid_node = DSR::Node::create<grid_node_type>(current_grid_name);
          G->add_or_modify_attrib_local<name_att>(current_grid_node, current_grid_name);
          G->add_or_modify_attrib_local<parent_att>(current_grid_node, robot.value().id());
          G->add_or_modify_attrib_local<grid_as_string_att>(current_grid_node, grid_as_string);
          G->add_or_modify_attrib_local<pos_x_att>(current_grid_node, (float) -70);
          G->add_or_modify_attrib_local<pos_y_att>(current_grid_node, (float) -364);
          if (std::optional<int> current_grid_node_id = G->insert_node(current_grid_node); current_grid_node_id.has_value())
          {
              if (G->insert_or_assign_edge(Edge(robot.value().id(), current_grid_node.id(), has_type, agent_id)))
                  std::cout << __FUNCTION__ << "Edge successfully created between robot and grid" << std::endl;
              else
              {
                  std::cout << __FILE__ << __FUNCTION__ << " Fatal error inserting new edge: " << robot.value().id() << "->" << current_grid_node_id.value()
                            << " type: has" << std::endl;
                  std::terminate();
              }
          } else
          {
              std::cout << __FILE__ << __FUNCTION__ << " Fatal error inserting_new 'grid' node" << std::endl;
              std::terminate();
          }
      }
  }
}
///////////////////////////////////////////////////////
//// Check new target from mouse
///////////////////////////////////////////////////////

void SpecificWorker::new_target_from_mouse(int pos_x, int pos_y, std::uint64_t id)
{
    qInfo() << __FUNCTION__ << pos_x, pos_y;
    // Check if there is not 'intention' node yet in G
    if (auto intention_nodes = G->get_nodes_by_type(intention_type_name); intention_nodes.empty())
    {
        if(auto robot = G->get_node(robot_name); robot.has_value())
        {
            DSR::Node intention_node = DSR::Node::create<intention_node_type>(current_intention_name);
            G->add_or_modify_attrib_local<parent_att>(intention_node,  robot.value().id());
            G->add_or_modify_attrib_local<level_att>(intention_node, G->get_node_level(robot.value()).value() + 1);
            G->add_or_modify_attrib_local<pos_x_att>(intention_node, (float)-90);
            G->add_or_modify_attrib_local<pos_y_att>(intention_node, (float)-354);
            try
            {
                if(std::optional<int> intention_node_id = G->insert_node(intention_node); intention_node_id.has_value())
                {
                    if(G->insert_or_assign_edge(Edge(robot.value().id(), intention_node.id(), has_type, agent_id)))
                    {}
                    else
                    {
                        std::cout << __FILE__ << __FUNCTION__ << " Fatal error inserting new edge: " << robot.value().id() << "->" << intention_node_id.value() << " type: has" << std::endl;
                        std::terminate();
                    }
                }
                else
                {
                    std::cout << __FILE__ << __FUNCTION__ << " Fatal error inserting_new 'intention' node" << std::endl;
                    std::terminate();
                }
            }
            catch(...)
            { std::cout << "BYE" << std::endl; std::terminate();}
        }
        else
        {
            std::cout << __FILE__ << __FUNCTION__ << " No node " << robot_name << " found in G" << std::endl;
            std::terminate();
        }
    }
    using namespace std::placeholders;
    if (auto target_node = G->get_node(id); target_node.has_value())
    {
        const std::string location =
                "[" + std::to_string(pos_x) + "," + std::to_string(pos_y) + "," + std::to_string(0) + "]";
        const std::string plan =
                "{\"plan\":[{\"action\":\"goto\",\"params\":{\"location\":" + location + ",\"object\":\"" +
                target_node.value().name() + "\"}}]}";
        std::cout << plan << std::endl;
        plan_buffer.put(plan, std::bind(&SpecificWorker::json_to_plan, this, _1, _2));
    } else
        qWarning() << __FILE__ << __FUNCTION__ << " No target node  " << QString::number(id) << " found";
}

///////////////////////////////////////////////////
/// Asynchronous changes on G nodes from G signals
///////////////////////////////////////////////////
void SpecificWorker::add_or_assign_node_slot(const std::uint64_t id, const std::string &type)
{
    // check node type
    using namespace std::placeholders;
    if (type == intention_type_name)
    {
        qInfo() << __FUNCTION__ ;
        auto node = G->get_node(id);
        if (auto parent = G->get_parent_node(node.value()); parent.has_value() and parent.value().name() == robot_name)
        {
            std::optional<std::string> plan = G->get_attrib_by_name<current_intention_att >(node.value());
            //std::optional<std::string> plan = G->get_attrib_by_name<plan_att>(node.value());
            if (plan.has_value())
                  plan_buffer.put(plan.value(), std::bind(&SpecificWorker::json_to_plan, this,_1, _2));
        }
    }
}

//////////////////////////////////////////////7
/// parser form JSON plan to Plan structure
void SpecificWorker::json_to_plan(const std::string &plan_string, Plan &plan)
{
    QJsonDocument doc = QJsonDocument::fromJson(QString::fromStdString(plan_string).toUtf8());
    QJsonObject planJson = doc.object();
    QJsonArray actionArray = planJson.value("plan").toArray();
    QJsonObject action_0 = actionArray.at(0).toObject();
    QString action = action_0.value("action").toString();
    if (action == "goto")
    {
        QJsonObject action_params = action_0.value("params").toObject();
        QString object = action_params.value("object").toString();
        QJsonArray location = action_params.value("location").toArray();
        plan.params["x"] = location.at(0).toDouble();
        plan.params["y"] = location.at(1).toDouble();
        plan.params["angle"] = location.at(2).toDouble();
        plan.action = Plan::Actions::GOTO;
        plan.target_place = object.toStdString();
    }
}

///////////////////////////////////////////////////
/// GUI
//////////////////////////////////////////////////
void SpecificWorker::draw_path(std::list<QPointF> &path, QGraphicsScene* viewer_2d)
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

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}


/**************************************/

//Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");

//void SpecificWorker::compute()
//{
//    //const auto &[search_state, candidate] = search_a_feasible_target(target, current_plan.params, robot);
//    std::list<QPointF> path;
//    QPointF currentRobotNose;
//    Mat::Vector3d nose_3d;
//    switch (search_state)
//    {
//        case SearchState::NEW_TARGET:
//            break;
//        case SearchState::NEW_FLOOR_TARGET:
//            std::cout << __FUNCTION__ << " Candidate found on floor: " << std::endl;
//            nose_3d = inner_eigen->transform(world_name, Mat::Vector3d(0, 380, 0), robot_name).value();
//            currentRobotNose = QPointF(nose_3d.x(), nose_3d.y());
//            path = grid.computePath(currentRobotNose, QPointF(candidate.x(), candidate.y()));
//            qInfo() << __FUNCTION__ << " Path size: " << path.size();
//            for (auto &&p: path)
//                qInfo() << p;
//            if (not path.empty())
//            {
//                std::vector<float> x_values;
//                x_values.reserve(path.size());
//                std::transform(path.cbegin(), path.cend(), std::back_inserter(x_values),
//                               [](const auto &value) { return value.x(); });
//                std::vector<float> y_values;
//                y_values.reserve(path.size());
//                std::transform(path.cbegin(), path.cend(), std::back_inserter(y_values),
//                               [](const auto &value) { return value.y(); });
//                if (auto node_paths = G->get_nodes_by_type(path_to_target_type); not node_paths.empty())
//                {
//                    auto path_to_target_node = node_paths.front();
//                    G->add_or_modify_attrib_local<path_x_values_att>(path_to_target_node, x_values);
//                    G->add_or_modify_attrib_local<path_y_values_att>(path_to_target_node, y_values);
//                    G->add_or_modify_attrib_local<path_target_x_att>(path_to_target_node,
//                                                                     (float) candidate.x());
//                    G->add_or_modify_attrib_local<path_target_y_att>(path_to_target_node,
//                                                                     (float) candidate.y());
//                    G->update_node(path_to_target_node);
//                } else // create path_to_target_node with the solution path
//                {
//                    auto path_to_target_node = Node(agent_id, path_to_target_type);
//                    G->add_or_modify_attrib_local<path_x_values_att>(path_to_target_node, x_values);
//                    G->add_or_modify_attrib_local<path_y_values_att>(path_to_target_node, y_values);
//                    G->add_or_modify_attrib_local<pos_x_att>(path_to_target_node, (float) -150);
//                    G->add_or_modify_attrib_local<pos_y_att>(path_to_target_node, (float) -400);
//                    G->add_or_modify_attrib_local<parent_att>(path_to_target_node, intention_node.id());
//                    G->add_or_modify_attrib_local<level_att>(path_to_target_node, 3);
//                    G->add_or_modify_attrib_local<level_att>(path_to_target_node, 3);
//                    G->add_or_modify_attrib_local<path_target_x_att>(path_to_target_node,
//                                                                     (float) candidate.x());
//                    G->add_or_modify_attrib_local<path_target_y_att>(path_to_target_node,
//                                                                     (float) candidate.y());
//
//                    auto id = G->insert_node(path_to_target_node);
//                    auto edge_to_intention = Edge(id.value(), intention_node.id(), think_type,
//                                                  agent_id);
//                    G->insert_or_assign_edge(edge_to_intention);
//                }
//                draw_path(path, &widget_2d->scene);
//            }
//            break;
//        case SearchState::AT_TARGET:
//            std::cout << __FUNCTION__ << " At target " << std::endl;
//            break;
//        case SearchState::NO_TARGET_FOUND:
//            std::cout << __FUNCTION__ << " No target found: " << std::endl;
//            break;
//    }
//}
//}
//