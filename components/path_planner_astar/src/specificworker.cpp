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
        
		//Inner Api
        inner_eigen = G->get_inner_eigen_api();

        // Ignore attributes from G
        G->set_ignored_attributes<rgb_att, depth_att>();

        // Custom widget
        dsr_viewer->add_custom_widget_to_dock("Path Planner A-Star", &custom_widget);
		widget_2d = qobject_cast<DSR::QScene2dViewer*> (dsr_viewer->get_widget(opts::scene));

		// path planner
		path_planner_initialize(&widget_2d->scene, true, "viriato-200-vrep.grid");

        widget_2d->set_draw_laser(true);
		connect(widget_2d, SIGNAL(mouse_right_click(int, int, int)), this, SLOT(new_target_from_mouse(int,int,int)));

		// check for existing intention node
		if(auto intentions = G->get_nodes_by_type(intention_type); not intentions.empty())
            this->update_node_slot(intentions.front().id(), "intention");

		this->Period = 100;
		timer.start(Period);
	}
}

void SpecificWorker::compute()
{
    // static Navigation<Grid<>,Controller>::SearchState last_state;
    Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");
    if( auto robot_o = G->get_node(robot_name); robot_o.has_value())
    {
        auto robot = robot_o.value();
        // Check for new plan
        if (auto plan_o = plan_buffer.try_get(); plan_o.has_value())
        {
            current_plan = plan_o.value();
            current_plan.print();
            if( auto intention_nodes = G->get_nodes_by_type(intention_type); not intention_nodes.empty())
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
                            std::cout << __FUNCTION__ << " Candidate found: " << candidate.format(CommaInitFmt)
                                      << std::endl;
                            break;
                        case SearchState::NEW_FLOOR_TARGET:
                            std::cout << __FUNCTION__ << " Candidate found on floor: " << std::endl;
                            nose_3d = inner_eigen->transform(world_name, Mat::Vector3d(0, 380, 0), robot_name).value();
                            currentRobotNose = QPointF(nose_3d.x(), nose_3d.y());
                            path = grid.computePath(currentRobotNose, QPointF(candidate.x(), candidate.y()));
                            qInfo() << __FUNCTION__ << " Path size: " << path.size();
                            for (auto &&p: path)
                                qInfo() << p;
                            if (not path.empty())
                            {
                                std::vector<float> x_values;
                                x_values.reserve(path.size());
                                std::transform(path.cbegin(), path.cend(), std::back_inserter(x_values),
                                               [](const auto &value) { return value.x(); });
                                std::vector<float> y_values;
                                y_values.reserve(path.size());
                                std::transform(path.cbegin(), path.cend(), std::back_inserter(y_values),
                                               [](const auto &value) { return value.y(); });
                                if (auto node_paths = G->get_nodes_by_type(path_to_target_type); not node_paths.empty())
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
                                    auto path_to_target_node = Node(agent_id, path_to_target_type);
                                    G->add_or_modify_attrib_local<path_x_values_att>(path_to_target_node, x_values);
                                    G->add_or_modify_attrib_local<path_y_values_att>(path_to_target_node, y_values);
                                    G->add_or_modify_attrib_local<pos_x_att>(path_to_target_node, (float) -150);
                                    G->add_or_modify_attrib_local<pos_y_att>(path_to_target_node, (float) -400);
                                    G->add_or_modify_attrib_local<parent_att>(path_to_target_node, intention_node.id());
                                    G->add_or_modify_attrib_local<level_att>(path_to_target_node, 3);
                                    G->add_or_modify_attrib_local<level_att>(path_to_target_node, 3);
                                    G->add_or_modify_attrib_local<path_target_x_att>(path_to_target_node,
                                                                                     (float) candidate.x());
                                    G->add_or_modify_attrib_local<path_target_y_att>(path_to_target_node,
                                                                                     (float) candidate.y());

                                    auto id = G->insert_node(path_to_target_node);
                                    auto edge_to_intention = Edge(id.value(), intention_node.id(), think_type,
                                                                  agent_id);
                                    G->insert_or_assign_edge(edge_to_intention);
                                }
                                draw_path(path, &widget_2d->scene);
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
    outerRegion.setLeft(G->get_attrib_by_name<OuterRegionLeft>(world_node).value());
    outerRegion.setRight(G->get_attrib_by_name<OuterRegionRight>(world_node).value());
    outerRegion.setBottom(G->get_attrib_by_name<OuterRegionBottom>(world_node).value());
    outerRegion.setTop(G->get_attrib_by_name<OuterRegionTop>(world_node).value());
    if(outerRegion.isNull())
    {
        qWarning() << __FILE__ << __FUNCTION__ << "Outer region of the scene could not be found in G. Aborting";
        std::terminate();
    }
    // if read_from_file is true we should read the parameters from the file to guarantee consistency
    dim.HMIN = std::min(outerRegion.left(), outerRegion.right());
    dim.WIDTH = std::max(outerRegion.left(), outerRegion.right()) - dim.HMIN;
    dim.VMIN = std::min(outerRegion.top(), outerRegion.bottom());
    dim.HEIGHT = std::max(outerRegion.top(), outerRegion.bottom()) - dim.VMIN;
    std::cout << __FUNCTION__ << "TileSize is " << conf_params->at("TileSize").value << std::endl;
    dim.TILE_SIZE = stoi(conf_params->at("TileSize").value);

    collisions =  std::make_shared<Collisions>();
    collisions->initialize(G, conf_params);

    grid.initialize(G, collisions, dim, read_from_file, file_name);
    grid.draw(&widget_2d->scene);

    robotXWidth = std::stof(conf_params->at("RobotXWidth").value);
    robotZLong = std::stof(conf_params->at("RobotZLong").value);
    robotBottomLeft     = Mat::Vector3d ( -robotXWidth / 2, robotZLong / 2, 0);
    robotBottomRight    = Mat::Vector3d ( - robotXWidth / 2,- robotZLong / 2, 0);
    robotTopRight       = Mat::Vector3d ( + robotXWidth / 2, - robotZLong / 2, 0);
    robotTopLeft        = Mat::Vector3d ( + robotXWidth / 2, + robotZLong / 2, 0);
}

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
    int d = dim.TILE_SIZE;
    for(int i : iter::range(grid.size()))
    {
        while (2 * x_pos * d < i)
        {
            const auto &k = Grid<>::Key(x_pos, y_pos);
            if (grid.isFree(k))
                candidates.emplace_back(Mat::Vector2d (x_pos, y_pos));
            x_pos = x_pos + d;
            if(candidates.size() > 0)   // arbitrary number hard to fix
                break;
        }
        while (2 * y_pos * d < i)
        {
            const auto &k = Grid<>::Key(x_pos, y_pos);
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


///////////////////////////////////////////////////////
//// Check new target from mouse
///////////////////////////////////////////////////////

void SpecificWorker::new_target_from_mouse(int pos_x, int pos_y, int id)
{
    // Check if there is not 'intention' node yet in G
    if (auto intention_nodes = G->get_nodes_by_type(intention_type); intention_nodes.empty())
    {
        if(auto robot = G->get_node(robot_name); robot.has_value())
        {
            Node intention_node(agent_id, intention_type);
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
void SpecificWorker::update_node_slot(const std::int32_t id, const std::string &type)
{
    // check node type
    using namespace std::placeholders;
    if (type == intention_type)
    {
        auto node = G->get_node(id);
        if (auto parent = G->get_parent_node(node.value()); parent.has_value() and parent.value().name() == robot_name)
        {
            std::optional<std::string> plan = G->get_attrib_by_name<plan_att>(node.value());
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