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
        setWindowTitle(QString::fromStdString(agent_name + "-" + dsr_input_file));

		connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::update_node_slot);
        
		//Inner Api
        inner_eigen = G->get_inner_eigen_api();

        // Ignore attributes from G
        G->set_ignored_attributes<rgb_att, depth_att>();

        // Custom widget
        dsr_viewer->add_custom_widget_to_dock("Social Navigation", &custom_widget);
		widget_2d = qobject_cast<DSR::QScene2dViewer*> (dsr_viewer->get_widget(opts::scene));

		// path planner
		path_planner_initialize(&widget_2d->scene, true, "viriato-200.grid");

        widget_2d->set_draw_laser(true);
		connect(widget_2d, SIGNAL(mouse_right_click(int, int, int)), this, SLOT(new_target_from_mouse(int,int,int)));

		this->Period = 100;
		timer.start(Period);
	}
}

void SpecificWorker::compute()
{
    // static Navigation<Grid<>,Controller>::SearchState last_state;
    //Eigen::IOFormat CleanFmt(Eigen::StreamPrecision, 0, ", ", "\n", "[", "]");
    Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");

    if( auto robot_o = G->get_node(robot_name); robot_o.has_value())
    {
        auto robot = robot_o.value();
        // Check for new plan
        if (auto plan_o = plan_buffer.try_get(); plan_o.has_value())
        {
            current_plan = plan_o.value();
            current_plan.print();
            if (auto target_o = G->get_node(current_plan.target_place); target_o.has_value())
            {
                auto target = target_o.value();
                if (auto path_to_target_o = G->get_node(path_to_target_name); path_to_target_o.has_value())
                {
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
                            std::cout << __FUNCTION__ << " Candidate found: " << candidate.format(CommaInitFmt)
                                      << std::endl;
                            nose_3d = inner_eigen->transform(world_name, Mat::Vector3d(250, 0, 0), robot_name).value();
                            currentRobotNose = QPointF(nose_3d.x(), nose_3d.y());
                            path = grid.computePath(currentRobotNose, QPointF(candidate.x(), candidate.y()));
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
        if (this->current_plan.is_location(Mat::Vector2d(params.at("x"), params.at("y"))))
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
    using namespace std::placeholders;
    if( auto target_node = G->get_node(id); target_node.has_value())
    {
        const std::string location = "[" + std::to_string(pos_x) + "," + std::to_string(pos_y) + "," +std::to_string(0) + "]";
        const std::string plan = "{\"plan\":[{\"action\":\"goto\",\"params\":{\"location\":" + location + ",\"object\":\"" + target_node.value().name() + "\"}}]}";
        plan_buffer.put(plan, std::bind(&SpecificWorker::json_to_plan, this,_1, _2));
    }
    else
        qWarning() << __FILE__ << __FUNCTION__ << " No target node  " << QString::number(id) << " found";
}

///////////////////////////////////////////////////
/// Asynchronous changes on G nodes from G signals
///////////////////////////////////////////////////
void SpecificWorker::update_node_slot(const std::int32_t id, const std::string &type)
{
    //check node type
    using namespace std::placeholders;
    if (type == "intention")
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


int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}


/**************************************/