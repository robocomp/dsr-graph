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
        //G->set_ignored_attributes<rgb_att, depth_att>();

        // Custom widget
        dsr_viewer->add_custom_widget_to_dock("Path Planner A-Star", &custom_widget);
		widget_2d = qobject_cast<DSR::QScene2dViewer*> (dsr_viewer->get_widget(opts::scene));

		// path planner
		path_follower_initialize();

        widget_2d->set_draw_laser(true);
		//connect(widget_2d, SIGNAL(mouse_right_click(int, int, int)), this, SLOT(new_target_from_mouse(int,int,int)));

		// check for existing intention node
		if(auto intentions = G->get_nodes_by_type(intention_type); not intentions.empty())
            this->update_node_slot(intentions.front().id(), "intention");

		this->Period = 100;
        std::cout<< __FUNCTION__ << "Initialization finished" << std::endl;
        timer.start(Period);
	}
}

void SpecificWorker::compute()
{
    static std::vector<QPointF> path;
    if( auto robot_o = G->get_node(robot_name); robot_o.has_value())
    {
        auto robot = robot_o.value();
        // Check for existing path_to_target_nodes
        if (auto path_o = path_buffer.try_get(); path_o.has_value()) // NEW PATH!
        {
            path.clear();
            path = path_o.value();
        }
        else    // KEEP WORKING IN OLD ONE
        {
            if( const auto laser_data = laser_buffer.try_get(); laser_data.has_value())
            {
                const auto &[laser_poly, laser_cart] = laser_data.value();
                //               qInfo() << "Path: " << path.size()  << " Laser size:" << laser_poly.size();
                // for (auto &&p:path)
                //      qInfo() << p;
                auto nose_3d = inner_eigen->transform(world_name, Mat::Vector3d(0, 360, 0), robot_name).value();
                auto current_robot_nose = QPointF(nose_3d.x(), nose_3d.y());
            }
        }
    }
}

void SpecificWorker::path_follower_initialize()
{

    robotXWidth = std::stof(conf_params->at("RobotXWidth").value);
    robotZLong = std::stof(conf_params->at("RobotZLong").value);
    robotBottomLeft     = Mat::Vector3d ( -robotXWidth / 2, robotZLong / 2, 0);
    robotBottomRight    = Mat::Vector3d ( - robotXWidth / 2,- robotZLong / 2, 0);
    robotTopRight       = Mat::Vector3d ( + robotXWidth / 2, - robotZLong / 2, 0);
    robotTopLeft        = Mat::Vector3d ( + robotXWidth / 2, + robotZLong / 2, 0);
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