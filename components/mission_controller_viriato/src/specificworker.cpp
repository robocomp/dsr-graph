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
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/enumerate.hpp>

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
	G->write_to_json_file("./"+agent_name+".json");
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
		opts main = opts::none;
		if(tree_view)
		    current_opts = current_opts | opts::tree;
		if(graph_view)
		{
		    current_opts = current_opts | opts::graph;
		    main = opts::graph;
		}
		if(qscene_2d_view)
		    current_opts = current_opts | opts::scene;
		if(osg_3d_view)
		    current_opts = current_opts | opts::osg;

		graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
		setWindowTitle(QString::fromStdString(agent_name + "-") + QString::number(agent_id));

        // 2D widget
        widget_2d = qobject_cast<DSR::QScene2dViewer *>(graph_viewer->get_widget(opts::scene));
        widget_2d->set_draw_laser(true);
        connect(widget_2d, SIGNAL(mouse_right_click(int, int, std::uint64_t)), this, SLOT(new_target_from_mouse(int, int, std::uint64_t)));

        // custom widget
        graph_viewer->add_custom_widget_to_dock("Pioneer Plan Controller", &custom_widget);
        connect(custom_widget.pushButton_start_mission, SIGNAL(clicked()), this, SLOT(slot_start_mission()));
        connect(custom_widget.pushButton_stop_mission, SIGNAL(clicked()), this, SLOT(slot_stop_mission()));
        connect(custom_widget.pushButton_cancel_mission, SIGNAL(clicked()), this, SLOT(slot_cancel_mission()));

        // Inner Api
        inner_eigen = G->get_inner_eigen_api();

        this->Period = period;
		timer.start(Period);
	}

}

void SpecificWorker::compute()
{
	//computeCODE
	//QMutexLocker locker(mutex);
	//try
	//{
	//  camera_proxy->getYImage(0,img, cState, bState);
	//  memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
	//  searchTags(image_gray);
	//}
	//catch(const Ice::Exception &e)
	//{
	//  std::cout << "Error reading from Camera" << e << std::endl;
	//}
	
	
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// Asynchronous changes on G nodes from G signals
////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::add_or_assign_node_slot(const std::uint64_t id, const std::string &type)
{
   if (type == path_to_target_type_name)
    {
        if( auto path_to_target_node = G->get_node(id); path_to_target_node.has_value())
        {
            auto x_values_o = G->get_attrib_by_name<path_x_values_att>(path_to_target_node.value());
            auto y_values_o = G->get_attrib_by_name<path_y_values_att >(path_to_target_node.value());
            if(x_values_o.has_value() and y_values_o.has_value())
            {
                path.clear();
                auto &x_values = x_values_o.value().get();
                auto &y_values = y_values_o.value().get();
                path.reserve(x_values.size());
                for(auto &&[p, q] : iter::zip(x_values,y_values))
                    path.emplace_back(Eigen::Vector3d(p, q, 0.f));
                draw_path(path, &widget_2d->scene);
            }
        }
    }
}



void SpecificWorker::draw_path(std::vector<Eigen::Vector3d> &path, QGraphicsScene* viewer_2d)
{
    static std::vector<QGraphicsLineItem *> scene_road_points;

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

///////////////////////////////////////////////////////
//// Check new target from mouse
///////////////////////////////////////////////////////
void SpecificWorker::new_target_from_mouse(int pos_x, int pos_y, std::uint64_t id)
{
    qInfo() << __FUNCTION__ << " Creating GOTO mission to " << pos_x << pos_y;
    const std::string location =
            "[" + std::to_string(pos_x) + "," + std::to_string(pos_y) + "," + std::to_string(0) + "]";
    const std::string plan_string =
            "{\"plan\":[{\"action\":\"goto\",\"params\":{\"location\":" + location + ",\"object\":\"" + "floor" + "\"}}]}";
    std::cout << __FUNCTION__ << " " << plan_string << std::endl;
    auto plan = Plan(plan_string);
    plan_buffer.put(plan);

    // Check if there is not 'intention' node yet in G
    if(auto robot = G->get_node(robot_name); robot.has_value())
    {
        if (auto intention_nodes = G->get_nodes_by_type(intention_type_name); intention_nodes.empty())
        {
            DSR::Node intention_node = DSR::Node::create<intention_node_type>(current_intention_name);
            G->add_or_modify_attrib_local<parent_att>(intention_node, robot.value().id());
            G->add_or_modify_attrib_local<level_att>(intention_node, G->get_node_level(robot.value()).value() + 1);
            G->add_or_modify_attrib_local<pos_x_att>(intention_node, (float) -90);
            G->add_or_modify_attrib_local<pos_y_att>(intention_node, (float) -304);
            G->add_or_modify_attrib_local<current_intention_att>(intention_node, plan.to_string());
            if (std::optional<int> intention_node_id = G->insert_node(intention_node); intention_node_id.has_value())
            {
                std::cout << __FUNCTION__ << " Node \"Intention\" successfully inserted in G" << std::endl;
                // insert EDGE
                DSR::Edge edge = DSR::Edge::create<has_edge_type>(robot.value().id(), intention_node.id());
                if (G->insert_or_assign_edge(edge))
                    std::cout << __FUNCTION__ << " Edge \"has_type\" inserted in G" << std::endl;
                else
                    std::cout << __FILE__ << __FUNCTION__ << " Fatal error inserting new edge: " << robot.value().id() << "->" << intention_node_id.value()
                              << " type: has" << std::endl;
            } else
                std::cout << __FUNCTION__ << " Node \"Intention\" could NOT be inserted in G" << std::endl;
        } else // there is one intention node
        {
            DSR::Node intention_node = intention_nodes.front();
            G->add_or_modify_attrib_local<current_intention_att>(intention_node, plan.to_string());
            if (G->update_node(intention_node))
                std::cout << __FUNCTION__ << " Node \"Intention\" successfully updated in G" << std::endl;
            else
                std::cout << __FILE__ << __FUNCTION__ << " Fatal error inserting_new 'intention' node" << std::endl;
        }
    }
    else
    {
        std::cout << __FILE__ << __FUNCTION__ << " Fatal error. Robot node not found. Terminating" << std::endl;
        std::terminate();
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// UI
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void SpecificWorker::slot_start_mission()
{

}

void SpecificWorker::slot_stop_mission()
{

}

void SpecificWorker::slot_cancel_mission()
{

}


int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}
