/*
 *    Copyright (C) 2022 by YOUR NAME HERE
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
	G->write_to_json_file("./" + agent_name + ".json");
	G.reset();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	//	THE FOLLOWING IS JUST AN EXAMPLE
	//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
	//	try
	//	{
	//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
	//		std::string innermodel_path = par.value;
	//		innerModel = std::make_shared(innermodel_path);
	//	}
	//	catch(const std::exception &e) { qFatal("Error reading config params"); }

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
	if (this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		timer.start(Period);
		// create graph
		G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, ""); // Init nodes
		std::cout << __FUNCTION__ << "Graph loaded" << std::endl;

		// dsr update signals
		connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::modify_node_slot);
		connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::modify_edge_slot);
		connect(G.get(), &DSR::DSRGraph::update_node_attr_signal, this, &SpecificWorker::modify_attrs_slot);
		connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
		connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);

		// Graph viewer
		using opts = DSR::DSRViewer::view;
		int current_opts = 0;
		opts main = opts::none;
		if (tree_view)
		{
			current_opts = current_opts | opts::tree;
		}
		if (graph_view)
		{
			current_opts = current_opts | opts::graph;
			main = opts::graph;
		}
		if (qscene_2d_view)
		{
			current_opts = current_opts | opts::scene;
		}
		if (osg_3d_view)
		{
			current_opts = current_opts | opts::osg;
		}
		graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
		setWindowTitle(QString::fromStdString(agent_name + "-") + QString::number(agent_id));

		// Inner api
		inner_eigen = G->get_inner_eigen_api();

		// RT APi
		rt_api = G->get_rt_api();

		last_object = -1;

		this->Period = period;
		timer.start(Period);
	}
}

void SpecificWorker::compute()
{
	check_focus();
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

void SpecificWorker::check_focus()
{
	if (auto mind = G->get_node("mind"); mind.has_value())
	{
		// G->update_node(mind.value());

		if (auto edge_on_focus = G->get_edges_by_type("on_focus"); edge_on_focus.size() > 0)
		{
			std::cout << edge_on_focus.size() << std::endl;
			if (auto object = G->get_node(edge_on_focus.at(0).to()); object.has_value())
			{
				if (auto node_world = inner_eigen->transform("world", object.value().name()); node_world.has_value() && last_object != object.value().id())
				{
					last_object = object.value().id();
					std::cout << 23 << std::endl;
					Plan plan(Plan::Actions::GOTO);
					plan.insert_attribute("x", node_world.value()[0]);
					plan.insert_attribute("y", node_world.value()[1]);
					plan.insert_attribute("destiny", QString::fromStdString(object.value().name()));

					std::cout << 25 << std::endl;

					if (auto intention_node = G->get_node("current_intention"); intention_node.has_value())
					{
						std::cout << "X position:" << node_world.value()[0] << "Y position:" << node_world.value()[1] << std::endl;
						// plan.insert_attribute("x", node_world.value()[0]);
						// plan.insert_attribute("y", node_world.value()[1]);
						// plan.insert_attribute("destiny", QString::fromStdString("cup")
						if (plan.is_complete())
							G->add_or_modify_attrib_local<current_intention_att>(intention_node.value(), plan.to_json());

						G->update_node(intention_node.value());
					}
					else
					{
						DSR::Node new_intention_node = DSR::Node::create<intention_node_type>(current_intention_name);
						G->add_or_modify_attrib_local<parent_att>(new_intention_node, mind.value().id());
						G->add_or_modify_attrib_local<level_att>(new_intention_node, G->get_node_level(mind.value()).value() + 1);
						G->add_or_modify_attrib_local<pos_x_att>(new_intention_node, (float)-466);
						G->add_or_modify_attrib_local<pos_y_att>(new_intention_node, (float)42);

						if (plan.is_complete())
							G->add_or_modify_attrib_local<current_intention_att>(new_intention_node, plan.to_json());

						if (std::optional<int> new_intention_node_id = G->insert_node(new_intention_node); new_intention_node_id.has_value())
						{
							std::cout << __FUNCTION__ << " Node \"Intention\" successfully inserted in G" << std::endl;
							// insert EDGE
							DSR::Edge edge = DSR::Edge::create<has_edge_type>(mind.value().id(), new_intention_node.id());
							if (G->insert_or_assign_edge(edge))
								std::cout << __FUNCTION__ << " Edge \"has_type\" inserted in G" << std::endl;
							else
								std::cout << __FILE__ << __FUNCTION__ << " Fatal error inserting new edge: " << mind.value().id() << "->" << new_intention_node_id.value()
										  << " type: has" << std::endl;
						}
						else
							std::cout << __FUNCTION__ << " Node \"Intention\" could NOT be inserted in G" << std::endl;
					}
				}

				if (auto current_path_node = G->get_node("current_path"); not current_path_node.has_value())
				{
					std::cout << "MoveBase" << std::endl;
					//Cuidado
					move_base();
					track_object_of_interest(object.value());
				}
				else
				{
					if (auto pan_tilt = G->get_node("viriato_head_camera_pan_tilt"); pan_tilt.has_value())
					{
						G->add_or_modify_attrib_local<nose_pose_ref_att>(pan_tilt.value(), std::vector<float>{0.0, 1000.0, 0.0});
						G->update_node(pan_tilt.value());
					}
				}
				// else
				// 	if(auto visible_edges = G->get_edges_by_type("visible"); visible_edges.size() > 0)
				// 		for( auto visible_edge : visible_edges)
				// 			if (auto visible_object = G->get_node(visible_edge.to()); visible_object.has_value())
				// 				if(visible_object.value().id()==object.value().id())
				// 					track_object_of_interest(object.value());
			}
		}
		else
			std::cout << "NO TENGO NADA QUE HACER " << std::endl;
	}
}

void SpecificWorker::track_object_of_interest(DSR::Node object)
{
	if (auto pan_tilt = G->get_node("viriato_head_camera_pan_tilt"); pan_tilt.has_value())
	{
		// get object pose in world coordinate frame
		auto po = inner_eigen->transform(world_name, object.name());
		auto pose = inner_eigen->transform("viriato_head_camera_pan_tilt", object.name());
		if (po.has_value() and pose.has_value())
		{
			G->add_or_modify_attrib_local<nose_pose_ref_att>(pan_tilt.value(), std::vector<float>{(float)pose.value().x(), (float)pose.value().y(), (float)pose.value().z()});
			G->update_node(pan_tilt.value());
			// qInfo() << "NOW ...." << pose.value().x() << pose.value().y() << pose.value().z();
			if (auto tr2 = G->get_attrib_by_name<nose_pose_ref_att>(pan_tilt.value()); tr2.has_value())
			{
				; // qInfo() << tr2.value().get();
			}
		}
	}
}

void SpecificWorker::move_base()
{
	if (auto robot = G->get_node("robot"); robot.has_value())
	{
		if (auto head_camera_pan_joint = G->get_node("viriato_head_camera_pan_joint"); head_camera_pan_joint.has_value())
		{
			auto cam_axis = inner_eigen->transform_axis("robot", "viriato_head_camera_pan_joint");

			std::cout << "ANGULO DE LA CAMARITA " << cam_axis.value()[5] * 180.0 / M_PI << std::endl;

			//(abs(cam_axis.value()[5] * 180.0 / M_PI) > 0.2) || (abs(cam_axis.value()[5] * 180.0 / M_PI) < -0.2)
			if ((abs(cam_axis.value()[5] * 180.0 / M_PI) > 0.2) || (abs(cam_axis.value()[5] * 180.0 / M_PI) < -0.2))
			{
				float rot_speed = cam_axis.value()[5];
				std::cout << "giro" << std::endl;
				G->add_or_modify_attrib_local<robot_ref_adv_speed_att>(robot.value(), 0.0f);
				G->add_or_modify_attrib_local<robot_ref_side_speed_att>(robot.value(), 0.0f);
				G->add_or_modify_attrib_local<robot_ref_rot_speed_att>(robot.value(), -rot_speed);
			}
			else
			{
				std::cout << "no giro" << std::endl;
				G->add_or_modify_attrib_local<robot_ref_adv_speed_att>(robot.value(), 0.0f);
				G->add_or_modify_attrib_local<robot_ref_side_speed_att>(robot.value(), 0.0f);
				G->add_or_modify_attrib_local<robot_ref_rot_speed_att>(robot.value(), 0.0f);
			}
			G->update_node(robot.value());
		}
	}
}
