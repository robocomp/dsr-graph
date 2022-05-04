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


//#DEFINE cam_range_angle 35;

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	//std::cout << "Destroying SpecificWorker" << std::endl;
	G->write_to_json_file("./"+agent_name+".json");
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
		connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::modify_node_slot);
		connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::modify_edge_slot);
		connect(G.get(), &DSR::DSRGraph::update_node_attr_signal, this, &SpecificWorker::modify_attrs_slot);
		connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
		connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);

		// Graph viewer
		using opts = DSR::DSRViewer::view;
		int current_opts = 0;
		opts main = opts::none;
		if(tree_view)
		{
		    current_opts = current_opts | opts::tree;
		}
		if(graph_view)
		{
		    current_opts = current_opts | opts::graph;
		    main = opts::graph;
		}
		if(qscene_2d_view)
		{
		    current_opts = current_opts | opts::scene;
		}
		if(osg_3d_view)
		{
		    current_opts = current_opts | opts::osg;
		}
		graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
		setWindowTitle(QString::fromStdString(agent_name + "-") + QString::number(agent_id));

		//Inner api
	    inner_eigen = G->get_inner_eigen_api();

		// RT APi
        rt_api = G->get_rt_api();

		this->Period = period;
		timer.start(Period);
	}

}

void SpecificWorker::compute()
{

	// auto object = G->get_node("cup");

	// if(object.has_value())
	// 	set_attention(object.value());
	
	track_object_of_interest();
	move_base();

}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

// void SpecificWorker::set_attention(DSR::Node &node)
// {
// 	//static bool already_executed = false;

// 	if(	auto mind = G->get_node("mind") ; mind.has_value())
// 	{
// 		// if(!already_executed)
// 		// {
// 			auto edge = DSR::Edge::create<on_focus_edge_type>(mind.value().id(), node.id());
// 			G->insert_or_assign_edge(edge);
// 			//already_executed = true;
// 		//}
// 	}
// 	else
// 		qWarning() << "Mind node has no value";
// }

void SpecificWorker::track_object_of_interest()
{
	//Sacar fuera?
    auto focus_edge = G->get_edges_by_type("on_focus");

    if (focus_edge.size() > 0)
    {
        static Eigen::Vector3d ant_pose;
        // auto object = G->get_node("cup");
        auto pan_tilt = G->get_node("viriato_head_camera_pan_tilt");
        auto object = G->get_node(focus_edge.at(0).to());

        if (object.has_value() and pan_tilt.has_value())
        {
            // get object pose in world coordinate frame
            auto po = inner_eigen->transform(world_name, object.value().name());
            auto pose = inner_eigen->transform("viriato_head_camera_pan_tilt", object.value().name());
            // pan-tilt center
            if (po.has_value() and pose.has_value() /*and ((pose.value() - ant_pose).cwiseAbs2().sum() > 10)*/) // OJO AL PASAR A METROS
            {
                //            G->add_or_modify_attrib_local<viriato_head_pan_tilt_nose_target_att>(pan_tilt.value(), std::vector<float>{(float)po.value().x(), (float)po.value().y(), (float)po.value().z()});
                G->add_or_modify_attrib_local<nose_pose_ref_att>(pan_tilt.value(), std::vector<float>{(float)pose.value().x(), (float)pose.value().y(), (float)pose.value().z()});
                G->update_node(pan_tilt.value());
                qInfo() << "NOW ...." << pose.value().x() << pose.value().y() << pose.value().z();
                if (auto tr2 = G->get_attrib_by_name<nose_pose_ref_att>(pan_tilt.value()); tr2.has_value())
                {
                    qInfo() << tr2.value().get();
                }
            }
            // ant_pose = pose.value();
            //move_base(robot);
        }
    }
}

void SpecificWorker::move_base()
{
	if(auto robot = G->get_node("robot") ; robot.has_value())
	{
		if( auto head_camera_pan_joint = G->get_node("viriato_head_camera_pan_joint") ; head_camera_pan_joint.has_value())
		{
			auto cam_axis = inner_eigen->transform_axis("robot", "viriato_head_camera_pan_joint");	

			// std::cout << "ANGULOS DE LA CAMARA " << (float)cam_axis.value()[3] << std::endl;
			// std::cout << "ANGULOS DE LA CAMARA " << (float)cam_axis.value()[4] << std::endl;
			std::cout << "ANGULOS DE LA CAMARA " << cam_axis.value()[5]*180.0/M_PI << std::endl;

			if( (abs(cam_axis.value()[5]*180.0/M_PI) != 0.0))
			{
				float rot_speed = cam_axis.value()[5]*0.3f;
				G->add_or_modify_attrib_local<robot_ref_adv_speed_att>(robot.value(), 0.0f);
				G->add_or_modify_attrib_local<robot_ref_side_speed_att>(robot.value(), 0.0f);
				G->add_or_modify_attrib_local<robot_ref_rot_speed_att>(robot.value(), -rot_speed);
				G->update_node(robot.value());
			}
			else
			{
				G->add_or_modify_attrib_local<robot_ref_adv_speed_att>(robot.value(), 0.0f);
				G->add_or_modify_attrib_local<robot_ref_side_speed_att>(robot.value(), 0.0f);
				G->add_or_modify_attrib_local<robot_ref_rot_speed_att>(robot.value(), 0.0f);
				G->update_node(robot.value());
			}

		}
	}
}


