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
	G->write_to_json_file("./"+agent_name+".json");
    G.reset();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	agent_name = params["agent_name"].value;
    agent_id = stoi(params["agent_id"].value);
    read_dsr = params["read_dsr"].value == "true";
    dsr_input_file = params["dsr_input_file"].value;

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
		this->startup_check();
	else
	{
		G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id); // Init nodes
        std::cout<< __FUNCTION__ << "Graph loaded" << std::endl;  

		// Graph viewer
		using opts = DSR::GraphViewer::view;
		int current_opts = 0;
		opts main = opts::none;
		if(tree_view)
		{
			current_opts = current_opts | opts::tree;
		}
		if(graph_view)
		{
			current_opts = current_opts | opts::graph;
			main = opts::none;
		}
		if(qscene_2d_view)
		{
			current_opts = current_opts | opts::scene;
		}
		if(osg_3d_view)
		{
			current_opts = current_opts | opts::osg;
		}
		graph_viewer = std::make_unique<DSR::GraphViewer>(this, G, current_opts, main);
		setWindowTitle(QString::fromStdString(agent_name + "-" + dsr_input_file));
        timer.start(100);
    }
}

void SpecificWorker::compute()
{
    static RoboCompGenericBase::TBaseState my_bstate;

    if (auto ldata = laser_buffer.get(); ldata.has_value())
        update_laser(ldata.value());
    if (auto bState = omnirobot_buffer.get(); bState.has_value())
    {
        update_omirobot(bState.value());
        my_bstate = bState.value();
    }
    checkNewCommand(my_bstate);
	if(auto rgb = rgb_buffer.get(); rgb.has_value())
		update_rgb(rgb.value());
}

void SpecificWorker::update_rgb(const RoboCompCameraRGBDSimple::TImage& rgb)
{
	qDebug() << __FUNCTION__; 
	auto node = G->get_node("Viriato_head_camera_front_sensor");
	if (node.has_value())
	{
		G->add_or_modify_attrib_local(node.value(), "rgb", rgb.image);
		G->add_or_modify_attrib_local(node.value(), "width", rgb.width);
		G->add_or_modify_attrib_local(node.value(), "height", rgb.height);
		G->add_or_modify_attrib_local(node.value(), "depth", rgb.depth);
		G->add_or_modify_attrib_local(node.value(), "cameraID", rgb.cameraID);
		G->add_or_modify_attrib_local(node.value(), "focalx", rgb.focalx);
		G->add_or_modify_attrib_local(node.value(), "focaly", rgb.focaly);
		G->add_or_modify_attrib_local(node.value(), "alivetime", rgb.alivetime);		
		G->update_node(node.value());
	}
}

void SpecificWorker::update_laser(const RoboCompLaser::TLaserData& ldata)
{
    // Transform laserData into two std::vector<float>
    std::vector<float> dists;
    std::transform(ldata.begin(), ldata.end(), std::back_inserter(dists), [](const auto &l) { return l.dist; });
    std::vector<float> angles;
    std::transform(ldata.begin(), ldata.end(), std::back_inserter(angles), [](const auto &l) { return l.angle; });

	// update laser in DSR
	auto node = G->get_node("laser");
	if (node.has_value())
	{
		G->add_or_modify_attrib_local(node.value(), "dists", dists);
		G->add_or_modify_attrib_local(node.value(), "angles", angles);
		G->update_node(node.value());
	}
}

void SpecificWorker::update_omirobot(const RoboCompGenericBase::TBaseState& bState)
{
	static RoboCompGenericBase::TBaseState last_state;
	auto robots = G->get_nodes_by_type("omnirobot"); //cambiar por robot_name from config
	if (robots.size() == 0)
	{ 
		std::cout << __FUNCTION__ << " No node omnirobot" << std::endl; 
		return; 
	}
	auto robot = robots.at(0); //TODO: what sould be done if there are more than one robot?
	auto parent = G->get_parent_node(robot);  
	if(not parent.has_value()) 
	{ 
		std::cout << __FUNCTION__ << " No parent found for node " << robot.name() << std::endl; 
		return;
	}
	
	if( areDifferent(bState.x, last_state.x, FLT_EPSILON) or areDifferent(bState.z, last_state.z, FLT_EPSILON) or areDifferent(bState.alpha, last_state.alpha, FLT_EPSILON))
	{
		G->insert_or_assign_edge_RT(parent.value(), robot.id(), std::vector<float>{bState.x, 0., bState.z}, std::vector<float>{0., bState.alpha, 0.});
		last_state = bState;
	}
}

// Check if rotation_speed or advance_speed have changed and move the robot consequently
void SpecificWorker::checkNewCommand(const RoboCompGenericBase::TBaseState& bState)
{
    auto robot = G->get_node("omnirobot"); //any omnirobot
    if (not robot.has_value())
    {
        std::cout << __FUNCTION__ << " No node omnirobot" << std::endl;
        return;
    }
    auto desired_z_speed = G->get_attrib_by_name<float>(robot.value(), "advance_speed");
    auto desired_rot_speed = G->get_attrib_by_name<float>(robot.value(), "rotation_speed");
    auto desired_x_speed = G->get_attrib_by_name<float>(robot.value(), "side_speed");
    if(not desired_z_speed.has_value() or not desired_rot_speed.has_value() or not desired_x_speed.has_value())
    {
        std::cout << __FUNCTION__ << " No valid attributes for robot speed" << std::endl;
        return;
    }
    // Check de values are within robot's accepted range. Read them from config
    std::cout << __FUNCTION__ << desired_rot_speed.value() << " " << desired_x_speed.value() << " " << desired_z_speed.value() << std::endl;
    const float lowerA = -10, upperA = 10, lowerR = -10, upperR = 10, lowerS = -10, upperS = 10;
    std::clamp(desired_z_speed.value(), lowerA, upperA);
    std::clamp(desired_x_speed.value(), lowerS, upperS);
    std::clamp(desired_rot_speed.value(), lowerR, upperR);

//    if( not (lowerA < desired_z_speed.value() and desired_z_speed.value() < upperA
//           and lowerR < desired_rot_speed.value() and desired_rot_speed.value() < upperR
//           and lowerS < desired_x_speed.value() and desired_x_speed.value() < upperS))
//    {
//        qDebug() << __FUNCTION__ << "Desired speed values out of bounds:" << desired_z_speed.value() << desired_rot_speed.value() << "where bounds are:" << lowerA << upperA << lowerR << upperR;
//        return;
//    }
    if( areDifferent(bState.advVz, desired_z_speed.value(), FLT_EPSILON) or areDifferent(bState.rotV, desired_rot_speed.value(), FLT_EPSILON) or areDifferent(bState.advVx, desired_x_speed.value(), FLT_EPSILON))
    {
        qDebug() << __FUNCTION__ << "Diff detected" << desired_z_speed.value() << bState.advVz << desired_rot_speed.value() << bState.rotV << desired_x_speed.value() << bState.advVx;
        try
        {
            omnirobot_proxy->setSpeedBase(desired_x_speed.value(), desired_z_speed.value(), desired_rot_speed.value());
        }
        catch(const RoboCompGenericBase::HardwareFailedException &re)
        { std::cout << re << '\n';}
        catch(const Ice::Exception &e)
        { std::cout << e.what() << '\n';}
    }
}

bool SpecificWorker::areDifferent(float a, float b, float epsilon)
{ 
	return !((fabs(a - b) <= epsilon * std::max(fabs(a), fabs(b)))); 
};

//////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}


//SUBSCRIPTION to pushRGBD method from CameraRGBDSimplePub interface
void SpecificWorker::CameraRGBDSimplePub_pushRGBD(RoboCompCameraRGBDSimple::TImage im, RoboCompCameraRGBDSimple::TDepth dep)
{
	qDebug() << __FUNCTION__;
	rgb_buffer.put(std::move(im));
	depth_buffer.put(std::move(dep));	
}

//SUBSCRIPTION to pushLaserData method from LaserPub interface
void SpecificWorker::LaserPub_pushLaserData(RoboCompLaser::TLaserData laserData)
{
	laser_buffer.put(std::move(laserData));
}

//SUBSCRIPTION to pushBaseState method from OmniRobotPub interface
void SpecificWorker::OmniRobotPub_pushBaseState(RoboCompGenericBase::TBaseState state)
{
	omnirobot_buffer.put(std::move(state));
}



