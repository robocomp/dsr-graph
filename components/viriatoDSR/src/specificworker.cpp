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
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	G->write_to_json_file("/home/robocomp/robocomp/components/dsr-graph/etc/"+agent_name+".json");
    G.reset();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	agent_name = params["agent_name"].value;
    agent_id = stoi(params["agent_id"].value);
    read_dsr = params["read_dsr"].value == "true";
    dsr_input_file = params["dsr_input_file"].value;
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
		graph_viewer = std::make_unique<DSR::GraphViewer>(this, G, opts::scene, opts::scene);
		setWindowTitle(QString::fromStdString(agent_name + "-" + dsr_input_file));
        timer.start(100);
    }
}

void SpecificWorker::compute()
{
	if(auto ldata = laser_buffer.get(); ldata.has_value())
	 	update_laser(ldata.value());
	if(auto bState = omnirobot_buffer.get(); bState.has_value())
		update_omirobot(bState.value());
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
	auto robots = G->get_nodes_by_type("omnirobot"); //any omnirobot
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



