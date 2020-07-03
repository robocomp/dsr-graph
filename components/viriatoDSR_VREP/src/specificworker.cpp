/*
 *    Copyright (C)2020 by YOUR NAME HERE
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
SpecificWorker::SpecificWorker(TuplePrx tprx) : GenericWorker(tprx)
{

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

	SHOW_IMAGE = params.at("ShowImage").value == "true";
	PUBLISH = params.at("Publish").value == "true";
	DEPTH = params.at("Depth").value == "true";
	LASER = params.at("Laser").value == "true"; 
	IMAGE = params.at("Image").value == "true";
	camera_name = params.at("CameraName").value;
	if(camera_name == "")
		qFatal("No camera provided, please check config file");
	laser_name = params.at("LaserName").value;
	if(laser_name == "")
		qFatal("No laser provided, please check config file");
	robot_name = params.at("RobotName").value;
	if(robot_name == "")
		qFatal("No robot provided, please check config file");

	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;

	// create graph
    G = std::make_shared<CRDT::CRDTGraph>(0, agent_name, agent_id); // Init nodes
	std::cout<< __FUNCTION__ << "Graph loaded" << std::endl;  

	// Graph viewer
	using opts = DSR::GraphViewer::View;
	graph_viewer = std::make_unique<DSR::GraphViewer>(G, std::list<opts>{opts::Scene});
	mainLayout.addWidget(graph_viewer.get());
	window.setLayout(&mainLayout);
	setCentralWidget(&window);
    setWindowTitle(QString::fromStdString(agent_name));
	
	// VREP
	b0Client = new b0RemoteApi("b0RemoteApi_c++Client","b0RemoteApiAddOn");
	qDebug() << __FUNCTION__ << " Connected to CoppeliaRobotics";
	auto handle_camera = b0Client->simxGetObjectHandle(camera_name.c_str(), b0Client->simxServiceCall());
	if( b0RemoteApi::readBool(handle_camera, 0))
		camera = b0RemoteApi::readInt(handle_camera, 1);
	else
		qFatal("Error getting camera handle");
	
	auto handle_robot = b0Client->simxGetObjectHandle("Viriato#0", b0Client->simxServiceCall());
	if( b0RemoteApi::readBool(handle_robot, 0))
		robot = b0RemoteApi::readInt(handle_robot, 1);
	else
		qFatal("Error getting robot handle");

	auto handle_laser = b0Client->simxGetObjectHandle(laser_name.c_str(), b0Client->simxServiceCall());
	if( b0RemoteApi::readBool(handle_laser, 0))
		laser = b0RemoteApi::readInt(handle_laser, 1);
	else
		qFatal("Error getting laser handle");
	
	this->Period = 50;
	timer.start(Period);
	qDebug() << __FUNCTION__ << " End Initialize";
}

void SpecificWorker::compute()
{
	if(IMAGE)
	{
		std::vector<int> size;
		//color image
		auto resImg = b0Client->simxGetVisionSensorImage(camera, false, b0Client->simxServiceCall());
		if(b0RemoteApi::readBool(resImg, 0))
		{
			b0RemoteApi::readIntArray(resImg, size, 1);
			int cols = size[0]; int rows = size[1]; int depth = 3; int len = cols*rows*depth;
			image.width = cols; image.height = rows; image.depth = 3; image.image.resize(len);
			memcpy(&image.image[0], b0RemoteApi::readByteArray(resImg, 2).data(), len);
			img_buffer.put(image);
		}
		else
			qDebug() << __FUNCTION__ << "Error capturing image";
	}	

	if( IMAGE and SHOW_IMAGE )
	{
		//cv::Mat cvimg = cv::Mat(cv::Size{640,480}, CV_8UC3,  b0RemoteApi::readByteArray(resImg, 2).data() );
		cv::Mat cvimg = cv::Mat(cv::Size{640,480}, CV_8UC3,  &image.image[0] );
		cv::Mat flipped;
		cv::flip(cvimg, flipped, 0);
		cv::imshow("", flipped);
		cv::waitKey(1);
	}

	//depth image
	if(DEPTH)
	{
		std::vector<int> size;
		auto resDepth = b0Client->simxGetVisionSensorDepthBuffer(camera, true, true, b0Client->simxServiceCall());
		if( b0RemoteApi::readBool(resDepth, 0)) 
		{
			b0RemoteApi::readIntArray(resDepth, size, 1);
			int dcols = size[0]; int drows = size[1]; int dlen = dcols*drows*4;  // OJO float size
			depth.cameraID = 0;
			depth.width = dcols; depth.height = drows; depth.focalx = 617; depth.focaly = 617; depth.alivetime = 0; 
			depth.depth.resize(dlen); 
			memcpy(&depth.depth[0], b0RemoteApi::readByteArray(resDepth, 2).data(), dlen);
			//depth_buffer.put(std::move(depth));
		}
		else
			qDebug() << __FUNCTION__ << "Error capturing depth";	
	}

	if(LASER)
	{
		laser_data.clear();
		std::vector<float> buffer;
		auto res_laser = b0Client->simxGetStringSignal("distances", b0Client->simxServiceCall());
		if(b0RemoteApi::readBool(res_laser, 0)) 
		{
			auto data = b0RemoteApi::readByteArray(res_laser, 1);
			std::vector<float> dists, angles;
			for(auto&& m: iter::chunked(data,12))
			{
				char cx[4] = {m[0],m[1],m[2],m[3]};
				float* x = (float*)cx;
				char cy[4] = {m[4],m[5],m[6],m[7]};
				float* y = (float*)cy;
				//std::cout << "[" << *x << " " << *y << "" << *z << "]" << std::endl;
				// x-axis of the laser points in the direction of movement of robot and z-axis is out of the plane. distance is in meters
				*x *= 1000; *y *= 1000;
				float dist = (float)sqrt(pow(*x,2) + pow(*y,2));
				float ang = atan2(*y,*x);
				laser_data.emplace_back(RoboCompLaser::TData{dist, ang});
				dists.emplace_back(dist);
				angles.push_back(ang);
			}
			laser_buffer.put(laser_data);
			auto node = G->get_node(202);
			if (node.has_value())
			{
				G->modify_attrib_local(node.value(), "dists", dists);
				G->modify_attrib_local(node.value(), "angles", angles);
				G->update_node(node.value());
			}
		}
		else
			qDebug() << __FUNCTION__ << "Error receiving laser data";
	}

	// Move robot from data in joy_buffer
	auto j = joy_buffer.get();
	if(j.has_value())
	{
		float rot = 0; float adv = 0; float side = 0;
		for(auto x: j.value().axes)
		{
			if(x.name=="advance")
				adv = x.value/100.;
			if(x.name=="rotate")
				rot = x.value/100.;
			if(x.name=="side")
				side = x.value/100.;
			// Replace this by direct call to b0
			OmniRobot_setSpeedBase(adv, side, rot);
		}
	}

	// Get robot pose and updated G
	auto base_pos = b0Client->simxGetObjectPosition(robot, -1, b0Client->simxServiceCall());
	auto base_rot = b0Client->simxGetObjectOrientation(robot, -1, b0Client->simxServiceCall());
	if(b0RemoteApi::readBool(base_pos, 0) and b0RemoteApi::readBool(base_rot, 0)) 
	{
		std::vector<float> pos_vector, rot_vector;
		auto rp = b0RemoteApi::readFloatArray(base_pos, pos_vector, 1);
		auto rr = b0RemoteApi::readFloatArray(base_rot, rot_vector, 1);
		if(rp and rr)
		{
			auto base = G->get_node(200);
			auto parent = G->get_node(G->get_node_parent(base.value()).value());  //change in API
			if (not parent.has_value()) return;
			pos_vector[0] *= 1000; pos_vector[1] *= 1000; pos_vector[2] *= 1000;
			G->insert_or_assign_edge_RT(parent.value(), base.value().id(), pos_vector, rot_vector);
			qDebug() << __FUNCTION__ << "Inserted:" << pos_vector[0] << pos_vector[1] << pos_vector[2];
			// if( areDifferent(bState.x, last_state.x, FLT_EPSILON) or areDifferent(bState.z, last_state.z, FLT_EPSILON) or areDifferent(bState.alpha, last_state.alpha, FLT_EPSILON))
			// {
			// 	G->insert_or_assign_edge_RT(parent.value(), base.id(), std::vector<float>{bState.x, 0., bState.z}, std::vector<float>{0., bState.alpha, 0.});
			// 	last_state = bState;		
			// }
		}
	}

	fps.print();
}

///////////////////////////////////////////////////////////////////
/// STUBS STUBS STUBS STUBS
//////////////////////////////////////////////////////////////////
/// Camera RGBD
//////////////////////////////////////////////////////////////////
void SpecificWorker::CameraRGBDSimple_getAll(TImage &im, TDepth &dep)
{	
	if(auto depth = depth_buffer.get(); depth.has_value())
		dep = depth.value();
	if(auto image = img_buffer.get(); image.has_value())
		im = image.value();
}

void SpecificWorker::CameraRGBDSimple_getDepth(TDepth &dep)
{
	if(auto depth = depth_buffer.get(); depth.has_value())
		dep = depth.value();
}

void SpecificWorker::CameraRGBDSimple_getImage(TImage &im)
{
	if(auto image = img_buffer.get(); image.has_value())
		im = image.value();
}

//////////////////////////////////////////////////////////////////
/// Laser
//////////////////////////////////////////////////////////////////
TLaserData SpecificWorker::Laser_getLaserAndBStateData(RoboCompGenericBase::TBaseState &bState)
{
 	if(auto ldata = laser_buffer.get(); ldata.has_value())
	 	return ldata.value();
	else
	 	return TLaserData();
}

LaserConfData SpecificWorker::Laser_getLaserConfData()
{
	return LaserConfData();
}

TLaserData SpecificWorker::Laser_getLaserData()
{
	if(auto ldata = laser_buffer.get(); ldata.has_value())
	 	return ldata.value();
	else
	 	return TLaserData();
}

//////////////////////////////////////////////////////////////////
/// Base
//////////////////////////////////////////////////////////////////
void SpecificWorker::OmniRobot_correctOdometer(int x, int z, float alpha)
{
}

void SpecificWorker::OmniRobot_getBasePose(int &x, int &z, float &alpha)
{
}

void SpecificWorker::OmniRobot_getBaseState(RoboCompGenericBase::TBaseState &state)
{
}

void SpecificWorker::OmniRobot_resetOdometer()
{
}

void SpecificWorker::OmniRobot_setOdometer(RoboCompGenericBase::TBaseState state)
{
}

void SpecificWorker::OmniRobot_setOdometerPose(int x, int z, float alpha)
{
}

// Replace this by an insertion in a DoubleBuffer
void SpecificWorker::OmniRobot_setSpeedBase(float advx, float advz, float rot)
{
	std::tuple<float, float, float> src(advx, advz, rot);
	std::stringstream buffer;
	msgpack::pack(buffer, src);
	auto res = b0Client->simxCallScriptFunction("setSpeed@Viriato#0", 1, buffer.str().c_str(), buffer.str().size(), b0Client->simxServiceCall());
	qDebug() << b0RemoteApi::readBool(res, 0);	
}

void SpecificWorker::OmniRobot_stopBase()
{

}

//////////////////////////////////////////////////////////////////
/// Joy
//////////////////////////////////////////////////////////////////

void SpecificWorker::JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data)
{
	joy_buffer.put(data);
}




