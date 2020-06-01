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

/**
       \brief
       @author authorname
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

#include "../../../graph-related-classes/CRDT.h"
#include "../../../graph-related-classes/CRDT_graphviewer.h"
#include <doublebuffer/DoubleBuffer.h>
#include <b0RemoteApi.h>
#include <fps/fps.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cppitertools/chunked.hpp>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	
	void CameraRGBDSimple_getAll(TImage &im, TDepth &dep);
	void CameraRGBDSimple_getDepth(TDepth &dep);
	void CameraRGBDSimple_getImage(TImage &im);
	TLaserData Laser_getLaserAndBStateData(RoboCompGenericBase::TBaseState &bState);
	LaserConfData Laser_getLaserConfData();
	TLaserData Laser_getLaserData();
	void OmniRobot_correctOdometer(int x, int z, float alpha);
	void OmniRobot_getBasePose(int &x, int &z, float &alpha);
	void OmniRobot_getBaseState(RoboCompGenericBase::TBaseState &state);
	void OmniRobot_resetOdometer();
	void OmniRobot_setOdometer(RoboCompGenericBase::TBaseState state);
	void OmniRobot_setOdometerPose(int x, int z, float alpha);
	void OmniRobot_setSpeedBase(float advx, float advz, float rot);
	void OmniRobot_stopBase();
	void JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data);
	
	// DSR
	std::shared_ptr<CRDT::CRDTGraph> getGCRDT() const {return G;};

public slots:
	void compute();
	void initialize(int period);
	
private:
	// DSR
	std::shared_ptr<CRDT::CRDTGraph> G;
	
	//params
	std::string agent_name;
	int agent_id;
	bool read_dsr;
	std::string dsr_input_file;

	// graph viewer
	std::unique_ptr<DSR::GraphViewer> graph_viewer;
	QHBoxLayout mainLayout;
	QWidget window;

	/////////////////
	RoboCompCameraRGBDSimple::TDepth depth;
	RoboCompCameraRGBDSimple::TImage image;
	RoboCompLaser::TLaserData laser_data;
	int camera, laser, robot;
	b0RemoteApi *b0Client=nullptr;
	FPSCounter fps;
	std::string camera_name, laser_name, robot_name;
	bool SHOW_IMAGE = false;
	bool DEPTH = false;
	bool LASER = false;
	bool PUBLISH = false;
	bool IMAGE = true;

	DoubleBuffer<RoboCompCameraRGBDSimple::TImage, RoboCompCameraRGBDSimple::TImage> img_buffer;
	DoubleBuffer<RoboCompCameraRGBDSimple::TDepth, RoboCompCameraRGBDSimple::TDepth> depth_buffer;
	DoubleBuffer<RoboCompJoystickAdapter::TData, RoboCompJoystickAdapter::TData> joy_buffer;
	DoubleBuffer<RoboCompLaser::TLaserData, RoboCompLaser::TLaserData> laser_buffer;

};

#endif
