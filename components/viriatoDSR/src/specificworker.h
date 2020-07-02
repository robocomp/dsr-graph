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

/**
	\brief
	@author authorname
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"
#include "doublebuffer/DoubleBuffer.h"

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void CameraRGBDSimplePub_pushRGBD(RoboCompCameraRGBDSimple::TImage im, RoboCompCameraRGBDSimple::TDepth dep);
	void LaserPub_pushLaserData(RoboCompLaser::TLaserData laserData);
	void OmniRobotPub_pushBaseState(RoboCompGenericBase::TBaseState state);

	// DSR
    std::shared_ptr<DSR::DSRGraph> getGDSR() const {return G;};

public slots:
	void compute();
	int startup_check();
	void initialize(int period);
private:
	bool startup_check_flag;
	// DSR
	std::shared_ptr<DSR::DSRGraph> G;

	//params
	std::string agent_name;
	int agent_id;
	bool read_dsr;
	std::string dsr_input_file;

	// graph viewer
	std::unique_ptr<DSR::GraphViewer> graph_viewer;
	QHBoxLayout mainLayout;
	QWidget window;

	// buffers
	DoubleBuffer<RoboCompLaser::TLaserData, RoboCompLaser::TLaserData> laser_buffer;
	DoubleBuffer<RoboCompGenericBase::TBaseState, RoboCompGenericBase::TBaseState> omnirobot_buffer;
	DoubleBuffer<RoboCompCameraRGBDSimple::TImage, RoboCompCameraRGBDSimple::TImage> rgb_buffer;
	DoubleBuffer<RoboCompCameraRGBDSimple::TDepth, RoboCompCameraRGBDSimple::TDepth> depth_buffer;
	
	void update_laser(const RoboCompLaser::TLaserData& ldata);
	void update_omirobot(const RoboCompGenericBase::TBaseState& bState);
	bool areDifferent(float a, float b, float epsilon);

};

#endif
