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
#include <doublebuffer/DoubleBuffer.h>
#include  "../../../etc/viriato_graph_names.h"


class SpecificWorker : public GenericWorker
{
    using Clock = std::chrono::system_clock;
    using MSec = std::chrono::duration<double, std::milli>;

Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void CameraRGBDSimplePub_pushRGBD(RoboCompCameraRGBDSimple::TImage im, RoboCompCameraRGBDSimple::TDepth dep);
	void LaserPub_pushLaserData(RoboCompLaser::TLaserData laserData);
	void OmniRobotPub_pushBaseState(RoboCompGenericBase::TBaseState state);
    void JointMotorPub_motorStates(RoboCompJointMotor::MotorStateMap mstateMap);
    void KinovaArmPub_newArmState(RoboCompKinovaArmPub::TArmState armstate);

	// DSR
    std::shared_ptr<DSR::DSRGraph> getGDSR() const {return G;};

    // Double Buffers to receive data from ViriatoPyRep. Public to be used from the xxxI classes
    DoubleBuffer<RoboCompLaser::TLaserData, RoboCompLaser::TLaserData> laser_buffer;
    DoubleBuffer<RoboCompGenericBase::TBaseState, RoboCompGenericBase::TBaseState> omnirobot_buffer;
    DoubleBuffer<RoboCompCameraRGBDSimple::TImage, RoboCompCameraRGBDSimple::TImage> rgb_buffer;
    DoubleBuffer<RoboCompCameraRGBDSimple::TDepth, RoboCompCameraRGBDSimple::TDepth> depth_buffer;
    DoubleBuffer<RoboCompJointMotor::MotorStateMap, RoboCompJointMotor::MotorStateMap> jointmotor_buffer;
    DoubleBuffer<RoboCompKinovaArmPub::TArmState, RoboCompKinovaArmPub::TArmState> kinovaarm_buffer;


public slots:
	void compute();
	int startup_check();
	void initialize(int period);
    void update_node_slot(const std::int32_t id, const std::string &type);

private:
    bool startup_check_flag;

    // ATTRIBUTE NAMES
    const std::string nose_target = "viriato_head_pan_tilt_nose_target";
    const std::string arm_tip_target = "viriato_left_arm_target";

    // DSR
	std::shared_ptr<DSR::DSRGraph> G;
    std::shared_ptr<DSR::InnerAPI> innermodel;

	//params
	std::string agent_name;
	int agent_id;
	bool read_dsr;
	std::string dsr_input_file;

	int tree_view;
	int graph_view;
	int qscene_2d_view;
	int osg_3d_view;

	// Graph Biewer
	std::unique_ptr<DSR::DSRViewer> dsr_viewer;
	QHBoxLayout mainLayout;
	QWidget window;
    std::unordered_map<int, int> G_person_id;


    void update_laser();
    RoboCompGenericBase::TBaseState update_omirobot();
	void update_rgbd();
    void update_pantilt_position();
    void update_arm_state();

	bool are_different(const std::vector<float> &a, const std::vector<float> &b, const std::vector<float> &epsilon);

	//void check_new_base_command(const RoboCompGenericBase::TBaseState& bState);
    //void check_new_dummy_values_for_coppelia();
    void check_new_nose_referece_for_pan_tilt();

};

#endif
