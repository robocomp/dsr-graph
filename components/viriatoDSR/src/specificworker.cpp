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
#include <cppitertools/zip.hpp>

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
	tree_view = (params["tree_view"].value == "true") ? DSR::DSRViewer::view::tree : 0;
	graph_view = (params["graph_view"].value == "true") ? DSR::DSRViewer::view::graph : 0;
	qscene_2d_view = (params["2d_view"].value == "true") ? DSR::DSRViewer::view::scene : 0;
	osg_3d_view = (params["3d_view"].value == "true") ? DSR::DSRViewer::view::osg : 0;
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
        std::cout<< __FUNCTION__ << " - Graph loaded" << std::endl;

        //Inner Api
        innermodel = G->get_inner_api();

        // Remove existing pan-tilt target
        if(auto pan_tilt = G->get_node(viriato_head_camera_pan_tilt); pan_tilt.has_value())
        {
            auto nose = innermodel->transformS("world", QVec::vec3(10,0,0),viriato_head_camera_name).value();
            G->add_or_modify_attrib_local<viriato_head_pan_tilt_nose_target>(pan_tilt.value(), std::vector<float>{nose.x(),nose.y(),nose.z()});
            G->update_node(pan_tilt.value());
        }

        // Set base speed reference to 0
        if(auto robot = G->get_node(this->robot_name); robot.has_value())
        {
            G->insert_or_assign_attrib<ref_adv_speed_att>(robot.value(), (float)0);
            G->insert_or_assign_attrib<ref_rot_speed_att>(robot.value(), (float)0);
            G->insert_or_assign_attrib<ref_side_speed_att>(robot.value(), (float)0);
        }

        // Graph viewer
		using opts = DSR::DSRViewer::view;
		int current_opts = tree_view | graph_view | qscene_2d_view | osg_3d_view;
		opts main = opts::none;
        if (graph_view)
            main = opts::graph;
		dsr_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
		setWindowTitle(QString::fromStdString(agent_name + "-" + std::to_string(agent_id)));
        timer.start(100);
    }
}

void SpecificWorker::compute()
{
    update_laser();
    auto bState = update_omirobot();
    update_rgbd();
    update_pantilt_position();
    check_new_dummy_values_for_coppelia();
    check_new_nose_referece_for_pan_tilt();
    check_new_base_command(bState);
}

/////////////////////////////////////////////////////////////////////////////////////////////////

void SpecificWorker::update_rgbd()
{
    const auto rgb_o = rgb_buffer.try_get();
    const  auto depth_o = depth_buffer.try_get();
    if (rgb_o.has_value() and depth_o.has_value())
    {
        const auto rgb = rgb_o.value(); const auto depth = depth_o.value();
        auto node = G->get_node(viriato_head_camera_name);
        if (node.has_value())
        {
            G->add_or_modify_attrib_local<rgb_att>(node.value(), rgb.image);
            G->add_or_modify_attrib_local<rgb_width>(node.value(), rgb.width);
            G->add_or_modify_attrib_local<rgb_height>(node.value(), rgb.height);
            G->add_or_modify_attrib_local<rgb_depth>(node.value(), rgb.depth);
            G->add_or_modify_attrib_local<rgb_cameraID>(node.value(), rgb.cameraID);
            G->add_or_modify_attrib_local<rgb_focalx>(node.value(), rgb.focalx);
            G->add_or_modify_attrib_local<rgb_focaly>(node.value(), rgb.focaly);
            G->add_or_modify_attrib_local<rgb_alivetime>(node.value(), rgb.alivetime);

            // depth
            G->add_or_modify_attrib_local<img_depth>(node.value(), depth.depth);
            G->add_or_modify_attrib_local<depth_width>(node.value(), depth.width);
            G->add_or_modify_attrib_local<depth_height>(node.value(), depth.height);
            G->add_or_modify_attrib_local<focalx>(node.value(), depth.focalx);
            G->add_or_modify_attrib_local<focaly>(node.value(), depth.focaly);
            G->add_or_modify_attrib_local<depth_cameraID>(node.value(), depth.cameraID);
            G->add_or_modify_attrib_local<depthFactor>(node.value(), depth.depthFactor);
            G->add_or_modify_attrib_local<alivetime>(node.value(), depth.alivetime);
            G->update_node(node.value());
        }
    } else std::this_thread::yield();
}

void SpecificWorker::update_laser()
{
    if (auto ldata = laser_buffer.try_get(); ldata.has_value())
    {
        // Transform laserData into two std::vector<float>
        auto data = ldata.value();
        std::vector<float> dists;
        std::transform(data.begin(), data.end(), std::back_inserter(dists), [](const auto &l) { return l.dist; });
        std::vector<float> angles;
        std::transform(data.begin(), data.end(), std::back_inserter(angles), [](const auto &l) { return l.angle; });

        // update laser in DSR
        auto node = G->get_node("laser");
        if (node.has_value())
        {
            G->add_or_modify_attrib_local<dists_att>(node.value(), dists);
            G->add_or_modify_attrib_local<angles_att>(node.value(), angles);
            G->update_node(node.value());
        }
    }
    else std::this_thread::yield();
}

RoboCompGenericBase::TBaseState SpecificWorker::update_omirobot()
{
    static RoboCompGenericBase::TBaseState last_state;
    if (auto bState_o = omnirobot_buffer.try_get(); bState_o.has_value())
    {
        const auto bState = bState_o.value();
        auto robot = G->get_node(robot_name);
        if (not robot.has_value())
        {
            qWarning() << __FUNCTION__ << " No node " << QString::fromStdString(robot_name);
            return last_state;
        }
        auto parent = G->get_parent_node(robot.value());
        if (not parent.has_value())
        {
            qWarning() << __FUNCTION__ << " No parent found for node " << QString::fromStdString(robot_name);
            return last_state;
        }
        if( are_different(std::vector<float>{bState.x, bState.z, bState.alpha},
                          std::vector<float>{last_state.x, last_state.z, last_state.alpha},
                          std::vector<float>{1, 1, 0.1}))
        {
            auto edge = G->get_edge_RT(parent.value(), robot->id()).value();
            G->modify_attrib_local<rotation_euler_xyz_att>(edge, std::vector<float>{0., bState.alpha, 0.});
            G->modify_attrib_local<translation_att>(edge, std::vector<float>{bState.x, 0., bState.z});
            G->modify_attrib_local<linear_speed_att>(edge, std::vector<float>{bState.advVx, 0, bState.advVz});
            G->modify_attrib_local<angular_speed_att>(edge, std::vector<float>{0, bState.rotV, 0});
            G->insert_or_assign_edge(edge);
            last_state = bState;
            return bState;
        }
    }
    else
        std::this_thread::yield();
    return last_state;
}

void SpecificWorker::update_pantilt_position()
{
    static std::vector<float> last_state{0.0, 0.0};
    static std::vector<float> epsilon{0.1, 0.1};

    if (auto jointmotors_o = jointmotor_buffer.try_get(); jointmotors_o.has_value())
    {
        const float pan = jointmotors_o.value().at(viriato_head_camera_pan_joint).pos;
        const float tilt = jointmotors_o.value().at(viriato_head_camera_tilt_joint).pos;
        //qInfo() << pan << tilt;
        if( are_different(std::vector<float>{pan, tilt}, last_state, epsilon))
        {
            auto pan_tilt = G->get_node(viriato_head_camera_pan_tilt);
            auto tilt_joint = G->get_node(viriato_head_camera_tilt_joint);
            auto pan_joint = G->get_node(viriato_head_camera_pan_joint);
            if(pan_tilt.has_value() and pan_joint.has_value())
            {
                G->insert_or_assign_edge_RT(pan_tilt.value(), pan_joint->id(), std::vector<float>{0.0, 0.0, 0.0},
                                            std::vector<float>{0.0, pan, 0.0});
                G->insert_or_assign_edge_RT(pan_joint.value(), tilt_joint->id(), std::vector<float>{0.0, 0.0, 0.0},
                                            std::vector<float>{-tilt, 0.0, 0.0});
            }
            else
                qWarning() << __FILE__ << __FUNCTION__ << "No nodes pan_joint or tilt_joint found";
        }
    }
}

// Check if rotation_speed or advance_speed have changed and move the robot consequently
void SpecificWorker::check_new_base_command(const RoboCompGenericBase::TBaseState& bState)
{
    auto robot = G->get_node(this->robot_name); //any omnirobot
    if (not robot.has_value())
    {
        qWarning() << __FUNCTION__ << " No node " <<  QString::fromStdString(this->robot_name);
        return;
    }
    auto ref_adv_speed = G->get_attrib_by_name<ref_adv_speed_att>(robot.value());
    auto ref_rot_speed = G->get_attrib_by_name<ref_rot_speed_att>(robot.value());
    auto ref_side_speed = G->get_attrib_by_name<ref_side_speed_att>(robot.value());
    if(not ref_adv_speed.has_value() or not ref_rot_speed.has_value() or not ref_side_speed.has_value())
    {
        qWarning() << __FUNCTION__ << " No valid attributes for robot speed";
        return;
    }
    // Check de values are within robot's accepted range. Read them from config
    //if(fabs(ref_adv_speed.value())>0 or fabs(ref_rot_speed.value())>0 or fabs(ref_side_speed.value())>0)
    //const float lowerA = -10, upperA = 10, lowerR = -10, upperR = 5, lowerS = -10, upperS = 10;
    //std::clamp(ref_adv_speed.value(), lowerA, upperA);
    //std::clamp(ref_side_speed.value(), lowerS, upperS);
    //std::clamp(ref_rot_speed.value(), lowerR, upperR);

    if( are_different(std::vector<float>{bState.advVz, bState.rotV, bState.advVx},
                      std::vector<float>{ref_adv_speed.value(), ref_rot_speed.value(), ref_side_speed.value() },
                      std::vector<float>{1, 0.1, 1}));
    {
        qDebug() << __FUNCTION__ << "Diff detected" << ref_adv_speed.value() << bState.advVz << ref_rot_speed.value() << bState.rotV << ref_side_speed.value() << bState.advVx;
        try
        {
                omnirobot_proxy->setSpeedBase(0, ref_adv_speed.value(), ref_rot_speed.value());

                //                std::cout << __FUNCTION__ << "Adv: " << ref_adv_speed.value() << " Side: " << ref_side_speed.value()
                //                          << " Rot: " << ref_rot_speed.value()
                //                          << " " << bState.advVz << " " << bState.advVx << " " << bState.rotV
                //                          << " " << (ref_adv_speed.value() - bState.advVz) << " "
                //                          << (ref_side_speed.value() - bState.advVx) << " " << (ref_rot_speed.value() - bState.rotV)
                //                          << std::endl;
        }
        catch(const RoboCompGenericBase::HardwareFailedException &re)
        { std::cout << re << '\n';}
        catch(const Ice::Exception &e)
        { std::cout << e.what() << '\n';}
    }
}

void SpecificWorker::check_new_dummy_values_for_coppelia()
{
    static float current_base_target_x = 0;
    static float current_base_target_y = 0;
    if( auto robot = G->get_node(robot_name); robot.has_value())
    {
        auto x = G->get_attrib_by_name<base_target_x_att>(robot.value());
        auto y = G->get_attrib_by_name<base_target_y_att>(robot.value());
        if( x.has_value() and y.has_value())
            if(x.value() != current_base_target_x or y.value() != current_base_target_y)
            {
                RoboCompCoppeliaUtils::PoseType dummy_pose{x.value(), 0.1, y.value(), 0.0, 0.0, 0.0};
                try
                { coppeliautils_proxy->addOrModifyDummy( RoboCompCoppeliaUtils::TargetTypes::Info, "base_dummy", dummy_pose); }
                catch (const Ice::Exception &e)
                { std::cout << e << " Could not communicate through the CoppeliaUtils interface" << std::endl; }
                current_base_target_x = x.value();
                current_base_target_y = y.value();
            }
    }
}

void SpecificWorker::check_new_nose_referece_for_pan_tilt()
{
    // zero position of nose
    static std::vector<float> ant_nose_target{10.0, 0.0, 0.0};
    if( auto pan_tilt = G->get_node(viriato_head_camera_pan_tilt); pan_tilt.has_value())
    {
        auto target = G->get_attrib_by_name<viriato_head_pan_tilt_nose_target>(pan_tilt.value());
        //if (target.has_value() and are_different(target.value(), ant_nose_target, std::vector<float>{1, 1, 1}))
        //{
            // convert target to world reference
            RoboCompCoppeliaUtils::PoseType dummy_pose{ target.value().get()[0], target.value().get()[1], target.value().get()[2], 0.0, 0.0, 0.0};
            try
            { coppeliautils_proxy->addOrModifyDummy( RoboCompCoppeliaUtils::TargetTypes::HeadCamera, nose_target, dummy_pose); }
            catch (const Ice::Exception &e)
            { std::cout << e << " Could not communicate through the CoppeliaUtils interface" << std::endl; }
            ant_nose_target = target.value();
        //}
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////
bool SpecificWorker::are_different(const std::vector<float> &a, const std::vector<float> &b, const std::vector<float> &epsilon)
{
    for(auto &&[aa, bb, e] : iter::zip(a, b, epsilon))
        if(fabs(aa-bb) > e)
            return true;
    return false;
};
////////////////////////////////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

/////////////////////////////////////////////////////////////////////
/// SUBSCRIPTION to pushRGBD method from CameraRGBDSimplePub interface
//////////////////////////////////////////////////////////////////////
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

void SpecificWorker::JointMotorPub_motorStates(RoboCompJointMotor::MotorStateMap mstateMap)
{
    jointmotor_buffer.put(std::move(mstateMap));
}