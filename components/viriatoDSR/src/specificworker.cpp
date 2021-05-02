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
#include <utility>
#include <cppitertools/zip.hpp>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(std::move(tprx))
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
        // Create graph
        G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id); // Init nodes
        std::cout << __FUNCTION__ << " - Graph loaded" << std::endl;
        rt = G->get_rt_api();

        //Inner Api
        inner_eigen = G->get_inner_eigen_api();

        // dsr update signals
        connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::add_or_assign_node_slot);
        connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::add_or_assign_edge_slot);
        //connect(G.get(), &DSR::DSRGraph::update_attrs_signal, this, &SpecificWorker::add_or_assign_attrs_slot);
        //connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
        //connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);

        // Set pan-tilt target to nose
        if (auto pan_tilt = G->get_node(viriato_head_camera_pan_tilt); pan_tilt.has_value())
        {
            // camera coordinate system
            //const auto nose = inner_eigen->transform(world_name, Mat::Vector3d(0,1000,0),viriato_head_camera_pan_tilt).value();
            Eigen::Vector3f nose(1000, 0, 0);  // cam ref system is rotated
            G->add_or_modify_attrib_local<viriato_head_pan_tilt_nose_target_att>(pan_tilt.value(),
                                                                                 std::vector<float>{static_cast<float>(nose.x()), static_cast<float>(nose.y()),
                                                                                                    static_cast<float>(nose.z())});
            G->update_node(pan_tilt.value());
        }
        //Set base speed reference to 0
        if (auto robot = G->get_node(robot_name); robot.has_value())
        {
            G->insert_or_assign_attrib<robot_ref_adv_speed_att>(robot.value(), (float) 0);
            G->insert_or_assign_attrib<robot_ref_rot_speed_att>(robot.value(), (float) 0);
            G->insert_or_assign_attrib<robot_ref_side_speed_att>(robot.value(), (float) 0);
        }

        // Graph viewer
        using opts = DSR::DSRViewer::view;
        int current_opts = 0;
        opts main = opts::graph;
        if (tree_view)
            current_opts = current_opts | opts::tree;
        if (graph_view)
            current_opts = current_opts | opts::graph;
        if(qscene_2d_view)
            current_opts = current_opts | opts::scene;
        if(osg_3d_view)
            current_opts = current_opts | opts::osg;
        graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
        setWindowTitle(QString::fromStdString(agent_name + "-" + std::to_string(agent_id)));

        timer.start(100);
    }
}
void SpecificWorker::compute()
{
    update_laser();
    update_omirobot();
    update_rgbd();
    update_pantilt_position();
    update_arm_state();

}

/////////////////////////////////////////////////////////////////////////////////////////////////

void SpecificWorker::update_rgbd()
{
    const auto rgb_o = rgb_buffer.try_get();
    const  auto depth_o = depth_buffer.try_get();
    if (rgb_o.has_value() and depth_o.has_value())
    {
        const auto& rgb = rgb_o.value(); const auto& depth = depth_o.value();
        auto node = G->get_node(viriato_head_camera_name);
        if (node.has_value())
        {
            G->add_or_modify_attrib_local<cam_rgb_att>(node.value(), rgb.image);
            G->add_or_modify_attrib_local<cam_rgb_width_att>(node.value(), rgb.width);
            G->add_or_modify_attrib_local<cam_rgb_height_att>(node.value(), rgb.height);
            G->add_or_modify_attrib_local<cam_rgb_depth_att>(node.value(), rgb.depth);
            G->add_or_modify_attrib_local<cam_rgb_cameraID_att>(node.value(), rgb.cameraID);
            G->add_or_modify_attrib_local<cam_rgb_focalx_att>(node.value(), rgb.focalx);
            G->add_or_modify_attrib_local<cam_rgb_focaly_att>(node.value(), rgb.focaly);
            G->add_or_modify_attrib_local<cam_rgb_alivetime_att>(node.value(), rgb.alivetime);
            // depth
            G->add_or_modify_attrib_local<cam_depth_att>(node.value(), depth.depth);
            G->add_or_modify_attrib_local<cam_depth_width_att>(node.value(), depth.width);
            G->add_or_modify_attrib_local<cam_depth_height_att>(node.value(), depth.height);
            G->add_or_modify_attrib_local<cam_depth_focalx_att>(node.value(), depth.focalx);
            G->add_or_modify_attrib_local<cam_depth_focaly_att>(node.value(), depth.focaly);
            G->add_or_modify_attrib_local<cam_depth_cameraID_att>(node.value(), depth.cameraID);
            G->add_or_modify_attrib_local<cam_depthFactor_att>(node.value(), depth.depthFactor);
            G->add_or_modify_attrib_local<cam_depth_alivetime_att>(node.value(), depth.alivetime);
            G->update_node(node.value());
        }
    } //else std::this_thread::yield();
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
            G->add_or_modify_attrib_local<laser_dists_att>(node.value(), dists);
            G->add_or_modify_attrib_local<laser_angles_att>(node.value(), angles);
            G->update_node(node.value());
        }
    }
    //else std::this_thread::yield();
}
void SpecificWorker::update_omirobot()
{
    static RoboCompGenericBase::TBaseState last_state;
    if (auto bState_o = omnirobot_buffer.try_get(); bState_o.has_value())
    {
        const auto bState = bState_o.value();
        auto robot = G->get_node(robot_name);
        if (not robot.has_value())
            qWarning() << __FUNCTION__ << " No node " << QString::fromStdString(robot_name);
        auto parent = G->get_parent_node(robot.value());
        if (not parent.has_value())
            qWarning() << __FUNCTION__ << " No parent found for node " << QString::fromStdString(robot_name);
        if( are_different(std::vector<float>{bState.x, bState.z, bState.alpha},
                          std::vector<float>{last_state.x, last_state.z, last_state.alpha},
                          std::vector<float>{1, 1, 0.1}))
        {
            auto edge = rt->get_edge_RT(parent.value(), robot->id()).value();
            G->modify_attrib_local<rt_rotation_euler_xyz_att>(edge, std::vector<float>{0., 0, bState.alpha});
            G->modify_attrib_local<rt_translation_att>(edge, std::vector<float>{bState.x,  bState.z, 0.0});
            G->modify_attrib_local<robot_local_linear_velocity_att>(edge, std::vector<float>{bState.advVx, 0, bState.advVz});
            G->modify_attrib_local<robot_local_angular_velocity_att>(edge, std::vector<float>{0, 0, bState.rotV});
            G->insert_or_assign_edge(edge);
            last_state = bState;
        }
    }
}
void SpecificWorker::update_pantilt_position()
{
    static std::vector<float> last_state{0.0, 0.0};
    static std::vector<float> epsilon{0.01, 0.01};

    if (auto jointmotors_o = jointmotor_buffer.try_get(); jointmotors_o.has_value())
    {
        const float pan = jointmotors_o.value().at(viriato_head_camera_pan_joint).pos;
        const float tilt = jointmotors_o.value().at(viriato_head_camera_tilt_joint).pos;
        const std::vector<float> current_state{pan, tilt};
        //qInfo() << pan << tilt;
        if( are_different(current_state, last_state, epsilon))
        {
            auto pan_tilt = G->get_node(viriato_head_camera_pan_tilt);
            auto tilt_joint = G->get_node(viriato_head_camera_tilt_joint);
            auto pan_joint = G->get_node(viriato_head_camera_pan_joint);
            if(pan_tilt.has_value() and pan_joint.has_value())
            {
                rt->insert_or_assign_edge_RT(pan_tilt.value(), pan_joint->id(), std::vector<float>{0.0, 0.0, 0.0},
                                            std::vector<float>{0.0, 0.0, -pan});
                rt->insert_or_assign_edge_RT(pan_joint.value(), tilt_joint->id(), std::vector<float>{0.0, 0.0, 0.0},
                                            std::vector<float>{tilt, 0.0, 0.0});
            }
            else
                qWarning() << __FILE__ << __FUNCTION__ << "No nodes pan_joint or tilt_joint found";
        }
        last_state = current_state;
    }
}
void SpecificWorker::update_arm_state()
{
    if (auto arm = kinovaarm_buffer.try_get(); arm.has_value())
        if( auto robot = G->get_node(robot_name); robot.has_value())
            if( auto left_hand_tip_node = G->get_node(viriato_left_arm_tip_name); left_hand_tip_node.has_value())
                rt->insert_or_assign_edge_RT(robot.value(), left_hand_tip_node->id(),
                                            std::vector<float>{arm->x, arm->y, arm->z},
                                            std::vector<float>{arm->rx, arm->ry, arm->rz});
}
void SpecificWorker::check_base_dummy()
{
    if (auto t = base_target_buffer.try_get(); t.has_value())
    {
        const auto [tx, ty, ta] = t.value();
        const auto r_pose = inner_eigen->transform_axis(world_name, robot_name).value();
        const float ra = r_pose[5];
        float diff = ta - ra;
        if( fabs(ta - ra) > 0.02)
        {
            diff = std::clamp(diff, -1.f, 1.f);
            //float adv = MAX_ADV_ROBOT_SPEED * f(diff) * g(dist);
            try
            { omnirobot_proxy->setSpeedBase(0, 0, -diff*30); }
            catch (const Ice::Exception &e)
            { std::cout << e.what() << std::endl; }
        }
    }
}

//// CHANGE THESE ONES BY SIGNAL SLOTS
void SpecificWorker::check_new_nose_referece_for_pan_tilt()
{
    if( auto pan_tilt = G->get_node(viriato_head_camera_pan_tilt); pan_tilt.has_value())
    {
        if( auto target = G->get_attrib_by_name<viriato_head_pan_tilt_nose_target_att>(pan_tilt.value()); target.has_value())
        {
            RoboCompCoppeliaUtils::PoseType dummy_pose{ target.value().get()[0], target.value().get()[1], target.value().get()[2], 0.0, 0.0, 0.0};
            try
            { coppeliautils_proxy->addOrModifyDummy( RoboCompCoppeliaUtils::TargetTypes::HeadCamera, nose_target, dummy_pose); }
            catch (const Ice::Exception &e)
            { std::cout << e << " Could not communicate through the CoppeliaUtils interface" << std::endl; }
        }
    }
}

///////////////////////////////////////////////////////////////////
/// Asynchronous changes on G nodes from G signals
///////////////////////////////////////////////////////////////////
void SpecificWorker::add_or_assign_node_slot(const std::uint64_t id, const std::string &type)
{
    if (type == left_hand_type_name)
    {
        if (auto node = G->get_node(id); node.has_value() and node.value().name() == viriato_left_arm_tip_name)
            if (auto target = G->get_attrib_by_name<viriato_arm_tip_target_att>(node.value()); target.has_value())
            {
                auto &target_ = target.value().get();
                if (target_.size() == 6)
                {
                    RoboCompCoppeliaUtils::PoseType dummy_pose{target_[0], target_[1], target_[2],
                                                               target_[3], target_[4], target_[5]};
                    qInfo() << __FUNCTION__ << " Dummy hand pose: " << dummy_pose.x << dummy_pose.y << dummy_pose.z;
                    try
                    {
                        coppeliautils_proxy->addOrModifyDummy(RoboCompCoppeliaUtils::TargetTypes::Hand, arm_tip_target,
                                                              dummy_pose);
                    }
                    catch (const Ice::Exception &e)
                    { std::cout << e << " Could not communicate through the CoppeliaUtils interface" << std::endl; }
                }
            }
    }
    else if (type == omnirobot_type_name)
    {
        if (auto robot = G->get_node(robot_name); robot.has_value())
        {
            // speed
            auto ref_adv_speed = G->get_attrib_by_name<robot_ref_adv_speed_att>(robot.value());
            auto ref_rot_speed = G->get_attrib_by_name<robot_ref_rot_speed_att>(robot.value());
            auto ref_side_speed = G->get_attrib_by_name<robot_ref_side_speed_att>(robot.value());
            if (ref_adv_speed.has_value() and ref_rot_speed.has_value() and ref_side_speed.has_value())
            {
                // Check de values are within robot's accepted range. Read them from config
                //const float lowerA = -10, upperA = 10, lowerR = -10, upperR = 5, lowerS = -10, upperS = 10;
                //std::clamp(ref_adv_speed.value(), lowerA, upperA);
                std::cout << __FUNCTION__ << " " << ref_side_speed.value() << " "  << ref_adv_speed.value() << " "  << ref_rot_speed.value() << std::endl;
                try
                { omnirobot_proxy->setSpeedBase(ref_side_speed.value(), ref_adv_speed.value(), ref_rot_speed.value()); }
                catch (const RoboCompGenericBase::HardwareFailedException &re)
                { std::cout << "Exception setting base speed " << re << '\n'; }
                catch (const Ice::Exception &e)
                { std::cout << e.what() << '\n'; }
            }
        }
    }
    else if(type == pan_tilt_type_name)
    {
        if( auto pan_tilt = G->get_node(viriato_head_camera_pan_tilt); pan_tilt.has_value())
        {
            if( auto target = G->get_attrib_by_name<viriato_head_pan_tilt_nose_target_att>(pan_tilt.value()); target.has_value())
            {
                RoboCompCoppeliaUtils::PoseType dummy_pose{ target.value().get()[0], target.value().get()[1], target.value().get()[2], 0.0, 0.0, 0.0};
                //qInfo() << __FUNCTION__ << "PAN_TILT " << dummy_pose.x << dummy_pose.y << dummy_pose.z;
                try
                { coppeliautils_proxy->addOrModifyDummy( RoboCompCoppeliaUtils::TargetTypes::HeadCamera, nose_target, dummy_pose); }
                catch (const Ice::Exception &e)
                { std::cout << e << " Could not communicate through the CoppeliaUtils interface" << std::endl; }
            }
        }
    }
}

void SpecificWorker::add_or_assign_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type)
{
    if (type == RT_edge_type_str and to == G->get_node(robot_name).value().id())
    {
        auto edge = G->get_edge(from, to, "RT");
        const auto x_values_o = G->get_attrib_by_name<rt_translation_att>(edge.value());
        auto rooms = G->get_nodes_by_type(room_type_name);
        for( const auto &r : rooms)
        {
            auto polygon_x = G->get_attrib_by_name<delimiting_polygon_x_att>(r);
            auto polygon_y = G->get_attrib_by_name<delimiting_polygon_y_att>(r);
            if (polygon_x.has_value() and polygon_y.has_value())
            {
                QPolygonF pol;
                for (auto &&[px, py] : iter::zip(polygon_x.value().get(), polygon_y.value().get()))
                    pol << QPointF(px, py);
                if(pol.containsPoint(QPointF(x_values_o.value().get()[0], x_values_o.value().get()[1]), Qt::WindingFill))
                {
                    // modificar o crear arco entre robot y r
                    if( auto room_edges = G->get_node_edges_by_type(G->get_node(robot_name).value(), "in"); not room_edges.empty())
                    {  //
                        for(const auto &r_edge : room_edges)
                            if(r_edge.to() == r.id()) return;
                            else G->delete_edge(r_edge.from(), r_edge.to(), "in");
                    }

                    // crear
                    DSR::Edge new_room_edge = DSR::Edge::create<in_edge_type>(G->get_node(robot_name).value().id(), r.id());
                    if (G->insert_or_assign_edge(new_room_edge))
                        std::cout << __FUNCTION__ << " Edge \"has_type\" inserted in G" << std::endl;
                    else
                        std::cout << __FILE__ << __FUNCTION__ << " Fatal error inserting new edge: " << G->get_node(robot_name).value().id() << "->" << r.id()
                                  << " type: is_in" << std::endl;

                }
            }
        }
    }
}
///////////////////////////////////////////////////////////////////
bool SpecificWorker::are_different(const std::vector<float> &a, const std::vector<float> &b, const std::vector<float> &epsilon)
{
    for(auto &&[aa, bb, e] : iter::zip(a, b, epsilon))
        if (fabs(aa - bb) > e)
            return true;
    return false;
};
///////////////////////////////////////////////////////////////////
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
	omnirobot_buffer.put(RoboCompGenericBase::TBaseState{state});
}

void SpecificWorker::JointMotorPub_motorStates(RoboCompJointMotor::MotorStateMap mstateMap)
{
    jointmotor_buffer.put(std::move(mstateMap));
}

////////////////////////////////////////////////////////////////////////////
/// SUBSCRIPTION to newArmState method from KinovaArmPub interface
void SpecificWorker::KinovaArmPub_newArmState(RoboCompKinovaArmPub::TArmState armstate)
{
    kinovaarm_buffer.put(RoboCompKinovaArmPub::TArmState{armstate});
}


