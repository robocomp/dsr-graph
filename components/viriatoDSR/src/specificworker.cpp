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
#include <cppitertools/chunked.hpp>


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

        // inner Api
        inner_eigen = G->get_inner_eigen_api();

        // self agent api
        agent_info_api = std::make_unique<DSR::AgentInfoAPI>(G.get());

        // dsr update signals
        connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::add_or_assign_node_slot, Qt::QueuedConnection);
        //connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::add_or_assign_edge_slot);
        // connect(G.get(), &DSR::DSRGraph::update_attrs_signal, this, &SpecificWorker::add_or_assign_attrs_slot);
        connect(G.get(), &DSR::DSRGraph::update_node_attr_signal, this, &SpecificWorker::change_attrs_slot, Qt::QueuedConnection);
        //connect(G.get(), &DSR::DSRGraph::update_node_attr_signal, this, &SpecificWorker::change_attrs_slot);
        //connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
        //connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);

        // Set pan-tilt target to nose
        if (auto pan_tilt = G->get_node(viriato_head_camera_pan_tilt_name); pan_tilt.has_value())
        {
            Eigen::Vector3f nose(0, 1000, 0);
            G->add_or_modify_attrib_local<viriato_head_pan_tilt_nose_pos_ref_att>(pan_tilt.value(),
                                                                                 std::vector<float>{static_cast<float>(nose.x()), static_cast<float>(nose.y()),
                                                                                                    static_cast<float>(nose.z())});
            G->update_node(pan_tilt.value());
        }
        //Set base speed reference to 0
        if (auto robot = G->get_node(robot_name); robot.has_value())
        {
            G->add_or_modify_attrib_local<robot_ref_adv_speed_att>(robot.value(), (float) 0);
            G->add_or_modify_attrib_local<robot_ref_rot_speed_att>(robot.value(), (float) 0);
            G->add_or_modify_attrib_local<robot_ref_side_speed_att>(robot.value(), (float) 0);
            G->update_node(robot.value());
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

        // initialize room occupancy
        try
        {
            RoboCompGenericBase::TBaseState bState;
            omnirobot_proxy->getBaseState(bState);
            update_room_occupancy(bState.x, bState.z);
        }

        catch(const Ice::Exception &e)
        { std::cout << e.what() << std::endl; std::cout << " No connection to get robot state. Aborting " << std::endl; std::terminate();}
        timer.start(50);
    }
}
void SpecificWorker::compute()
{
    update_laser();
    update_omirobot_timed();
    update_rgbd();
    update_pantilt_position_timed();
    //update_pantilt_position();
    //update_arm_state();

    //check_new_nose_referece_for_pan_tilt();

    fps.print("FPS: ", [this](auto x){ graph_viewer->set_external_hz(x);});
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
    if (auto bState_o = omnirobot_buffer.try_get(); bState_o.has_value())
    {
        const auto bState = bState_o.value();
        if(auto robot = G->get_node(robot_name); robot.has_value())
        {
            if (auto parent = G->get_parent_node(robot.value()); parent.has_value())
            {
                auto edge = rt->get_edge_RT(parent.value(), robot->id()).value();
                G->modify_attrib_local<rt_rotation_euler_xyz_att>(edge, std::vector<float>{0., 0, bState.alpha});
                G->modify_attrib_local<rt_translation_att>(edge, std::vector<float>{bState.x, bState.z, 0.0});
                G->modify_attrib_local<robot_local_linear_velocity_att>(edge, std::vector<float>{bState.advVx, 0, bState.advVz});
                G->modify_attrib_local<robot_local_angular_velocity_att>(edge, std::vector<float>{0, 0, bState.rotV});

                G->insert_or_assign_edge(edge);
                update_room_occupancy(bState.x, bState.z);
            } else
                qWarning() << __FUNCTION__ << " No parent found for node " << QString::fromStdString(robot_name);
        }
        else
            qWarning() << __FUNCTION__ << " No robot node found " << QString::fromStdString(robot_name);
    }
}
void SpecificWorker::update_omirobot_timed()
{
    static const int MAX_PACK_BLOCKS = 20;
    static const int BLOCK_SIZE = 3;

    if (auto bState_o = omnirobot_buffer.try_get(); bState_o.has_value())
    {
        const auto bState = bState_o.value();
        if(auto robot = G->get_node(robot_name); robot.has_value())
        {
            if (auto parent = G->get_parent_node(robot.value()); parent.has_value())  // replace by get_parent_id(robot.value())
            {
                //rt->insert_or_assign_edge_RT(parent.value(), robot.value().id(), std::vector<float>{bState.x, bState.z, 0.f}, std::vector<float>{0.f, 0.f, bState.alpha});
                
                // initialize packs in case a new egde is created
                std::vector<float> tr_pack(BLOCK_SIZE * MAX_PACK_BLOCKS, 0.f);
                std::vector<float> rot_pack(BLOCK_SIZE * MAX_PACK_BLOCKS, 0.f);
                std::vector<std::uint64_t> time_stamps(MAX_PACK_BLOCKS, 0);
                int index = 0;  //head
                
                rot_pack[2] = bState.alpha; tr_pack[0] = bState.x; tr_pack[1] = bState.z;
                time_stamps[0] =  static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count());

                // add contents if there exists a RT edge
                if( auto edge = rt->get_edge_RT(parent.value(), robot->id()); edge.has_value())
                {
                    auto head_o = G->get_attrib_by_name<rt_head_index_att>(edge.value());
                    auto tstamps_o = G->get_attrib_by_name<rt_timestamps_att>(edge.value());
                    auto tr_pack_o = G->get_attrib_by_name<rt_translation_att>(edge.value());
                    auto rot_pack_o = G->get_attrib_by_name<rt_rotation_euler_xyz_att>(edge.value());
                    if (head_o.has_value() and tstamps_o.has_value() and tr_pack_o.has_value() and rot_pack_o.has_value())
                    {
                        auto timestamp_index = (int)(head_o.value()/BLOCK_SIZE+1) % MAX_PACK_BLOCKS;
                        index = timestamp_index * BLOCK_SIZE;
                        time_stamps = tstamps_o.value().get();
                        tr_pack = tr_pack_o.value().get();
                        rot_pack = rot_pack_o.value().get();
                        tr_pack[index] = bState.x;
                        tr_pack[index + 1] = bState.z;
                        tr_pack[index + 2] = 0.f;
                        rot_pack[index] = 0.f;
                        rot_pack[index + 1] = 0.f;
                        rot_pack[index + 2] = bState.alpha;
                        time_stamps[timestamp_index] = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::system_clock::now().time_since_epoch()).count());

//                        for(int i = 0; auto &&v : iter::chunked(tr_pack,3))
//                        {
//                            qInfo() << " head " << index << " - pan(" << i << "): " << v[0] << v[1] << v[2];
//                            qInfo() << " time " << time_stamps[i];
//                            i++;
//                        }
//                        qInfo() << "-------------------------";

                    }  // if there is an RT edge without timestamps, then add a new one starting time from now
                }

                DSR::Edge edge = DSR::Edge::create<RT_edge_type>(parent->id(), robot->id());
                G->add_or_modify_attrib_local<rt_translation_att>(edge, tr_pack);
                G->add_or_modify_attrib_local<rt_rotation_euler_xyz_att>(edge, rot_pack);
                G->add_or_modify_attrib_local<rt_timestamps_att>(edge, time_stamps);
                G->add_or_modify_attrib_local<rt_head_index_att>(edge, index);
                G->add_or_modify_attrib_local<rt_translation_velocity_att>(edge, std::vector<float>{bState.advVx, bState.advVz, 0.f});
                G->add_or_modify_attrib_local<rt_rotation_euler_xyz_velocity_att>(edge, std::vector<float>{0, 0, bState.rotV});
                G->insert_or_assign_edge(edge);

                update_room_occupancy(bState.x, bState.z);
            } else
                qWarning() << __FUNCTION__ << " No parent found for node " << QString::fromStdString(robot_name);
        }
        else
            qWarning() << __FUNCTION__ << " No robot node found " << QString::fromStdString(robot_name);
    }
}
void SpecificWorker::update_room_occupancy(float robot_x, float robot_y)
{
    auto rooms = G->get_nodes_by_type(room_type_name);
    auto robot = G->get_node(robot_name).value();
    for( const auto &room : rooms)
    {
        auto polygon_x = G->get_attrib_by_name<delimiting_polygon_x_att>(room);
        auto polygon_y = G->get_attrib_by_name<delimiting_polygon_y_att>(room);
        if (polygon_x.has_value() and polygon_y.has_value())
        {
            bool existing_edge = false;
            QPolygonF pol;
            for (auto &&[px, py] : iter::zip(polygon_x.value().get(), polygon_y.value().get())) pol << QPointF(px, py);
            if(pol.containsPoint(QPointF(robot_x, robot_y), Qt::OddEvenFill))
            {   // robot inside room. check edge exists and delete existing ones pointing to other rooms
                if( auto room_edges = G->get_node_edges_by_type(robot, "in"); not room_edges.empty())
                {  // if there is at least one edge IN going out from robot
                    for(const auto &r_edge : room_edges)
                        if(r_edge.to() != room.id())   // if edge does not go to current room delete
                            G->delete_edge(r_edge.from(), r_edge.to(), "in");
                        else existing_edge = true;
                }
                // create
                if(not existing_edge)
                {
                    DSR::Edge new_room_edge = DSR::Edge::create<in_edge_type>(G->get_node(robot_name).value().id(), room.id());
                    if (G->insert_or_assign_edge(new_room_edge))
                        std::cout << __FUNCTION__ << " Edge \"in_type\" inserted in G" << std::endl;
                    else
                        std::cout << __FILE__ << __FUNCTION__ << " Fatal error inserting new edge: " << G->get_node(robot_name).value().id() << "->"
                                  << room.id()
                                  << " type: is_in" << std::endl;
                }
            }
        }
    }
}
void SpecificWorker::update_pantilt_position_timed()
{
    static const int MAX_PACK_BLOCKS = 20;
    static const int BLOCK_SIZE = 3;

    if (auto jointmotors_o = jointmotor_buffer.try_get(); jointmotors_o.has_value())
    {
        const float pan = jointmotors_o.value().at(viriato_head_camera_pan_joint_name).pos;
        const float tilt = jointmotors_o.value().at(viriato_head_camera_tilt_joint_name).pos;
        const std::vector<float> current_state{pan, tilt};
        auto pan_tilt = G->get_node(viriato_head_camera_pan_tilt_name);
        auto tilt_joint = G->get_node(viriato_head_camera_tilt_joint_name);
        auto pan_joint = G->get_node(viriato_head_camera_pan_joint_name);
        if (pan_tilt.has_value() and pan_joint.has_value() and tilt_joint.has_value())
        {
            ///////////// PAN ///////////////////////////////////////////////////////////////////////////////////////////
            //rt->insert_or_assign_edge_RT(pan_tilt.value(), pan_joint->id(), std::vector<float>{0.0, 0.0, 0.0},
            //                             std::vector<float>{0.0, 0.0, -pan});

            std::vector<float> tr_pan_pack(3 * MAX_PACK_BLOCKS, 0.f);
            std::vector<float> rot_pan_pack(3 * MAX_PACK_BLOCKS, 0.f);
            std::vector<std::uint64_t> time_stamps_pan(MAX_PACK_BLOCKS, 0);
            int index_pan = 0;  //head
            
            rot_pan_pack[2] = -pan;
            time_stamps_pan[0] =  static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count());
            
            // replace contents if there exists a RT edge
            if( auto edge = rt->get_edge_RT(pan_tilt.value(), pan_joint->id()); edge.has_value())
            {
                auto head_o = G->get_attrib_by_name<rt_head_index_att>(edge.value());
                auto tstamps_o = G->get_attrib_by_name<rt_timestamps_att>(edge.value());
                auto rot_pack_o = G->get_attrib_by_name<rt_rotation_euler_xyz_att>(edge.value());
                if (head_o.has_value() and tstamps_o.has_value() and rot_pack_o.has_value())
                {
                    auto timestamp_index = (int)(head_o.value()/BLOCK_SIZE+1) % MAX_PACK_BLOCKS;
                    index_pan = timestamp_index * BLOCK_SIZE;
                    time_stamps_pan = tstamps_o.value().get();
                    rot_pan_pack = rot_pack_o.value().get();
                    rot_pan_pack[index_pan + 2] = -pan; // -rotation in Z
                    time_stamps_pan[timestamp_index] = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::system_clock::now().time_since_epoch()).count());

//                    for(int i = 0; auto &&v : iter::chunked(rot_pan_pack,3))
//                    {
//                        qInfo() << " head " << index_pan << " - pan(" << i << "): " << v[0] << v[1] << v[2];
//                        qInfo() << " time " << time_stamps_pan[i];
//                        i++;
//                    }
//                    qInfo() << "-------------------------";
                }
            }
            DSR::Edge edge_pan = DSR::Edge::create<RT_edge_type>(pan_tilt.value().id(), pan_joint->id());
            G->add_or_modify_attrib_local<rt_translation_att>(edge_pan, tr_pan_pack);
            G->add_or_modify_attrib_local<rt_rotation_euler_xyz_att>(edge_pan, rot_pan_pack);
            G->add_or_modify_attrib_local<rt_timestamps_att>(edge_pan, time_stamps_pan);
            G->add_or_modify_attrib_local<rt_head_index_att>(edge_pan, index_pan);
            G->insert_or_assign_edge(edge_pan);

            ///////////// TILT ///////////////////////////////////////////////////////////////////////////////////////////
            //rt->insert_or_assign_edge_RT(pan_joint.value(), tilt_joint->id(), std::vector<float>{0.0, 0.0, 0.0},
            //                             std::vector<float>{tilt, 0.0, 0.0});

            std::vector<float> tr_tilt_pack(3 * MAX_PACK_BLOCKS, 0.f);
            std::vector<float> rot_tilt_pack(3 * MAX_PACK_BLOCKS, 0.f);
            std::vector<std::uint64_t> time_stamps_tilt(MAX_PACK_BLOCKS, 0);
            int index_tilt = 0;  //head

            rot_tilt_pack[0] = tilt;
            time_stamps_tilt[0] =  static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count());

            // replace contents if there exists a RT edge
            if( auto edge = rt->get_edge_RT(pan_joint.value(), tilt_joint->id()); edge.has_value())
            {
                auto head_o = G->get_attrib_by_name<rt_head_index_att>(edge.value());
                auto tstamps_o = G->get_attrib_by_name<rt_timestamps_att>(edge.value());
                auto rot_pack_o = G->get_attrib_by_name<rt_rotation_euler_xyz_att>(edge.value());
                if (head_o.has_value() and tstamps_o.has_value() and rot_pack_o.has_value())
                {
                    auto timestamp_index = (int)(head_o.value()/BLOCK_SIZE+1) % MAX_PACK_BLOCKS;
                    index_tilt = timestamp_index * BLOCK_SIZE;
                    time_stamps_tilt = tstamps_o.value().get();
                    rot_tilt_pack = rot_pack_o.value().get();
                    rot_tilt_pack[index_tilt] = tilt;  // rotation in X
                    time_stamps_tilt[timestamp_index] = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::system_clock::now().time_since_epoch()).count());
                 }
            }
            DSR::Edge edge_tilt = DSR::Edge::create<RT_edge_type>(pan_joint.value().id(), tilt_joint->id());
            G->add_or_modify_attrib_local<rt_translation_att>(edge_tilt, tr_tilt_pack);
            G->add_or_modify_attrib_local<rt_rotation_euler_xyz_att>(edge_tilt, rot_tilt_pack);
            G->add_or_modify_attrib_local<rt_timestamps_att>(edge_tilt, time_stamps_tilt);
            G->add_or_modify_attrib_local<rt_head_index_att>(edge_tilt, index_tilt);
            G->insert_or_assign_edge(edge_tilt);
        } else
            qWarning() << __FILE__ << __FUNCTION__ << "No nodes pan_joint or tilt_joint found";
    }
}
void SpecificWorker::update_pantilt_position()
{
    static std::vector<float> last_state{0.0, 0.0};
    static std::vector<float> epsilon{0.01, 0.01};

    if (auto jointmotors_o = jointmotor_buffer.try_get(); jointmotors_o.has_value())
    {
        const float pan = jointmotors_o.value().at(viriato_head_camera_pan_joint_name).pos;
        const float tilt = jointmotors_o.value().at(viriato_head_camera_tilt_joint_name).pos;
        const std::vector<float> current_state{pan, tilt};
        //qInfo() << pan << tilt;
        if( are_different(current_state, last_state, epsilon))
        {
            auto pan_tilt = G->get_node(viriato_head_camera_pan_tilt_name);
            auto tilt_joint = G->get_node(viriato_head_camera_tilt_joint_name);
            auto pan_joint = G->get_node(viriato_head_camera_pan_joint_name);
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
    if( auto pan_tilt = G->get_node(viriato_head_camera_pan_tilt_name); pan_tilt.has_value())
    {
        if( auto target = G->get_attrib_by_name<viriato_head_pan_tilt_nose_pos_ref_att>(pan_tilt.value()); target.has_value())
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
    //qInfo() << __FUNCTION__  << id << QString::fromStdString(type);
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
                        coppeliautils_proxy->addOrModifyDummy(RoboCompCoppeliaUtils::TargetTypes::Hand, arm_tip_target, dummy_pose);
                    }
                    catch (const Ice::Exception &e)
                    { std::cout << e << " Could not communicate through the CoppeliaUtils interface" << std::endl; }
                }
            }
    }
    else if (type == omnirobot_type_name)   // pasar al SLOT the change attrib
    {
        //qInfo() << __FUNCTION__  << " Dentro " << id << QString::fromStdString(type);
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
}

void SpecificWorker::add_or_assign_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type)
{
//    if (type == RT_edge_type_str and to == G->get_node(robot_name).value().id())
//    {
//        auto edge = G->get_edge(from, to, "RT");
//        const auto x_values_o = G->get_attrib_by_name<rt_translation_att>(edge.value());
//        auto rooms = G->get_nodes_by_type(room_type_name);
//        auto robot = G->get_node(robot_name).value();
//        for( const auto &room : rooms)
//        {
//            auto polygon_x = G->get_attrib_by_name<delimiting_polygon_x_att>(room);
//            auto polygon_y = G->get_attrib_by_name<delimiting_polygon_y_att>(room);
//            if (polygon_x.has_value() and polygon_y.has_value())
//            {
//                bool existing_edge = false;
//                QPolygonF pol;
//                for (auto &&[px, py] : iter::zip(polygon_x.value().get(), polygon_y.value().get()))
//                    pol << QPointF(px, py);
//                if(pol.containsPoint(QPointF(x_values_o.value().get()[0], x_values_o.value().get()[1]), Qt::OddEvenFill))
//                {   // robot inside room. check edge exists and delete existing ones pointing to other rooms
//                    if( auto room_edges = G->get_node_edges_by_type(robot, "in"); not room_edges.empty())
//                    {  // if there is at least one edge IN going out from robot
//                        for(const auto &r_edge : room_edges)
//                            if(r_edge.to() != room.id())   // if edge does not go to current room delete
//                                G->delete_edge(r_edge.from(), r_edge.to(), "in");
//                            else existing_edge = true;
//                    }
//                    // create
//                    if(not existing_edge)
//                    {
//                        DSR::Edge new_room_edge = DSR::Edge::create<in_edge_type>(G->get_node(robot_name).value().id(), room.id());
//                        if (G->insert_or_assign_edge(new_room_edge))
//                            std::cout << __FUNCTION__ << " Edge \"has_type\" inserted in G" << std::endl;
//                        else
//                            std::cout << __FILE__ << __FUNCTION__ << " Fatal error inserting new edge: " << G->get_node(robot_name).value().id() << "->"
//                                      << room.id()
//                                      << " type: is_in" << std::endl;
//                    }
//                }
//            }
//        }
//    }
}
void SpecificWorker::change_attrs_slot(std::uint64_t id, const std::vector<std::string>& att_names)
{
    if (id == 206)
        if(const auto node = G->get_node(206); node.has_value())
        {
            if (auto att_name = std::ranges::find(att_names, "viriato_head_pan_tilt_nose_pos_ref"); att_name != std::end(att_names))
            {
                //const auto t_att = attribs.at("viriato_head_pan_tilt_nose_pos_ref");
                //std::vector<float> target = std::get<std::vector<float>>(t_att.value());
                std::vector<float> target = G->get_attrib_by_name<viriato_head_pan_tilt_nose_pos_ref_att>(node.value()).value().get();
                qInfo() << __FUNCTION__  << id << " Saccadic " <<  target[0] << target[1] << target[2];
                RoboCompCoppeliaUtils::PoseType dummy_pose{target[0], target[1], target[2], 0.0, 0.0, 0.0};
                try
                { coppeliautils_proxy->addOrModifyDummy(RoboCompCoppeliaUtils::TargetTypes::HeadCamera, nose_target, dummy_pose); }
                catch (const Ice::Exception &e)
                { std::cout << e << " Could not communicate through the CoppeliaUtils interface" << std::endl; }
            }
            if (auto att_name = std::ranges::find(att_names, "viriato_head_pan_tilt_nose_speed_ref"); att_name != std::end(att_names))
            {
                //const auto t_att = attribs.at("viriato_head_pan_tilt_nose_speed_ref");
                //std::vector<float> target = std::get<std::vector<float>>(t_att.value());
                std::vector<float> target = G->get_attrib_by_name<viriato_head_pan_tilt_nose_speed_ref_att>(node.value()).value().get();
                qInfo() << __FUNCTION__  << id << " Smooth " <<  target[0] << target[1] << target[2];
                RoboCompCoppeliaUtils::SpeedType dummy_speed{target[0], target[1], target[2], 0.0, 0.0, 0.0};
                try
                { coppeliautils_proxy->setDummySpeed(RoboCompCoppeliaUtils::TargetTypes::HeadCamera, nose_target, dummy_speed); }
                catch (const Ice::Exception &e)
                { std::cout << e << " Could not communicate through the CoppeliaUtils interface" << std::endl; }
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


