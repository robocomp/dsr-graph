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
    G.reset();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
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
        // create graph
        G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, "", dsrgetid_proxy); // Init nodes
        std::cout << __FUNCTION__ << "Graph loaded" << std::endl;
        rt = G->get_rt_api();
        G->set_ignored_attributes<cam_rgb_att, laser_dists_att, laser_angles_att, cam_depth_att>();

        // Graph viewer
        using opts = DSR::DSRViewer::view;
        int current_opts = 0;
        //opts main = opts::none;
        if (tree_view)
            current_opts = current_opts | opts::tree;
        if (graph_view)
            current_opts = current_opts | opts::graph;
        if (qscene_2d_view)
            current_opts = current_opts | opts::scene;
        if (osg_3d_view)
            current_opts = current_opts | opts::osg;
        dsr_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts);
        setWindowTitle(QString::fromStdString(agent_name + "-" + dsr_input_file));

        //Inner Api
        innermodel = G->get_inner_eigen_api();
        std::cout << "Initialize worker" << std::endl;

        this->Period = 100;
        timer.start(Period);
    }
}

void SpecificWorker::compute()
{
    /*try {
        auto pdata = people_data_buffer.get();
        process_people_data(pdata);
    } catch (...) {}
    */
    if(auto pdata = people_data_buffer.try_get(); pdata.has_value())
        process_people_data(pdata.value());
    //else std::this_thread::yield();
    //check people thas has not been seen
    check_unseen_people();
}

//////////////////////////////////////////////////////////////////////////////////////////////

void SpecificWorker::process_people_data(RoboCompHumanToDSRPub::PeopleData people)
{
    //qDebug()<<"PROCESS PEOPLEDATA";
    std::vector<float> zeros{0.0,0.0,0.0};
    std::optional<Node> world_n = G->get_node("world");
    if(not world_n.has_value())
        return;
    
    for(const RoboCompHumanToDSRPub::Person &person: people.peoplelist)
    {
        int G_id = -1;
        //We have to keep an equivalence between detector and graph ids.
        if (G_person_id.find(person.id) != G_person_id.end()) 
            G_id = G_person_id[person.id];
        std::string person_name = "person [" + std::to_string(person.id) + "]";
        std::optional<Node> person_n = G->get_node(G_id);
        if(person_n.has_value()) //update edges
        {
            qInfo() << __FUNCTION__ << " update person:" << person_n->id();
            std::vector<float> trans{person.x, person.y, person.z};
            std::vector<float> rot{0, person.ry, 0};
            rt->insert_or_assign_edge_RT(world_n.value(), person_n->id(), trans, rot);
            std::cout << __FUNCTION__  << " Update RT "<<world_n->id()<<" ("<<trans[0]<<","<<trans[1]<<","<<trans[2]<<")("<<rot[0]<<","<<rot[1]<<","<<rot[2]<<std::endl;
/*            for(const auto &[name, key] : person.joints) //update joints edge values
            {
                std::string node_name = name + " [" + std::to_string(person.id) + "]";
                std::optional<Node> joint_n = G->get_node(node_name);
                std::string parent_name = jointMap[name].parent_name + " [" + std::to_string(person.id) + "]";;
                std::optional<Node> parent_n = G->get_node(parent_name);
                if(joint_n.has_value() and parent_n.has_value())
                {
                    QVec joint_to_personIN = QVec::vec3(key.px, key.py, key.pz);
                
std::cout<<"Update RT "<<name<<" "<<parent_name<<std::endl;                    
                    RTMat world_to_joint( 0.0, 0.0, 0.0, key.wx, key.wy, key.wz);
                    auto world_to_parent = innermodel->get_transformation_matrix(parent_name, "world");
                    
                    QVec joint_to_parentIA = (world_to_joint * world_to_parent.value() ).getTr();
                    std::vector<float> jtp{ joint_to_parentIA.x(), joint_to_parentIA.y(), joint_to_parentIA.z()};
                    
//                    std::cout<<node_name<<std::endl;
//                    joint_to_personIN.print("innermodel");
//                    joint_to_parentIA.print("inner_api");
                    
                    G->insert_or_assign_edge_RT(parent_n.value(), joint_n->id(), jtp, zeros);
                }
                else
                    qDebug()<<"node could not be reached"<< QString::fromStdString(node_name);
            }*/
        }
        else //create nodes
        {
            //qDebug() << __FUNCTION__ << "Person does not exist => Creation";
            person_n = create_node("person", person_name, person.id, world_n->id());
            if (not person_n.has_value()) 
                std::terminate();
            std::optional<Node> person_n_mesh = create_node_mesh(person_name, person1_path, person_n.value().id());
            if (not person_n_mesh.has_value()) 
                std::terminate();
            rt->insert_or_assign_edge_RT(world_n.value(), person_n->id(), std::vector<float>{person.x, person.y, person.z}, std::vector<float>{0.0, 0.0, 0.0});
            rt->insert_or_assign_edge_RT(person_n.value(), person_n_mesh->id(), std::vector<float>{0.0, 0.0, 0.0}, std::vector<float>{1.5796,0.0, 0.0});
            
            //create joints nodes
/*            for(std::string name : COCO_IDS)
            {
                std::string node_name = name + " [" + std::to_string(person.id) + "]";
                std::optional<Node> joint_n = create_node("joint", node_name, person_n->id());
            }
            //create edge with cannonical position
            G->insert_or_assign_edge_RT(world_n.value(), person_n->id(), zeros, zeros);
            for(std::string name : COCO_IDS)
            {
                JOINT_CONNECTION joint = jointMap[name];
                std::string parent_name = joint.parent_name + " [" + std::to_string(person.id) + "]";
                std::optional<Node> parent_n = G->get_node(parent_name);
                std::string node_name = name + " [" + std::to_string(person.id) + "]";
                std::optional<Node> joint_n = G->get_node(node_name);
                if(joint_n.has_value() and parent_n.has_value())
                    G->insert_or_assign_edge_RT(parent_n.value(), joint_n->id(), joint.translation, zeros);
                else
                    qDebug()<<"Error adding edge_RT from"<<QString::fromStdString(parent_name)<<"to"<<QString::fromStdString(node_name);
            }*/
        }
        //update timestamp
        people_last_seen[person_n.value().id()] = std::chrono::system_clock::now();
    }
    //qDebug()<<"PROCESS PEOPLEDATA END";
}

void SpecificWorker::check_unseen_people()
{
    auto now = std::chrono::system_clock::now();
	for (std::map<int,std::chrono::system_clock::time_point>::iterator it = people_last_seen.begin(); it != people_last_seen.end();)
	{
		auto last_view = std::chrono::duration_cast<std::chrono::milliseconds>(now - it->second).count();
		if(last_view > MAXTIME)
		{
			qDebug()<<"REMOVE PERSON"<<it->first;
			G->delete_node(it->first);
            // remove from equivalence map
            int node_person_id = it->first;
            std::unordered_map<int,int>::iterator it2 = std::find_if(G_person_id.begin(), G_person_id.end(),
						[&node_person_id](const std::pair<int, int> &p) {
							return p.second == node_person_id;
						});
            if(it2 != G_person_id.end())
                G_person_id.erase(it2);

            it = people_last_seen.erase(it);
 
            //TODO: remove joints when enabled
		}
		else
		{
			++it;
		}
	}
}

std::optional<Node> SpecificWorker::create_node(const std::string &type, const std::string &name, int person_id,  uint32_t parent_id)
{
    Node node;
    node.type(type);
    node.agent_id(agent_id);
    node.name(name);
    G->add_or_modify_attrib_local<pos_x_att>(node, 100.0f);
    G->add_or_modify_attrib_local<pos_y_att>(node,  100.0f);
    G->add_or_modify_attrib_local<name_att>(node, name);
    G->add_or_modify_attrib_local<color_att>(node, std::string("GoldenRod"));
    G->add_or_modify_attrib_local<parent_att>(node, parent_id);
    G->add_or_modify_attrib_local<level_att>(node, G->get_node_level(  G->get_node(parent_id).value()    ).value() + 1);
    //G->insert_or_assign_edge_RT(world_n.value(), person_n->id(), std::vector<float>{person.x, person.y, person.z}, std::vector<float>{0.0, 0.0, 0.0});
    try
    {
        std::optional<int> new_id = G->insert_node(node);
        if(new_id.has_value()) 
        {
            qDebug() << __FUNCTION__ << "Create node: ID " << new_id.value();
            //We have to keep an equivalence between detector and graph ids.
            G_person_id[person_id] = new_id.value();
            return node;
        }
        else
        {
            qDebug() << __FUNCTION__ << "insert_node returned no value for" << QString::fromStdString(node.name());
            return {};
        }
    }
    catch(const std::exception& e)
    {
        std::cout << __FUNCTION__ <<  e.what() << std::endl;
        std::terminate();
    }
}

std::optional<Node> SpecificWorker::create_node_mesh(const std::string &name, const std::string &path, uint32_t parent_id)
{
    Node node;
    node.type("mesh");
    node.name(name + "_mesh");
    G->add_or_modify_attrib_local<pos_x_att>(node,  100.0f);
    G->add_or_modify_attrib_local<pos_y_att>(node,  130.0f);
    G->add_or_modify_attrib_local<name_att>(node, name + "_mesh");
    G->add_or_modify_attrib_local<color_att>(node, std::string("GoldenRod"));
    G->add_or_modify_attrib_local<path_att>(node, path);
    G->add_or_modify_attrib_local<scalex_att>(node, 900);
    G->add_or_modify_attrib_local<scaley_att>(node, 900);
    G->add_or_modify_attrib_local<scalez_att>(node,  900);
    G->add_or_modify_attrib_local<parent_att>(node,  parent_id);
    G->add_or_modify_attrib_local<level_att>(node, G->get_node_level( G->get_node(parent_id).value()).value() + 1);
    try
    {     
        std::optional<int> new_id = G->insert_node(node);
        if(new_id.has_value())
            return node;
        else 
        {
            qDebug() << __FUNCTION__ << "insert_node returned no value for" << QString::fromStdString(node.name());
            return {};
        }
    }
    catch(const std::exception& e)
    {
        std::cout << __FUNCTION__ <<  e.what() << std::endl;
        std::terminate();
    }
}

int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}


/////////////////////////////////////////////////////////////////////
// SUBSCRIPTION to newPeopleData method from HumanToDSRPub interface
////////////////////////////////////////////////////////////////////

void SpecificWorker::HumanToDSRPub_newPeopleData(RoboCompHumanToDSRPub::PeopleData people)
{
    qDebug() << "received RoboCompHumanToDSRPub::PeopleData " << people.peoplelist.size();
    people_data_buffer.put(std::move(people));
}

