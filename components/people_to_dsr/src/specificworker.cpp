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
{}

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
    agent_id = stoi(params["agent_id"].value);
    dsr_output_path = params["dsr_output_path"].value;
    agent_name = params["agent_name"].value;
    return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;

    G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, "", dsrgetid_proxy); // Init nodes
    innermodel = G->get_inner_api();

    // GraphViewer creation
    using opts = DSR::GraphViewer::view;
	graph_viewer = std::make_unique<DSR::GraphViewer>(this, G, opts::graph|opts::osg);
    
    this->Period = 100;
    timer.start(Period);
    
}

void SpecificWorker::compute()
{
    if(auto pdata = people_data_buffer.get(); pdata.has_value())
        process_people_data(pdata.value());
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
        if (G_person_id.find(person.id) != G_person_id.end()) {
            G_id = G_person_id[person.id];
        }
        std::string person_name = "person [" + std::to_string(person.id) + "]";
        std::optional<Node> person_n = G->get_node(G_id);
        if(person_n.has_value()) //update edges
        {
            //qDebug()<<"update person:"<<person_n->id();
            std::vector<float> trans{person.x, person.y, person.z};
            std::vector<float> rot{0, person.ry, 0};
            G->insert_or_assign_edge_RT(world_n.value(), person_n->id(), trans, rot);
            //std::cout << "Update RT "<<world_n->id()<<" ("<<trans[0]<<","<<trans[1]<<","<<trans[2]<<")("<<rot[0]<<","<<rot[1]<<","<<rot[2]<<std::endl;
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
                    auto world_to_parent = innermodel->getTransformationMatrixS(parent_name, "world");
                    
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
            //qDebug()<<"Person does not exist => Creation";
            person_n = create_node("person", person_name, person.id, world_n->id());
            if (not person_n.has_value()) 
                return;
            G->insert_or_assign_edge_RT(world_n.value(), person_n->id(), std::vector<float>{person.x, person.y, person.z}, std::vector<float>{0.0, 0.0, 0.0});
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



//SUBSCRIPTION to newPeopleData method from HumanToDSRPub interface
void SpecificWorker::HumanToDSRPub_newPeopleData(RoboCompHumanToDSRPub::PeopleData people)
{
    qDebug() << "received RoboCompHumanToDSRPub::PeopleData " << people.peoplelist.size();
    people_data_buffer.put(std::move(people));
}
/*
int SpecificWorker::get_new_node_id()
{
    int new_id = -1;
    try{
        new_id = dsrgetid_proxy->getID();    
    }
    catch(const std::exception& e)
    {
        std::cerr << "Error asking for new node id " << e.what() << '\n';
    }
    return new_id;
}*/

std::optional<Node> SpecificWorker::create_node(std::string type, std::string name, int person_id,  int parent_id)
{

    Node node;
    node.type(type);
    node.agent_id(agent_id);
    node.name(name);
    G->add_or_modify_attrib_local(node, "pos_x", 100.0);
    G->add_or_modify_attrib_local(node, "pos_y", 100.0);
    G->add_or_modify_attrib_local(node, "name", name);
    G->add_or_modify_attrib_local(node, "color", std::string("GoldenRod"));
    G->add_or_modify_attrib_local(node, "path", std::string("/home/robocomp/robocomp/"));
    G->add_or_modify_attrib_local(node, "parent", G->get_node("world").value().id());
    G->add_or_modify_attrib_local(node, "level", G->get_node_level(G->get_node_root().value()).value() + 1);
    
    std::optional<int> new_id = G->insert_node(node);

    if(new_id.has_value()) {
        qDebug() << __FUNCTION__ << "Create node: ID " << new_id.value();
        //We have to keep an equivalence between detector and graph ids.
        G_person_id[person_id] = new_id.value();
        return node;
    }
    else
        qDebug() << __FUNCTION__ << "Error on node "<<QString::fromStdString(node.name())<<"creation";
    return {};
}
