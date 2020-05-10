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

    G = std::make_shared<CRDT::CRDTGraph>(0, agent_name, agent_id, ""); // Init nodes
    innermodel = G->get_inner_api();

    // GraphViewer creation
    graph_viewer = std::make_unique<DSR::GraphViewer>(G);
	mainLayout.addWidget(graph_viewer.get());
	window.setLayout(&mainLayout);
	setCentralWidget(&window);
    
    this->Period = 100000;
    timer.start(Period);
}

void SpecificWorker::compute()
{
}

//SUBSCRIPTION to newPeopleData method from HumanToDSR interface
void SpecificWorker::HumanToDSR_newPeopleData(PeopleData people)
{
    std::vector<float> zeros{0.0,0.0,0.0};

    std::optional<Node> world_n = G->get_node("world");
    if(not world_n.has_value())
        return;
    
    for(const RoboCompHumanToDSR::Person &person: people.peoplelist)
    {
        std::string person_name = "person [" + std::to_string(person.id) + "]";
        std::optional<Node> person_n = G->get_node(person_name);
        if(person_n.has_value()) //update edges
        {
            qDebug()<<"update person:"<<person_n->id();
            std::vector<float> values{person.x, person.y, person.z};
            G->insert_or_assign_edge_RT(world_n.value(), person_n->id(), values, zeros);
            std::cout << "Update RT "<<world_n->id()<<" "<<values<<std::endl;
            for(const auto &[name, key] : person.joints) //update joints edge values
            {
                std::string node_name = name + " [" + std::to_string(person.id) + "]";
                std::optional<Node> joint_n = G->get_node(node_name);
                if(joint_n.has_value())
                {
                    std::vector<float> values{key.x, key.y, key.z};
                    
                    // RTMat world_to_joint( 0.0, 0.0, 0.0, key.x, key.y, key.z);
                    // auto world_to_person = innermodel->getTransformationMatrixS(person_name, "world");
                    // QVec joint_to_person = (world_to_joint * world_to_person.value()).getTr();
                    // std::vector<float> jtp{ joint_to_person.x(), joint_to_person.y(), joint_to_person.z()};
                    // G->insert_or_assign_edge_RT(person_n.value(), joint_n->id(), jtp, zeros);
                
                    G->insert_or_assign_edge_RT(person_n.value(), joint_n->id(), values, zeros);
                }
                else
                    qDebug()<<"node could not be reached"<< QString::fromStdString(node_name);
            }
        }
        else //create nodes
        {
            qDebug()<<"Person does not exist => Creation";
            std::optional<Node> person_n = create_node("person", person_name);
            if (not person_n.has_value()) 
                return;
            G->insert_or_assign_edge_RT(world_n.value(), person_n->id(), std::vector<float>{person.x, person.y, person.z}, std::vector<float>{0.0, 0.0, 0.0});
            //create joints nodes
            for(std::string name : COCO_IDS)
            {
                std::string node_name = name + " [" + std::to_string(person.id) + "]";
                std::optional<Node> joint_n = create_node("joint", node_name);
                if(joint_n.has_value())
                    G->insert_or_assign_edge_RT(person_n.value(), joint_n->id(), std::vector<float>{0.0, 0.0, 0.0}, std::vector<float>{0.0, 0.0, 0.0});
            }
        }
    }
}

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
}

std::optional<Node> SpecificWorker::create_node(std::string type, std::string name)
{
    int id = get_new_node_id();
    if (id == -1)
       return {};
    Node node;
    node.type(type);
    node.id(id);
    node.agent_id(agent_id);
    node.name(name);
    G->insert_or_assign_attrib_by_name(node, "pos_x", 100);
    G->insert_or_assign_attrib_by_name(node, "pos_y", 100);
    G->insert_or_assign_attrib_by_name(node, "name", name);
    G->insert_or_assign_attrib_by_name(node, "color", std::string("GoldenRod"));
    if( G->insert_or_assign_node(node))
        return node; 
    return {};
}
