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
    G->write_to_json_file(dsr_output_path+agent_name+".json");
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
    G->start_subscription_thread(true);     // regular subscription to deltas
    G->start_fullgraph_request_thread();    // for agents that want to request the graph for

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
//computeCODE
//QMutexLocker locker(mutex);

}




//SUBSCRIPTION to newPeopleData method from HumanToDSR interface
void SpecificWorker::HumanToDSR_newPeopleData(PeopleData people)
{
    std::optional<Node> world_n = G->get_node("world");
    if(not world_n.has_value())
        return;
    
    for(const RoboCompHumanToDSR::Person &person: people.peoplelist)
    {
        std::string person_name = "person [" + std::to_string(person.id) + "]";
        std::optional<Node> person_n = G->get_node(person_name);
qDebug()<<person_n->id();        
        if(person_n.has_value()) //update edges
        {
            qDebug()<<"Person" << QString::fromStdString(person_name) << "update";
            std::optional<Edge> edge_p_w = G->get_edge(world_n->id(), person_n->id(), "RT");
            if(edge_p_w.has_value())
            {
                std::vector<float> values{person.x, person.y, 5000.0};//person.z};
                G->add_attrib(edge_p_w->attrs(), "trans", values);
                G->insert_or_assign_edge(edge_p_w.value());
            }
            for(const auto &[name, key] : person.joints) //update joints edge values
            {
                std::string node_name = name + " [" + std::to_string(person.id) + "]";
                std::optional<Node> joint_n = G->get_node(node_name);
                if(joint_n.has_value())
                {
                    auto edge_p_j = G->get_edge(person_n->id(), joint_n->id(), "RT");
                    if(edge_p_j.has_value())
                    {
                        std::vector<float> values{key.x, key.y, key.z};
                        G->add_attrib(edge_p_j->attrs(), "trans", values);
                        G->insert_or_assign_edge(edge_p_j.value());
                    }
                }
            }
        }
        else //create nodes
        {
            qDebug()<<"Person does not exist => Creation";
            int person_id = create_node("person", person_name);
            if (person_id == -1 ) 
                return;
            create_rt_edge(world_n->id(), person_id, std::vector<float>{person.x, person.y, person.z});
            //create joints nodes
            for(std::string name : COCO_IDS)
            {
                std::string node_name = name + " [" + std::to_string(person_id) + "]";
                int node_id = create_node("joint", node_name);
                create_rt_edge(person_id, node_id, std::vector<float>{0.0, 0.0, 0.0});
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

int SpecificWorker::create_node(std::string type, std::string name)
{
    int id = get_new_node_id();
    if (id == -1)
       return -1;
    Node node;
    node.type(type);
    node.id(id);
    node.agent_id(agent_id);
    node.name(name);
    G->add_attrib(node.attrs(), "pos_x", 100);
    G->add_attrib(node.attrs(), "pos_y", 100);
    G->add_attrib(node.attrs(), "name", name);
    G->add_attrib(node.attrs(), "color", std::string("GoldenRod"));
    if( G->insert_or_assign_node(node))
        return id; 
    return -1;
}

bool SpecificWorker::create_rt_edge(int from, int to, std::vector<float> values)
{
    Edge edge_rt;
    edge_rt.type("RT");
    edge_rt.from(from);
    edge_rt.to(to);
    G->add_attrib(edge_rt.attrs(), "translation", values);
    
    return G->insert_or_assign_edge(edge_rt);
}
