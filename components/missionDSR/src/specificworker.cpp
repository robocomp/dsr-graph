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
	//G->write_to_json_file("./"+agent_name+".json");
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
		timer.start(Period);
		// create graph
		G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, "", dsrgetid_proxy); // Init nodes
		std::cout<< __FUNCTION__ << "Graph loaded" << std::endl;

        // Ignore attributes from G
        G->set_ignored_attributes<cam_rgb_att, cam_depth_att, laser_dists_att, laser_angles_att>();

        // Graph viewer
		using opts = DSR::DSRViewer::view;
		int current_opts = 0;
		opts main = opts::none;
		if(tree_view)
		{
		    current_opts = current_opts | opts::tree;
		}
		if(graph_view)
		{
		    current_opts = current_opts | opts::graph;
		    main = opts::graph;
		}
		if(qscene_2d_view)
		{
		    current_opts = current_opts | opts::scene;
		}
		if(osg_3d_view)
		{
		    current_opts = current_opts | opts::osg;
		}
		graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
		setWindowTitle(QString::fromStdString(agent_name + "-") + QString::number(agent_id));

        // custom_widget
		graph_viewer->add_custom_widget_to_dock("Mission", &custom_widget);
        connect(custom_widget.set_pb,SIGNAL(clicked()),this, SLOT(set_mission_slot()));
        connect(custom_widget.delete_pb,SIGNAL(clicked()),this, SLOT(del_mission_slot()));
        //connect(G.get(), SIGNAL(update_node_signal), this, SLOT(update_mission_slot));
        connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::update_mission_slot);
        connect(custom_widget.mug_cb, SIGNAL(stateChanged(int)), this, SLOT(set_predefined_mission()));
        connect(custom_widget.floor_cb, SIGNAL(stateChanged(int)), this, SLOT(set_predefined_mission()));
        connect(custom_widget.none_cb, SIGNAL(stateChanged(int)), this, SLOT(set_predefined_mission()));
        
        
		this->Period = period;
		timer.start(Period);
	}
    initialize_object_list();
}

void SpecificWorker::initialize_object_list()
{
    auto map = G->getCopy();
	for(const auto &[k, node] : map)
    {
        custom_widget.object_cb->addItem(QString::fromStdString(node.name()));
    }
}

void SpecificWorker::compute()
{
	
}

////////////////////////////////////////
// UI slots
////////////////////////////////////////
void SpecificWorker::update_mission_slot(const std::int32_t id, const std::string &type)
{
    if (type == "intention")
    {
        auto node = G->get_node(id);
        if(auto parent = G->get_parent_node(node.value()); parent.has_value())
        {
            if (parent.value().name() == robot_name)
            {
                std::optional<std::string> plan = G->get_attrib_by_name<plan_att>(node.value());
                if (plan.has_value())
                {
                    custom_widget.mission_te->setText(QString::fromStdString(plan.value()));
                }
            }
        }
    }
}

//remove data from node intent
void SpecificWorker::del_mission_slot()
{
    auto node = this->get_intent_node();
    if (node.has_value())
        G->insert_or_assign_attrib<plan_att>(node.value(), std::string());
}

//add data from node intent
void SpecificWorker::set_mission_slot()
{
    if(custom_widget.mision_cb->currentText() == "Goto")
    {
        float x = custom_widget.x_pos_sb->value();
        float z = custom_widget.z_pos_sb->value();
        float alpha = custom_widget.alpha_pos_sb->value();
        std::string object_name = custom_widget.object_cb->currentText().toStdString();
        set_mission(x, z, alpha, object_name);
    }
    else
        qWarning() << __FUNCTION__ << "Only goto mission is defined yet";
}

void SpecificWorker::set_mission(float x, float z, float alpha, std::string object_name)
{
    QJsonObject action = this->goto_action_to_json(object_name, {x,z,alpha});
    QList<QJsonObject> actions;
    actions.push_back(action);
    std::string plan = generate_json_plan(actions);
    auto node = this->get_intent_node(true);
    G->insert_or_assign_attrib<plan_att>(node.value(), plan);
}


////////////////////////////////////////////////////////////////////////
/// get intent node from G
///////////////////////////////////////////////////////////////////////
std::optional<Node> SpecificWorker::get_intent_node(bool create)
{
    auto intent_nodes = G->get_nodes_by_type("intention");
    for (auto node : intent_nodes)
    {
        auto parent = G->get_parent_node(node);
        if(not parent.has_value())
            std::cout << __FILE__ << __FUNCTION__ << " Intent node without parent" << std::endl;
        
        if(parent.value().name() == robot_name)
            return node;
    }
    if(create)
    {
        //intent node not found => creation
        if(auto robot = G->get_node(robot_name); robot.has_value())
        {
            Node node;
            node.type(intention_type);
            G->add_or_modify_attrib_local<parent_att>(node,  robot.value().id());
            G->add_or_modify_attrib_local<level_att>(node, G->get_node_level(robot.value()).value() + 1);
            G->add_or_modify_attrib_local<pos_x_att>(node, (float)-90);
            G->add_or_modify_attrib_local<pos_y_att>(node, (float)-354);
            try
            {     
                std::optional<int> new_id = G->insert_node(node);
                if(new_id.has_value())
                {
                    Edge edge;
                    edge.type("has");
                    //get two ids
                    edge.from(robot.value().id());
                    edge.to(new_id.value());
                    if(not G->insert_or_assign_edge(edge))
                    {
                        std::cout<<"Error inserting new edge: "<<robot.value().id()<<"->"<<new_id.value()<<" type: has"<<std::endl;
                        std::terminate();
                    }
                    return node;
                }
                else 
                {
                    qWarning() << __FUNCTION__ << "insert_node returned no value for" << QString::fromStdString(node.name());
                    return {};
                }
            }
            catch(const std::exception& e)
            {
                std::cout << __FUNCTION__ <<  e.what() << std::endl;
                std::terminate();
            }
        }
        else
        {
            std::cout << __FILE__ << __FUNCTION__ << " No node robot found";
            std::terminate();
        }
    }
    else
        return {};
}


QJsonObject SpecificWorker::goto_action_to_json(std::string object_name, std::vector<float> location)
{
    QJsonObject actionObject;
    actionObject["action"] = "goto";
    //action params
    QJsonObject paramObject;
    paramObject["object"] = QString::fromStdString(object_name);
    QJsonArray vector;
    std::copy(location.begin(), location.end(), std::back_inserter(vector));
    paramObject["location"] = QJsonArray(vector);
    actionObject["params"] = paramObject;
    return actionObject;
}


std::string SpecificWorker::generate_json_plan(QList<QJsonObject> actions)
{
    QJsonObject jsonPlan;
    QJsonArray actionArray;

    //new action
    for(const QJsonObject actionObject : actions)
        actionArray.push_back(actionObject);

    jsonPlan["plan"] = actionArray;
    QJsonDocument doc(jsonPlan);
    QString strJson(doc.toJson(QJsonDocument::Compact));
    
    return strJson.toStdString();
}

void SpecificWorker::set_predefined_mission()
{
    if(custom_widget.none_cb->isChecked())
    {
        this->del_mission_slot();
    }
    else if (custom_widget.floor_cb->isChecked())
    {
        set_mission(1900.f, 669.f, 94.f, "infiniteFloor");
    }
    else if (custom_widget.mug_cb->isChecked())
    {
        set_mission(0.f, 0.f, 0.f, "glass_1");
    }
    
}

////////////////////////////////////////////////////////////////////////

int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}
