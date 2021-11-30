/*
 *    Copyright (C) 2021 by YOUR NAME HERE
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
    QLoggingCategory::setFilterRules("*.debug=false\n");
	this->startup_check_flag = startup_check;
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
		G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, ""); // Init nodes
		std::cout<< __FUNCTION__ << "Graph loaded" << std::endl;  

		//dsr update signals
		connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::modify_node_slot);
		connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::modify_edge_slot);
		connect(G.get(), &DSR::DSRGraph::update_node_attr_signal, this, &SpecificWorker::modify_attrs_slot);
		connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
		connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);

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

        // Inner Api
        inner_eigen = G->get_inner_eigen_api();

        // self agent api
        agent_info_api = std::make_unique<DSR::AgentInfoAPI>(G.get());

        // human model
        delete_person_model();
        create_person_model();

		this->Period = period;
		timer.start(Period);
	}
}

void SpecificWorker::compute()
{


}
/////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::create_person_model()
{
    //    person -> body -> { head | trunk}
    //    head -> { face | nape }
    //    face -> {eyes | mouth | ears | nose}
    //    trunk -> { arms | hip | legs }
    //    arms-> { shoulder | elbow | wrist | hand }
    //    hand -> fingers
    //    leg -> { knee | foot}

    // add person to world
    add_node<person_node_type>("person_1", world_name);
    add_node<body_node_type>("body_1", "person_1");
    add_node<head_node_type>("head_1", "body_1");
    add_node<trunk_node_type>("trunk_1", "body_1");
    add_node<face_node_type>("face_1", "head_1");
    add_node<nape_node_type>("nape_1", "head_1");
    add_node<left_arm_node_type>("left_arm_1", "trunk_1");
    add_node<right_arm_node_type>("right_arm_1", "trunk_1");
    add_node<hip_node_type>("hip_1", "trunk_1");
    add_node<left_leg_node_type>("left_leg_1", "hip_1");
    add_node<right_leg_node_type>("right_leg_1", "hip_1");

}
void SpecificWorker::delete_person_model()
{
    // go from leaves up
    G->delete_node("right_leg_1");
    G->delete_node("left_leg_1");
    G->delete_node("right_arm_1");
    G->delete_node("left_arm_1");
    G->delete_node("hip_1");
    G->delete_node("trunk_1");
    G->delete_node("face_1");
    G->delete_node("nape_1");
    G->delete_node("head_1");
    G->delete_node("body_1");
    G->delete_node("person_1");
    qInfo() << __FUNCTION__ << "Person 1  deleted";
}

////
std::tuple<float, float> SpecificWorker::get_position_by_level_in_graph(const DSR::Node &parent)
{
    auto children = G->get_node_edges_by_type(parent, "RT");
    std::vector<float> x_values;
    for(const auto child : children)
    {
        x_values.push_back(G->get_attrib_by_name<pos_x_att>(G->get_node(child.to()).value()).value());
        qInfo() << __FUNCTION__ << x_values.back();
    }
    float max = G->get_attrib_by_name<pos_x_att>(parent).value() - 300;
    if(not x_values.empty())
        max = std::ranges::max(x_values);
    return std::make_tuple(max + 150 , G->get_attrib_by_name<pos_y_att>(parent).value() +  80);
}
std::tuple<float, float> SpecificWorker::get_random_position_to_draw_in_graph(const std::string &type)
{
    static std::random_device rd;
    static std::mt19937 mt(rd());

    float low_x_limit = -800, low_y_limit = -700, upper_x_limit = 800, upper_y_limit = 500;
    low_x_limit = -300;
    upper_x_limit = 0;
    low_y_limit = -300;
    upper_y_limit = 0;
    std::uniform_real_distribution<double> dist_x(low_x_limit, upper_x_limit);
    std::uniform_real_distribution<double> dist_y(low_y_limit, upper_y_limit);

    return std::make_tuple(dist_x(mt), dist_y(mt));
}
///////////////////////////////////////// TESTING ///////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}




