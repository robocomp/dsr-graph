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
		G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, ""); // Init nodes
		std::cout<< __FUNCTION__ << "Graph loaded" << std::endl;  

		//dsr update signals
		connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::update_node_slot);
		connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::add_or_assign_edge_slot);
		connect(G.get(), &DSR::DSRGraph::update_attrs_signal, this, &SpecificWorker::add_or_assign_attrs_slot);
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
        //inner_eigen = G->get_inner_eigen_api();

        this->Period = period;
		timer.start(Period);
	}

}

void SpecificWorker::compute()
{
    // consts
    static const float MIN_DIST = 800.;
    static const float MAX_ADV = 800.;
    static const float MAX_ROT = 1.;

    auto laser_node = G->get_node(laser_name).value();
    const auto &angles = G->get_attrib_by_name<laser_angles_att>(laser_node).value().get();  // from -PI to PI
    auto left_limit = std::distance(angles.begin(), std::find_if(angles.begin(),angles.end(), [](auto &a){ return a > -M_PI/6;}));
    auto right_limit = std::distance(angles.begin(), std::find_if(angles.begin(),angles.end(), [](auto &a){ return a > M_PI/6;}));
    auto dists = G->get_attrib_by_name<laser_dists_att>(laser_node).value().get();
    auto min_dist = std::min(dists.begin() + left_limit, dists.begin() + right_limit);
    float adv = 0; float rot = 0; float side= 0;

    qInfo() << *min_dist;

    if(*min_dist < MIN_DIST)
    {
        adv = 0; side = 0; rot = MAX_ROT;
    }
    else
    {
        adv = MAX_ADV; side = 0; rot = 0;
    }

    auto robot_node = G->get_node(robot_name);
    G->add_or_modify_attrib_local<robot_ref_adv_speed_att>(robot_node.value(), adv);
    G->add_or_modify_attrib_local<robot_ref_rot_speed_att>(robot_node.value(), rot);
    G->add_or_modify_attrib_local<robot_ref_side_speed_att>(robot_node.value(), side);
    G->update_node(robot_node.value());
}

//////////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}




