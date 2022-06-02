/*
 *    Copyright (C) 2022 by YOUR NAME HERE
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
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }





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

		this->Period = period;
		timer.start(Period);
	}

	//std::vector<Node> nodos;
}

void SpecificWorker::compute()
{
	compute_testYolo();
}

void SpecificWorker::compute_testYolo()
{

	std::shared_ptr<DSR::RT_API> rt_api = G->get_rt_api();
	std::shared_ptr<DSR::InnerEigenAPI> inner_eigen = G->get_inner_eigen_api();

    // auto containers = G->get_nodes_by_type("container");
    //auto cup = G->get_node("cup");
    auto table1 = G->get_node("table1");
	auto table2 = G->get_node("table2");
	api_geom api_geom(G,rt_api,inner_eigen);
    //if(cup.has_value() && containers.size() > 0)
    // {
    //     for(auto&& container : containers)
    //     {
    //         //auto container_name = G->get_attrib_by_name<rt_translation_att>(edge2.value());
    //         std::cout << "Distance: " << container.name() << " " << api_geom::distance_between_objects(G,rt_api,cup.value(), container) << std::endl;
    //     }
    // }

    // if(cup.has_value() && table.has_value())
    //     api_geom::distance_object_parent(G,rt_api,cup.value(),table.value());

	auto transformwithoutpoints = inner_eigen->transform(table1.value().name(),table2.value().name());

	auto world = G->get_node(world_name).value();
	G->delete_edge(world.id(), table2.value().id(), "RT");
	G->update_node(world);

    float x = transformwithoutpoints.value().x();
    float y = transformwithoutpoints.value().y();
    float z = transformwithoutpoints.value().z();

	G->add_or_modify_attrib_local<parent_att>(table2.value(), table1.value().id());
    G->add_or_modify_attrib_local<level_att>(table2.value(), 2);
	rt_api->insert_or_assign_edge_RT(table1.value(),table2.value().id(),std::vector<float>{x, y, z},std::vector<float>{0., 0., 0.});

	G->update_node(table1.value());
	G->update_node(table2.value());
	G->update_node(world);
}


int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}




