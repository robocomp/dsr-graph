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
#include <QFileDialog>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx), startup_check_flag(startup_check)
{
    //this->startup_check_flag = startup_check;
	QLoggingCategory::setFilterRules("*.debug=false\n");
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	qDebug() << "Destroying SpecificWorker" ;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	try
	{
		agent_name = params["agent_name"].value;
    	agent_id = stoi(params["agent_id"].value);
    	dsr_input_file = params["dsr_input_file"].value;
   	 	dsr_output_path = params["dsr_output_path"].value;
		dsr_write_to_file = params["dsr_write_to_file"].value == "true";
    	this->Period = stoi(params["period"].value);
		tree_view = (params["tree_view"].value == "true") ? DSR::DSRViewer::view::tree : 0;
		graph_view = (params["graph_view"].value == "true") ? DSR::DSRViewer::view::graph : 0;
		qscene_2d_view = (params["2d_view"].value == "true") ? DSR::DSRViewer::view::scene : 0;
		osg_3d_view = (params["3d_view"].value == "true") ? DSR::DSRViewer::view::osg : 0;
	}
	catch(const std::exception &e) { qFatal("Error reading config params"); }
	return true;
}

void SpecificWorker::initialize(int period)
{
	qDebug() << "Initialize worker" ;
    this->Period = period;
    if(this->startup_check_flag)
    {
        this->startup_check();
    }
    else
    {
        // create graph
        G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, dsr_input_file);
        std::cout << __FUNCTION__ << "Graph loaded" << std::endl;
        //check RT tree
        G->print();
        if (auto g = G->get_node_root(); g.has_value())
            check_rt_tree(G->get_node_root().value());
        else
        {
            std::cout << "G nor created. Aborting" << std::endl;
            std::terminate();
        }

        // Graph viewer
        using opts = DSR::DSRViewer::view;
		int current_opts = tree_view | graph_view | qscene_2d_view | osg_3d_view;
        opts main = opts::none;
        if (graph_view)
        	main = opts::graph;
		dsr_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
		setWindowTitle(QString::fromStdString(agent_name + "-" + dsr_input_file));

        // Ignore attributes from G
        //G->set_ignored_attributes<cam_rgb_att, cam_depth_att, laser_angles_att, laser_dists_att>();

        // self agent api
        agent_info_api = std::make_unique<DSR::AgentInfoAPI>(G.get());

		this->Period = 200;
        // Compute max Id in G
        get_max_id_from_G();
        if (dsr_write_to_file)
            timer.start(Period);
    }
}

void SpecificWorker::compute()
{
    G->write_to_json_file(dsr_output_path + agent_name + "_" + std::to_string(output_file_count) + ".json");
    output_file_count++;
}

void SpecificWorker::get_max_id_from_G()
{
	node_id = 0;
	for (const auto &[key, node] : G->getCopy())
        if (node.id() > node_id)
            node_id = node.id();
    qInfo() << "MAX ID from file:" << node_id;
}
//Check RT tree is well built
void SpecificWorker::check_rt_tree(const DSR::Node &node)
{
    std::optional<std::map<std::pair<uint64_t, std::string>, DSR::Edge>> edges_map = G->get_edges(node.id());
    if (edges_map.has_value()) {
        for (const auto&[pair, edge] : edges_map.value())
        {
            if(edge.type() == "RT")
            {
                std::cout << __FUNCTION__  << "edge from " << edge.from() << " to " << edge.to() << std::endl;
                std::optional<DSR::Node> child = G->get_node(edge.to());
                assertm(child.has_value() == true, "Edge without child node");
                std::optional<std::uint64_t> parent_id = G->get_parent_id(child.value());
                assertm(parent_id.has_value() == true, "Edge without parent node");
                assertm(node.id() == parent_id.value(), "Inconsistency between edge and node parent");
                std::optional<std::uint32_t> child_level = G->get_node_level(child.value());
                assertm(child_level.has_value() == true, "Child without level attribute");
                std::optional<std::uint32_t> node_level = G->get_node_level(node);
                assertm(node_level.has_value() == true, "Parent without level attribute");
                assertm(child_level.value() == (node_level.value() + 1),
                        "Inconsistency between parent and child levels");
                check_rt_tree(child.value());
            }
        }
    }
}



//////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}


////////////////////////////////////////////////////////
//// IMPLEMENTS SECTION
///////////////////////////////////////////////////////

int SpecificWorker::DSRGetID_getID()
{
	QMutexLocker locker(mutex);
	node_id++;
	std::cout << "Request served with id: " << node_id << std::endl;
	return node_id;
}


