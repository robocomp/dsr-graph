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
#include <cppitertools/enumerate.hpp>

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
    try
    {
        agent_name = params.at("agent_name").value;
        agent_id = stoi(params.at("agent_id").value);
        tree_view = params.at("tree_view").value == "true";
        graph_view = params.at("graph_view").value == "true";
        qscene_2d_view = params.at("2d_view").value == "true";
        osg_3d_view = params.at("3d_view").value == "true";
    }
    catch(const std::exception &e){ std::cout << e.what() << " Error reading params from config file" << std::endl;};

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
		//connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::modify_node_slot);
		//connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::modify_edge_slot);
		connect(G.get(), &DSR::DSRGraph::update_node_attr_signal, this, &SpecificWorker::modify_node_attrs_slot);
        connect(G.get(), &DSR::DSRGraph::update_edge_attr_signal, this, &SpecificWorker::modify_edge_attrs_slot);
		//connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
		//connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);

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

        // Ignore attributes from G
        G->set_ignored_attributes<cam_rgb_att, cam_depth_att, laser_angles_att, laser_dists_att>();

        // self agent api
        agent_info_api = std::make_unique<DSR::AgentInfoAPI>(G.get());

        log_file.open("log_file.txt", std::ofstream::out | std::ofstream::trunc);
        if ( not log_file)
        {
            std::cout << __FUNCTION__ << "Error opening log_file.txt for writing" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        list_of_excluded_nodes.push_back("agent");

		this->Period = period;
		timer.start(Period);
	}
}

void SpecificWorker::compute()
{

}

/////////////////////////////////////////////////////////////////////////////////////////
std::string SpecificWorker::visitor(DSR::ValType &var, unsigned int vector_elements_to_write)
{
    auto vector_to_str = [vector_elements_to_write]<typename T>(const T &vec)
            {  stringstream  str; str << "[";
               for(auto &&[i, elem]: vec | iter::enumerate)
               {
                   if (i == vector_elements_to_write) break;
                   str << std::to_string(elem) << ", ";
               }
                auto s = str.str();
                s.pop_back();s.pop_back();  // remove last comma
                s += "]";
                return s;
            };
    auto TypeSelector = Overload
        {
                [](std::basic_string<char> &a) { return std::string(a); },
                [](std::int32_t &a) { return std::to_string(a); },
                [](float &a) { return std::to_string(a); },
                [vector_to_str](std::vector<float> &a) { return vector_to_str(a);},
                [](bool a) { if(a) return std::string("True"); else return std::string("False"); },
                [vector_to_str](std::vector<uint8_t> &a) { return vector_to_str(a); },
                [](std::uint32_t &a) { return std::to_string(a); },
                [](std::uint64_t &a) { return std::to_string(a); },
                [](double &a) { return std::to_string(a); },
                [vector_to_str](std::vector<std::uint64_t> &a) { return vector_to_str(a); },
                [vector_to_str](std::array<float, 2> &a) { return vector_to_str(a); },
                [vector_to_str](std::array<float, 3> &a) { return vector_to_str(a); },
                [vector_to_str](std::array<float, 4> &a) { return vector_to_str(a); },
                [vector_to_str](std::array<float, 6> &a) { return vector_to_str(a); }
        };

    return std::visit(TypeSelector, var);
}
void SpecificWorker::modify_node_attrs_slot(std::uint64_t id, const std::vector<std::string>& att_names)
{
    if(const auto node_o = G->get_node(id); node_o.has_value())
    {
        auto node = node_o.value();
        if (auto res = std::ranges::find(list_of_excluded_nodes, node.type()); res != std::end(list_of_excluded_nodes))
            return;
        for(const auto &att_name : att_names)
        {
            stringstream str;
            if (auto att = node.attrs().find(att_name); att != node.attrs().end())
            {
                str << "NODE, " << node.name() << ", " << att_name << ", " << visitor(att->second.value(), 3) << ", " << att->second.timestamp() << std::endl;
                std::cout << str.str() << std::endl;
                log_file << str.str();
            }
        }
    }
};
void SpecificWorker::modify_edge_attrs_slot(std::uint64_t from, std::uint64_t to, const std::string &type, const std::vector<std::string>& att_names)
{
    if(const auto edge_o = G->get_edge(from, to, type); edge_o.has_value())
    {
        auto edge = edge_o.value();
        auto node_from = G->get_node(from).value().name();
        auto node_to = G->get_node(to).value().name();
        for(const auto att_name : att_names)
        {
            stringstream str;
            if (auto att = edge.attrs().find(att_name); att != edge.attrs().end())
            {
                str << "EDGE. " << type << ", " << node_from << ", " << node_to << ", " << att_name << ", " << visitor(att->second.value(), 3) << ", " << att->second.timestamp() << std::endl;
                std::cout << str.str() << std::endl;
                log_file << str.str();
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}




