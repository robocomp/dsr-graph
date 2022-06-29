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
        eliminado = false;
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

}

void SpecificWorker::compute()
{

	if(auto intention = G->get_node(current_intention_name); intention.has_value())
    {
        std::optional<std::string> plan = G->get_attrib_by_name<current_intention_att>(intention.value());

        /// GOTOPERSON
        if (plan.value().find("goToPerson") != std::string::npos){
            qInfo() << __FUNCTION__ << QString::fromStdString(plan.value());
            qInfo() << __FUNCTION__ << "ENCONTRADOOOO";

            //char[] caracteres = plan.value().toCharArray();

            auto plan_string = QString::fromStdString(plan.value());
            qInfo() << __FUNCTION__ << plan_string << " PLLAAAAAAAAAAAN";
            QString buscar = "goToPerson";
            QString buscar2 = "@";
            qInfo() << "La posición de goToPerson es: "<< plan_string.indexOf(buscar);
            qInfo() << "La posición de @ es: "<< plan_string.indexOf(buscar2);

            // create ACTION edge to destination node (to be used instead of textual Plan)
            bool exists_edge = false;
            if( auto target_room_edges = G->get_node_edges_by_type(G->get_node(robot_name).value(), "is_near"); not target_room_edges.empty())
            {
                for (const auto &tr_edge : target_room_edges)
                    if (tr_edge.to() == 900)    //  found goto edge to the target room
                        exists_edge = true;
                    else
                        G->delete_edge(tr_edge.from(), tr_edge.to(), "is_near");   // found got edge to other room. Deleted
            }
            if(not exists_edge)  // not found edge to target room
            {
                // create edge from intention node to the target room node
                DSR::Edge target_room_edge = DSR::Edge::create<is_near_edge_type>(G->get_node(robot_name).value().id(), 900);
                if (G->insert_or_assign_edge(target_room_edge))
                    std::cout << __FUNCTION__ << " Edge \"is_near_type\" inserted in G" << std::endl;
                else
                    std::cout << __FILE__ << __FUNCTION__ << " Fatal error inserting new edge: " << G->get_node(robot_name).value().id() << "->" << 900
                              << " type: is_near" << std::endl;
            }

            eliminado = true;

        }

        /// TAKETHEATTENTION
        if (plan.value().find("takeTheAttention") != std::string::npos){
            qInfo() << __FUNCTION__ << QString::fromStdString(plan.value());
            qInfo() << __FUNCTION__ << "ENCONTRADOOOO";

            // create ACTION edge to destination node (to be used instead of textual Plan)
            bool exists_edge = false;
            if( auto target_room_edges = G->get_node_edges_by_type(G->get_node("person_0").value(), "front"); not target_room_edges.empty())
            {
                for (const auto &tr_edge : target_room_edges)
                    if (tr_edge.to() == 200)    //  found goto edge to the target room
                        exists_edge = true;
                    else
                        G->delete_edge(tr_edge.from(), tr_edge.to(), "front");   // found got edge to other room. Deleted
            }
            if(not exists_edge)  // not found edge to target room
            {
                // create edge from intention node to the target room node
                DSR::Edge target_room_edge = DSR::Edge::create<front_edge_type>(G->get_node("person_0").value().id(), 200);
                if (G->insert_or_assign_edge(target_room_edge))
                    std::cout << __FUNCTION__ << " Edge \"front\" inserted in G" << std::endl;
                else
                    std::cout << __FILE__ << __FUNCTION__ << " Fatal error inserting new edge: " << G->get_node("person_0").value().id() << "->" << 200
                              << " type: front" << std::endl;
            }

            eliminado = true;

        }

        /// ASKFORINDIVIDUALPERMISSION
        if (plan.value().find("askForIndividualPermission") != std::string::npos){
            qInfo() << __FUNCTION__ << QString::fromStdString(plan.value());
            qInfo() << __FUNCTION__ << "ENCONTRADOOOO";

            // create ACTION edge to destination node (to be used instead of textual Plan)

            if( auto target_room_edges = G->get_node_edges_by_type(G->get_node("person_0").value(), "front"); not target_room_edges.empty())
            {
                for (const auto &tr_edge : target_room_edges)
                    if (tr_edge.to() == 200)    //  found goto edge to the target room
                        G->delete_edge(tr_edge.from(), tr_edge.to(), "front");   // found got edge to other room. Deleted


            }

            if( auto target_room_edges = G->get_node_edges_by_type(G->get_node("person_0").value(), "is_blocking"); not target_room_edges.empty())
            {
                for (const auto &tr_edge : target_room_edges)
                    if (tr_edge.to() == 200)    //  found goto edge to the target room
                        G->delete_edge(tr_edge.from(), tr_edge.to(), "is_blocking");   // found got edge to other room. Deleted


            }

            eliminado = true;

        }

        /// CHHANGEROOM
        if (plan.value().find("changeRoom") != std::string::npos){
            qInfo() << __FUNCTION__ << QString::fromStdString(plan.value());
            qInfo() << __FUNCTION__ << "ENCONTRADOOOO";

            // create ACTION edge to destination node (to be used instead of textual Plan)
            bool exists_edge = false;
            if( auto target_room_edges = G->get_node_edges_by_type(G->get_node(robot_name).value(), "in"); not target_room_edges.empty())
            {
                for (const auto &tr_edge : target_room_edges)
                    if (tr_edge.to() == 50)    //  found goto edge to the target room
                        G->delete_edge(tr_edge.from(), tr_edge.to(), "in");   // found got edge to other room. Deleted
            }
            if( auto target_room_edges = G->get_node_edges_by_type(G->get_node(robot_name).value(), "in"); not target_room_edges.empty())
            {
                for (const auto &tr_edge : target_room_edges)
                    if (tr_edge.to() == 60)    //  found goto edge to the target room
                        exists_edge = true;
                    else
                        G->delete_edge(tr_edge.from(), tr_edge.to(), "in");   // found got edge to other room. Deleted
            }
            if(not exists_edge)  // not found edge to target room
            {
                // create edge from intention node to the target room node
                DSR::Edge target_room_edge = DSR::Edge::create<in_edge_type>(G->get_node(robot_name).value().id(), 60);
                if (G->insert_or_assign_edge(target_room_edge))
                    std::cout << __FUNCTION__ << " Edge \"in\" inserted in G" << std::endl;
                else
                    std::cout << __FILE__ << __FUNCTION__ << " Fatal error inserting new edge: " << G->get_node(robot_name).value().id() << "->" << 60
                              << " type: in" << std::endl;
            }
            eliminado = true;


        }

        if(eliminado){

            if(auto intention = G->get_node(current_intention_name); intention.has_value())
            {
                if(auto path = G->get_node(current_path_name); path.has_value())
                    G->delete_node(path.value().id());
                G->delete_node(intention.value().id());
            }
            else
                qWarning() << __FUNCTION__ << "No intention node found";
            eliminado = false;
        }

    }
	
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}




