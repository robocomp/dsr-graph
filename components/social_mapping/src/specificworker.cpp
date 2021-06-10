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
    conf_params  = std::make_shared<RoboCompCommonBehavior::ParameterList>(params);
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
		connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::add_or_assign_node_slot);
//		connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::add_or_assign_edge_slot);
//		connect(G.get(), &DSR::DSRGraph::update_attrs_signal, this, &SpecificWorker::add_or_assign_attrs_slot);
//		connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
//		connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);

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

		//widget
        // 2D widget
        widget_2d = qobject_cast<DSR::QScene2dViewer *>(graph_viewer->get_widget(opts::scene));
        if (widget_2d != nullptr)
        {
            widget_2d->set_draw_laser(false);
//            connect(widget_2d, SIGNAL(mouse_right_click(int, int, std::uint64_t)), this, SLOT(new_target_from_mouse(int, int, std::uint64_t)));
        }
		//grid
        QRectF outerRegion;
        auto world_node = G->get_node(world_name).value();
        outerRegion.setLeft(G->get_attrib_by_name<OuterRegionLeft_att>(world_node).value());
        outerRegion.setRight(G->get_attrib_by_name<OuterRegionRight_att>(world_node).value());
        outerRegion.setBottom(G->get_attrib_by_name<OuterRegionBottom_att>(world_node).value());
        outerRegion.setTop(G->get_attrib_by_name<OuterRegionTop_att>(world_node).value());
        if(outerRegion.isNull())
        {
            qWarning() << __FILE__ << __FUNCTION__ << "Outer region of the scene could not be found in G. Aborting";
            std::terminate();
        }
        grid.dim.setCoords(outerRegion.left(), outerRegion.top(), outerRegion.right(), outerRegion.bottom());
        grid.TILE_SIZE = stoi(conf_params->at("tile_size").value);
//        collisions =  std::make_shared<Collisions>();
//        collisions->initialize(G, conf_params);
//        grid.initialize(G, collisions, false);
        if( auto grid_node = G->get_node("current_grid"); grid_node.has_value())
        {
            if (auto grid_as_string = G->get_attrib_by_name<grid_as_string_att>(grid_node.value()); grid_as_string.has_value()) {
                grid.readFromString(grid_as_string.value());
                grid_initialized = true;

            }
        }
        qInfo() << "SIZE " << grid.size();


		this->Period = period;
		timer.start(Period);
	}

}

void SpecificWorker::compute()
{
    if(personal_spaces_changed)
    {
        get_polylines_from_dsr();

        if(grid_initialized) {
            insert_polylines_in_grid();
            inject_grid_in_G(grid);
        }
        personal_spaces_changed = false;
    }
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}



void SpecificWorker::get_polylines_from_dsr(){
    auto personal_spaces_nodes = G->get_nodes_by_type("personal_space");
    auto affordance_spaces_nodes = G->get_nodes_by_type("affordance_space");
    intimate_seq.clear();
    personal_seq.clear();
    social_seq.clear();
    affordances_seq.clear();

    for (auto node: personal_spaces_nodes)
    {
        QPolygonF intimate_pol, personal_pol, social_pol;
        auto intimate_x = G->get_attrib_by_name<ps_intimate_x_pos_att>(node).value().get();
        auto intimate_y = G->get_attrib_by_name<ps_intimate_y_pos_att>(node).value().get();
        for(auto &&[point_x,point_y] : iter::zip(intimate_x, intimate_y)){
            intimate_pol.push_back(QPointF(point_x,point_y));
        }
        auto personal_x = G->get_attrib_by_name<ps_personal_x_pos_att>(node).value().get();
        auto personal_y = G->get_attrib_by_name<ps_personal_y_pos_att>(node).value().get();
        for(auto &&[point_x,point_y] : iter::zip(personal_x, personal_y)){
            personal_pol.push_back(QPointF(point_x,point_y));
        }

        auto social_x = G->get_attrib_by_name<ps_social_x_pos_att>(node).value().get();
        auto social_y = G->get_attrib_by_name<ps_social_y_pos_att>(node).value().get();
        for(auto &&[point_x,point_y] : iter::zip(social_x, social_y)){
            social_pol.push_back(QPointF(point_x,point_y));
        }

        intimate_seq.push_back(intimate_pol);
        personal_seq.push_back(personal_pol);
        social_seq.push_back(social_pol);
    }

    for(auto node: affordance_spaces_nodes){
        QPolygonF affordance_pol;
        auto aff_x = G->get_attrib_by_name<aff_x_pos_att>(node).value().get();
        auto aff_y = G->get_attrib_by_name<aff_y_pos_att>(node).value().get();
        for(auto &&[point_x,point_y] : iter::zip(aff_x, aff_y)){
            affordance_pol.push_back(QPointF(point_x,point_y));
        }
        affordances_seq.push_back(affordance_pol);
    }

}
void SpecificWorker::insert_polylines_in_grid() {

    grid.resetGrid();

    for (auto &&poly_soc : affordances_seq)
        grid.modifyCostInGrid(poly_soc, 2.0);

    //To set occupied
    for (auto &&poly_intimate : iter::chain(intimate_seq)) {
        grid.markAreaInGridAs(poly_intimate, false);
    }
    for (auto &&poly_per : social_seq)
        grid.modifyCostInGrid(poly_per, 10.0);

    for (auto &&poly_soc : personal_seq)
        grid.modifyCostInGrid(poly_soc, 8.0);




    if (widget_2d != nullptr)
        grid.draw(&widget_2d->scene);
}

void SpecificWorker::inject_grid_in_G(const Grid &grid)
{
    std::string grid_as_string = grid.saveToString();
    if (auto current_grid_node_o = G->get_node("social_grid"); current_grid_node_o.has_value())
    {
        G->add_or_modify_attrib_local<grid_as_string_att>(current_grid_node_o.value(), grid_as_string);
        G->update_node(current_grid_node_o.value());
    }
    else
    {
        if (auto mind = G->get_node(robot_mind_name); mind.has_value())
        {
            DSR::Node current_grid_node = DSR::Node::create<grid_node_type>("social_grid");
//            G->add_or_modify_attrib_local<name_att>(current_grid_node, "social_grid");
            G->add_or_modify_attrib_local<parent_att>(current_grid_node, mind.value().id());
            G->add_or_modify_attrib_local<level_att>(current_grid_node, G->get_node_level(mind.value()).value() + 1);
            G->add_or_modify_attrib_local<grid_as_string_att>(current_grid_node, grid_as_string);
            G->add_or_modify_attrib_local<pos_x_att>(current_grid_node, (float) -262);
            G->add_or_modify_attrib_local<pos_y_att>(current_grid_node, (float) 91);
            if (std::optional<int> current_grid_node_id = G->insert_node(current_grid_node); current_grid_node_id.has_value())
            {
                DSR::Edge edge = DSR::Edge::create<has_edge_type>(mind.value().id(), current_grid_node.id());
                if (G->insert_or_assign_edge(edge))
                    std::cout << __FUNCTION__ << "Edge successfully created between robot and grid" << std::endl;
                else
                {
                    std::cout << __FILE__ << __FUNCTION__ << " Fatal error inserting new edge: " << mind.value().id() << "->" << current_grid_node_id.value()
                              << " type: has" << std::endl;
                    std::terminate();
                }
            } else
            {
                std::cout << __FILE__ << __FUNCTION__ << " Fatal error inserting_new 'grid' node" << std::endl;
                std::terminate();
            }
        }
    }
}
///////////////////////////////////////////////////
/// Asynchronous changes on G nodes from G signals
///////////////////////////////////////////////////
void SpecificWorker::add_or_assign_node_slot(const std::uint64_t id, const std::string &type)
{
    cout<< __FUNCTION__ <<" "<< id<<" " << type <<endl;
    if (type == "grid")  // grid
    {
        if (auto node = G->get_node(id); node.has_value())
        {
            if (auto grid_as_string = G->get_attrib_by_name<grid_as_string_att>(node.value()); grid_as_string.has_value()){
                grid.readFromString(grid_as_string.value());
                grid_initialized = true;
            }
        }
    }

     if (type=="personal_space")
        personal_spaces_changed = true;

}