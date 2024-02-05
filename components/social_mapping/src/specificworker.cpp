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
	//G->write_to_json_file("./"+agent_name+".json");
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
    intimate_cost_value = stof(params["intimate_cost_value"].value);
    personal_cost_value = stof(params["personal_cost_value"].value);
    social_cost_value = stof(params["social_cost_value"].value);
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

        // 2D widget
        widget_2d = qobject_cast<DSR::QScene2dViewer *>(graph_viewer->get_widget(opts::scene));
        if (widget_2d != nullptr)
            widget_2d->set_draw_laser(false);

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

        // read personal_spaces from G id they exists
        if( auto grid_node = G->get_node(current_grid_name); grid_node.has_value())
        {
            auto personal_spaces_nodes = G->get_nodes_by_type(personal_space_type_name);
            auto affordance_spaces_nodes = G->get_nodes_by_type(affordance_space_type_name);
            if(not personal_spaces_nodes.empty() or not affordance_spaces_nodes.empty())
                space_nodes_buffer.put(std::make_tuple(personal_spaces_nodes, affordance_spaces_nodes));
        }

        // read grid from G id it exists
        if( auto grid_node = G->get_node(current_grid_name); grid_node.has_value())
        {
            if (auto grid_as_string = G->get_attrib_by_name<grid_as_string_att>(grid_node.value()); grid_as_string.has_value())
                grid_buffer.put(std::string{grid_as_string.value().get()});
        }
        qInfo() << __FUNCTION__ << "SIZE " << grid.size();

		this->Period = period;
		timer.start(Period);
	}
}

void SpecificWorker::compute()
{   
    if(!loaded_grid){
        if (auto grid_node = G->get_node(current_grid_name); grid_node.has_value())
        {
            if (const auto grid_as_string = G->get_attrib_by_name<grid_as_string_att>(
                        grid_node.value()); grid_as_string.has_value())
            {
                grid.readFromString(grid_as_string.value());
                loaded_grid = true;
                std::cout << "---------LOADED GRID--------" << std::endl;

            }
        }
        else
        {
            loaded_grid=false;
            std::cout << "---------NO GRID--------" << std::endl;
        }
    }
    else
    {
        if (auto space_nodes = space_nodes_buffer.try_get(); space_nodes.has_value()) {
            // std::cout << "SPACE NODES"<< std::endl;
            const auto spaces = get_polylines_from_dsr(space_nodes.value());
            last_spaces = spaces;
            spaces_saved = true;
            insert_polylines_in_grid(spaces);
            inject_grid_in_G();
        }
        else if (auto space_nod = G->get_nodes_by_type(personal_space_type_name); space_nod.size()==0)
        {
            // std::cout << "NO SPACE NODES"<< std::endl;
            if(spaces_saved){
                vector<QPolygonF> clear_polygon;
                last_spaces = std::make_tuple(clear_polygon, clear_polygon, clear_polygon, get<3>(last_spaces));
                insert_polylines_in_grid(last_spaces);
                inject_grid_in_G();
            }
        }
    }
    fps.print("FPS: ", [this](auto x){ graph_viewer->set_external_hz(x);});
}

//void SpecificWorker::compute()
//{
//    if( auto space_nodes = space_nodes_buffer.try_get(); space_nodes.has_value())
//    {
//        const auto spaces = get_polylines_from_dsr(space_nodes.value());
//        if (auto grid_node = G->get_node(current_grid_name); grid_node.has_value())
//        {
//            if (const auto grid_as_string = G->get_attrib_by_name<grid_as_string_att>(grid_node.value()); grid_as_string.has_value())
//            {
//                grid.readFromString(grid_as_string.value());
//                insert_polylines_in_grid(spaces);
//                inject_grid_in_G();
//            }
//            else
//            {}
//        } else
//        {}
//    }
/*    if( auto grid_string = grid_buffer.try_get(); grid_string.has_value())
    {
        auto personal_spaces_nodes = G->get_nodes_by_type(personal_space_type_name);
        auto affordance_spaces_nodes = G->get_nodes_by_type(affordance_space_type_name);
        const auto spaces = get_polylines_from_dsr(std::make_tuple(personal_spaces_nodes, affordance_spaces_nodes));
        if(not personal_spaces_nodes.empty() or not affordance_spaces_nodes.empty())
        {
            std::string compressed_data = gzip::compress(grid_string.value().data(), grid_string.value().size());
            qInfo() << compressed_data.size() << " " << grid_string.value().size();
            std::string decompressed_data = gzip::decompress(compressed_data.data(), compressed_data.size());

            //grid.readFromString(grid_string.value());
            grid.readFromString(decompressed_data);
            insert_polylines_in_grid(spaces);
            inject_grid_in_G();
        }
        else
        {}  // grid has not changed. Use current one.
    }*/
//}

////////////////////////////////////////////////////////////////////////////

SpecificWorker::Spaces SpecificWorker::get_polylines_from_dsr(const std::tuple<std::vector<DSR::Node>,std::vector<DSR::Node>> &space_nodes)
{
    const auto &[personal_spaces_nodes, affordance_spaces_nodes] = space_nodes;
    Space intimate_seq, personal_seq, social_seq, affordances_seq;

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

    for(auto node: affordance_spaces_nodes)
    {
        QPolygonF affordance_pol;
        auto aff_x = G->get_attrib_by_name<aff_x_pos_att>(node).value().get();
        auto aff_y = G->get_attrib_by_name<aff_y_pos_att>(node).value().get();
        for(auto &&[point_x,point_y] : iter::zip(aff_x, aff_y))
        {
            affordance_pol.push_back(QPointF(point_x,point_y));
        }
        affordances_seq.push_back(affordance_pol);
    }
    return std::make_tuple(intimate_seq, personal_seq, social_seq, affordances_seq);
}
void SpecificWorker::insert_polylines_in_grid(const Spaces &spaces)
{
    const auto &[intimate_seq, personal_seq, social_seq, affordances_seq] = spaces;
    grid.resetGrid();
    for (auto &&poly_aff : affordances_seq)
        grid.modifyCostInGrid(poly_aff, 2.0);

    //To set occupied

    for (auto &&poly_soc : social_seq)
        grid.modifyCostInGrid(poly_soc, social_cost_value);

    for (auto &&poly_per : personal_seq)
        grid.modifyCostInGrid(poly_per, personal_cost_value);

    for (auto &&poly_intimate : iter::chain(intimate_seq))
        /*grid.markAreaInGridAs(poly_intimate, false);*/grid.modifyCostInGrid(poly_intimate, intimate_cost_value);

  //  if (widget_2d != nullptr)
  //      grid.draw(&widget_2d->scene);
}

void SpecificWorker::empty_polylines_in_grid(const Spaces &spaces)
{
    const auto &[intimate_seq, personal_seq, social_seq, affordances_seq] = spaces;
    grid.resetGrid();
    for (auto &&poly_aff : affordances_seq)
        grid.modifyCostInGrid(poly_aff, 2.0);

    //To set occupied

    for (auto &&poly_soc : social_seq)
        grid.modifyCostInGrid(poly_soc, social_cost_value);

    for (auto &&poly_per : personal_seq)
        grid.modifyCostInGrid(poly_per, personal_cost_value);

    for (auto &&poly_intimate : iter::chain(intimate_seq))
        /*grid.markAreaInGridAs(poly_intimate, false);*/grid.modifyCostInGrid(poly_intimate, intimate_cost_value);

   // if (widget_2d != nullptr)
   //     grid.draw(&widget_2d->scene);
}

void SpecificWorker::inject_grid_in_G()
{
    std::string grid_as_string = grid.saveToString();
    if (auto current_grid_node_o = G->get_node(current_grid_name); current_grid_node_o.has_value())
    {
        G->add_or_modify_attrib_local<grid_as_string_att>(current_grid_node_o.value(), grid_as_string);
        G->update_node(current_grid_node_o.value());
    }
    else
        std::cout << __FILE__ << __FUNCTION__ << " No grid node in G. Ignoring personal spaces" << std::endl;

    // Creo que si no hay nodo grid no se debe crear aquí

//    {
//        if (auto mind = G->get_node(robot_mind_name); mind.has_value())
//        {
//            DSR::Node current_grid_node = DSR::Node::create<grid_node_type>(social_grid_type_name);
//            G->add_or_modify_attrib_local<parent_att>(current_grid_node, mind.value().id());
//            G->add_or_modify_attrib_local<level_att>(current_grid_node, G->get_node_level(mind.value()).value() + 1);
//            G->add_or_modify_attrib_local<grid_as_string_att>(current_grid_node, grid_as_string);
//            G->add_or_modify_attrib_local<pos_x_att>(current_grid_node, (float) -262);
//            G->add_or_modify_attrib_local<pos_y_att>(current_grid_node, (float) 91);
//            if (std::optional<int> current_grid_node_id = G->insert_node(current_grid_node); current_grid_node_id.has_value())
//            {
//                DSR::Edge edge = DSR::Edge::create<has_edge_type>(mind.value().id(), current_grid_node.id());
//                if (G->insert_or_assign_edge(edge) == false)
//                    std::cout << __FILE__ << __FUNCTION__ << " Fatal error inserting new edge: " << mind.value().id() << "->" << current_grid_node_id.value()
//                              << " type: has" << std::endl;
//            }
//            else
//                std::cout << __FILE__ << __FUNCTION__ << " Fatal error inserting_new 'grid' node" << std::endl;
//        }
//    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////
/// Asynchronous changes on G nodes from G signals
//////////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::add_or_assign_node_slot(const std::uint64_t id, const std::string &type)
{
    if (type == grid_type_name)  // grid
    {
        if (auto node = G->get_node(id); node.has_value())
        {
            if (const auto grid_as_string = G->get_attrib_by_name<grid_as_string_att>(node.value()); grid_as_string.has_value())
                grid_buffer.put(std::string{grid_as_string.value().get()});
        }
    }
     if (type==personal_space_type_name)
     {
         auto personal_spaces_nodes = G->get_nodes_by_type(personal_space_type_name);
         auto affordance_spaces_nodes = G->get_nodes_by_type(affordance_space_type_name);
         space_nodes_buffer.put(std::make_tuple(personal_spaces_nodes, affordance_spaces_nodes));
     }
}

//////////////////////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}
