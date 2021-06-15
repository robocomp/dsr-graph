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
#include <cppitertools/zip.hpp>
#include <cppitertools/sliding_window.hpp>
#include <execution>

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
		//connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::add_or_assign_node_slot);
		//connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::add_or_assign_edge_slot);
		//connect(G.get(), &DSR::DSRGraph::update_attrs_signal, this, &SpecificWorker::add_or_assign_attrs_slot);
		//connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
		//connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);

		//inner
        inner_eigen = G->get_inner_eigen_api();

        // ignore attributes
        G->set_ignored_attributes<cam_rgb_att, depth_att>();

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

        // widget 2d
        widget_2d = qobject_cast<DSR::QScene2dViewer*> (graph_viewer->get_widget(opts::scene));
        if(widget_2d != nullptr)
            widget_2d->set_draw_laser(true);

		this->Period = 100;
		timer.start(Period);
	}
}

void SpecificWorker::compute()
{
    // Comprobar si hay nodos de tipo personas
    if(auto personal_spaces = G->get_nodes_by_type("personal_space"); not personal_spaces.empty())
    {
        //Eigen::VectorXf all_gauss_x(personal_space.size() * 9), all_gauss_y(personal_space.size() * 9);
        auto laser_node= G->get_node(laser_name).value();
        if(const auto angles_o = G->get_attrib_by_name<laser_angles_att>(laser_node); angles_o.has_value())
        {
            const auto &angles = angles_o.value().get();
            if( const auto dist_o = G->get_attrib_by_name<laser_dists_att>(laser_node); dist_o.has_value())
            {
                const auto &dist = dist_o.value().get();
                // obtener_puntos_gausianas(personal_space, all_gauss_x, all_gauss_y);
                // modificar_laser(all_gauss_x, all_gauss_y, angles, dist);
                modify_laser(personal_spaces, angles, dist);
                update_social_laser(dist, angles);
            }
        }
    }
    fps.print("FPS: ", [this](auto x){ graph_viewer->set_external_hz(x);});
}

void SpecificWorker::update_social_laser(const std::vector<float> &dist, const std::vector<float> &angles)
 {
    if (auto laser = G->get_node(laser_name); laser.has_value())
        {
            if (auto laser_node = G->get_node(laser_social_name); laser_node.has_value())
            {
                //std::cout << __FUNCTION__ << " Adding fake laser to laser_node node " << std::endl;
                G->add_or_modify_attrib_local<laser_dists_att>(laser_node.value(), dist);//distancias
//                if (G->update_node(laser_node.value()))
//                    std::cout << __FUNCTION__ << " Node \"FakeLaser\" successfully updated in G" << std::endl;
//                else
//                    std::cout << __FILE__ << __FUNCTION__ << " Fatal error updating 'laser_node' node" << std::endl;
                G->update_node(laser_node.value());
            } else  // create a new node
            {
                DSR::Node new_laser_node = DSR::Node::create<laser_node_type>(laser_social_name);
                G->add_or_modify_attrib_local<parent_att>(new_laser_node, laser.value().id());
                G->add_or_modify_attrib_local<level_att>(new_laser_node,G->get_node_level(laser.value()).value() + 1);
                G->add_or_modify_attrib_local<pos_x_att>(new_laser_node, (float) 70);
                G->add_or_modify_attrib_local<pos_y_att>(new_laser_node, (float) -220);
                G->add_or_modify_attrib_local<laser_angles_att>(new_laser_node, angles);
                G->add_or_modify_attrib_local<laser_dists_att>(new_laser_node, dist);

                if (auto laser_node_id = G->insert_node(new_laser_node); laser_node_id.has_value())
                {
                    std::cout << __FUNCTION__ << " Node \"FakeLaser\" successfully inserted in G" << std::endl;
                    // insert EDGE
                    DSR::Edge edge = DSR::Edge::create<has_edge_type>(laser.value().id(), new_laser_node.id());
                    if (G->insert_or_assign_edge(edge))
                        std::cout << __FUNCTION__ << " Edge \"has_type\" inserted in G" << std::endl;
                    else
                        std::cout << __FILE__ << __FUNCTION__ << " Fatal error inserting new edge: "
                                  << laser.value().id() << "->" << laser_node_id.value()
                                  << " type: has" << std::endl;
                } else
                    std::cout << __FUNCTION__ << " Node \"FakeLaser\" could NOT be inserted in G" << std::endl;
            }
        }
 }
void SpecificWorker::modify_laser(const std::vector<DSR::Node> &personal_spaces,
                                  const std::vector<float> &angles,
                                  const std::vector<float> &dist)
{
    // compute lines joining gaussian points
    QVector<QLineF> lines;
    for( const auto &p_space : personal_spaces)
    {
        const auto gauss_x_o = G->get_attrib_by_name<ps_intimate_x_pos_att>(p_space);
        const auto gauss_y_o = G->get_attrib_by_name<ps_intimate_y_pos_att>(p_space);
        if( gauss_x_o.has_value() and gauss_y_o.has_value())
        {
            const std::vector<float> &gauss_x = gauss_x_o.value().get();
            const std::vector<float> &gauss_y = gauss_y_o.value().get();
            for(auto &&pos : iter::sliding_window(iter::zip(gauss_x, gauss_y),2))
            {
                const auto &[x1, y1] = pos[0];
                const auto &[x2, y2] = pos[1];
                lines.append(QLineF(QPointF(x1, y1), QPointF(x2, y2)));
            }
            lines.append(QLineF(QPointF(gauss_x.front(), gauss_y.front()), QPointF(gauss_x.back(), gauss_y.back())));
        }
        else  qWarning() << __FUNCTION__ << " No attributes gauss_x or gauss_y available in G";
    }
    // compute points of intersection of laser rays with gaussian polygons
    if(const auto robot_in_w = inner_eigen->transform(world_name, robot_name); robot_in_w.has_value())
    {
        QPointF robot(robot_in_w.value().x(), robot_in_w.value().y());
        std::vector<float> new_dist;
        new_dist.reserve(dist.size());
        for (auto &&[ang, dis] : iter::zip(angles, dist))
        {
            if (const auto laser_cart_world = inner_eigen->transform(world_name, Mat::Vector3d(dis*sin(ang), dis*cos(ang), 0.f), robot_name); laser_cart_world.has_value())
            {
                QPointF laser_point(laser_cart_world.value()[0], laser_cart_world.value()[1]);
                QLineF laser_line(robot, laser_point);  //Should be laser_node instead of robot
                std::vector<float> distances;
                std::transform(std::execution::par, lines.begin(), lines.end(), distances.begin(), [laser_line, robot, dis](auto line)
                        {
                            QPointF point;
                            if (laser_line.intersect(line, &point) == 1)
                                return QVector2D(point - robot).length();
                            else
                                return dis;
                        });
                if (not distances.empty())
                    new_dist.emplace_back(std::ranges::min(distances));
            }
            qWarning() << __FUNCTION__ << " No transform between world and robot available for conversion of laser points";
        }
    }
    else
        qWarning() << __FUNCTION__ << " No transform between world and robot available";
}

//std::optional<Eigen::Vector3d> SpecificWorker::transform_robot_to_world(float dist, float angle)
//{
//    float x= dist * cos(angle);
//    float y= dist * sin(angle);
//    return inner_eigen->transform(world_name, Mat::Vector3d(x, y, 0), robot_name);
//}

//void SpecificWorker::obtener_puntos_gausianas(auto personal_space, Eigen::VectorXf &all_gauss_x, Eigen::VectorXf &all_gauss_y)
//{
//    //auto gaussian = personal_space.size();
//    for (int i = 0; i < personal_space.size(); ++i)
//    {
//        const auto &gauss_x= G->get_attrib_by_name<ps_intimate_x_pos_att>(personal_space[i]).value().get();
//        const auto &gauss_y = G->get_attrib_by_name<ps_intimate_y_pos_att>(personal_space[i]).value().get();
//        for(int x=0; x<9; ++x)
//        {
//            all_gauss_x[9*i+x]=gauss_x[x];
//            all_gauss_y[9*i+x]=gauss_y[x];
//        }
//    }
//}
//void SpecificWorker::modificar_laser(Eigen::VectorXf all_gauss_x, Eigen::VectorXf all_gauss_y, auto angles, auto &dist)
//{
//    QVector<QLineF> lines;
//    for(int i = 0; i < all_gauss_x.size() / 9; i++)
//    {
//        for(int j = 0; j < 9; j++)
//        {
//            int pos = 9 * i + j;
//            const QPointF x(all_gauss_x[pos], all_gauss_y[pos]), y(all_gauss_x[(pos + 1) % 9], all_gauss_y[(pos + 1) % 9]);
//            QLineF newLine(x, y);
//            lines.append(newLine);
//        }
//    }
//
//    for (int i=0;i<angles.size();i++)
//    {
//        if(const auto dimensions = inner_eigen-> transform_axis(world_name, robot_name); dimensions.has_value())
//        {
//            QPointF robot (dimensions.value().x(), dimensions.value().y()), point = mod_privado(lines, robot, dist[i], angles[i]);
//            Eigen::Vector2f robot_v = {robot.x(), robot.y()}, point_v = {point.x() , point.y()};
//            dist[i] = point_v.norm();
//        }
//        else
//            qWarning() << __FUNCTION__ << "Error entrada";
//    }
//}

/////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

void SpecificWorker::add_or_assign_node_slot(std::uint64_t, const std::string &type)
{

}