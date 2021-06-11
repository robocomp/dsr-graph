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
		connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::add_or_assign_node_slot);
		connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::add_or_assign_edge_slot);
		//connect(G.get(), &DSR::DSRGraph::update_attrs_signal, this, &SpecificWorker::add_or_assign_attrs_slot);
		connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
		connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);

		//inner
        inner_eigen = G->get_inner_eigen_api();

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

        widget_2d = qobject_cast<DSR::QScene2dViewer*> (graph_viewer->get_widget(opts::scene));

        if(widget_2d != nullptr)
            widget_2d->set_draw_laser(true);

		this->Period = period;
		timer.start(Period);
	}
    //DSR:: Node laser_fake = DSR:: Node::create<laser_node_type>("laser_fake");
    //G->add_or_modify_attrib_local<name_att>(laser_fake,(string)"laser_fake");
    //G->add_or_modify_attrib_local<parent_att>(laser_fake,G->get_node(laser_name).value().id());
    //G->add_or_modify_attrib_local<level_att>(laser_fake, G->get_node_level(G->get_node(laser_name).value()).value() + 1);
    //G->add_or_modify_attrib_local<pos_x_att>(laser_fake, (float) 90);
    //G->add_or_modify_attrib_local<pos_y_att>(laser_fake, (float) -257);
    //G->insert_node(laser_fake);
}

void SpecificWorker::compute()
{
    //Comprobar si hay nodos de tipo personas

    if(auto personal_space = G->get_nodes_by_type("personal_space"); not personal_space.empty()) {
        Eigen::VectorXf all_gauss_x(personal_space.size() * 9), all_gauss_y(personal_space.size() * 9);

        auto laser_node= G->get_node(laser_name).value();
        const auto &angles = G->get_attrib_by_name<laser_angles_att>(laser_node).value().get();
        auto dist = G->get_attrib_by_name<laser_dists_att>(laser_node).value().get();
        obtener_puntos_gausianas(personal_space, all_gauss_x, all_gauss_y);
        modificar_laser(all_gauss_x, all_gauss_y, angles, dist);

        if (auto laser = G->get_node(laser_name); laser.has_value()) {
            if (auto laser_node = G->get_node(laser_social_name); laser_node.has_value()) {
                std::cout << __FUNCTION__ << " Adding fake laser to laser_node node " << std::endl;
                G->add_or_modify_attrib_local<laser_dists_att>(laser_node.value(), dist);//distancias
                if (G->update_node(laser_node.value()))
                    std::cout << __FUNCTION__ << " Node \"FakeLaser\" successfully updated in G" << std::endl;
                else
                    std::cout << __FILE__ << __FUNCTION__ << " Fatal error updating 'laser_node' node" << std::endl;
            } else  // create a new node
            {
                DSR::Node new_laser_node = DSR::Node::create<laser_node_type>(laser_social_name);
                G->add_or_modify_attrib_local<parent_att>(new_laser_node, laser.value().id());
                G->add_or_modify_attrib_local<level_att>(new_laser_node,G->get_node_level(laser.value()).value() + 1);
                G->add_or_modify_attrib_local<pos_x_att>(new_laser_node, (float) 70);
                G->add_or_modify_attrib_local<pos_y_att>(new_laser_node, (float) -220);
                G->add_or_modify_attrib_local<laser_angles_att>(new_laser_node, angles);
                G->add_or_modify_attrib_local<laser_dists_att>(new_laser_node, dist);

                if (std::optional<int> laser_node_id = G->insert_node(new_laser_node); laser_node_id.has_value()) {
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
    //
	//qInfo()<< angles.size() << endl;
    //qInfo()<< dist.size() << endl;

	//computeCODE
	//QMutexLocker locker(mutex);
	//try
	//{
	//  camera_proxy->getYImage(0,img, cState, bState);
	//  memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
	//  searchTags(image_gray);
	//}
	//catch(const Ice::Exception &e)
	//{
	//  std::cout << "Error reading from Camera" << e << std::endl;
	//}
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

void SpecificWorker::add_or_assign_node_slot(std::uint64_t, const std::string &type){

}

void SpecificWorker::obtener_puntos_gausianas(auto personal_space, Eigen::VectorXf &all_gauss_x, Eigen::VectorXf &all_gauss_y){
    //auto gaussian = personal_space.size();
    for (int i = 0; i < personal_space.size(); ++i) {
        const auto &gauss_x= G->get_attrib_by_name<ps_intimate_x_pos_att>(personal_space[i]).value().get();
        const auto &gauss_y = G->get_attrib_by_name<ps_intimate_y_pos_att>(personal_space[i]).value().get();
        for(int x=0; x<9; ++x) {
            all_gauss_x[9*i+x]=gauss_x[x];
            all_gauss_y[9*i+x]=gauss_y[x];
        }
    }
}
void SpecificWorker::modificar_laser(Eigen::VectorXf all_gauss_x, Eigen::VectorXf all_gauss_y, auto angles, auto &dist) {
    QVector<QLineF> lines;
    for(int i = 0; i < all_gauss_x.size() / 9; i++){
        for(int j = 0; j < 9; j++){
            int pos = 9 * i + j;
            const QPointF x(all_gauss_x[pos], all_gauss_y[pos]), y(all_gauss_x[(pos + 1) % 9], all_gauss_y[(pos + 1) % 9]);
            QLineF newLine(x, y);
            lines.append(newLine);

        }
    }

    for (int i=0;i<angles.size();i++) {
        if(const auto dimensions = inner_eigen-> transform_axis(world_name, robot_name); dimensions.has_value())
        {
            QPointF robot (dimensions.value().x(), dimensions.value().y()), point = mod_privado(lines, robot, dist[i], angles[i]);
            Eigen::Vector2f robot_v = {robot.x(), robot.y()}, point_v = {point.x() , point.y()};
            dist[i] = point_v.norm();
        }
        else
            qWarning() << __FUNCTION__ << "Error entrada";

    }
}


QPointF SpecificWorker::mod_privado(QVector<QLineF> lines, QPointF robot, float dist, float angle) {
    auto robot_laser = transform_robot_to_world(dist, angle);
    QPointF laser (robot_laser[0], robot_laser[1]);
    QLineF laser_line(robot, laser);
    QPointF less_distance_point = laser;
    for (int i=0; i < lines.size(); i++){
        QPointF point;
        int intersection_type = laser_line.intersect(lines.at(i), &point);
        if (intersection_type == 1) {
            Eigen::Vector2f robot_point = {point.x() - robot.x(), point.y() - robot.y()};
            if (robot_point.norm() < dist) {
                less_distance_point = point;
            }
        }
    }
    return less_distance_point;
}

Eigen::Vector3d SpecificWorker::transform_robot_to_world(float dist, float angle)
{
    auto inner = G->get_inner_eigen_api();
    float x= dist * cos(angle);
    float y= dist * sin(angle);
    auto robot_laser= inner->transform(world_name,Mat::Vector3d(x, y, 0),robot_name).value();
    return robot_laser;
}
