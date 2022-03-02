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

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"
#include <doublebuffer/DoubleBuffer.h>
#include  "../../../etc/graph_names.h"
#include <fps/fps.h>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

public slots:
	void compute();
	int startup_check();
	void initialize(int period);

private:
	// DSR graph
	std::shared_ptr<DSR::DSRGraph> G;
    std::shared_ptr<DSR::InnerEigenAPI> inner_eigen;
	//DSR params
	std::string agent_name;
	int agent_id;

	bool tree_view;
	bool graph_view;
	bool qscene_2d_view;
	bool osg_3d_view;
    FPSCounter fps;

    //drawing
    DSR::QScene2dViewer* widget_2d;
    std::vector<std::tuple<QVector2D, QVector2D>> forces_vector;

	// DSR graph viewer
	std::unique_ptr<DSR::DSRViewer> graph_viewer;
	QHBoxLayout mainLayout;
	void add_or_assign_node_slot(std::uint64_t, const std::string &type);
	void add_or_assign_attrs_slot(std::uint64_t id, const std::map<std::string, DSR::Attribute> &attribs){};
	void add_or_assign_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type){};
	void del_edge_slot(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){};
	void del_node_slot(std::uint64_t from){};     
	bool startup_check_flag;
  
    Eigen::Vector3d transform_robot_to_world(float dist, float angle);
    void modificar_laser(Eigen::VectorXf all_gauss_x, Eigen::VectorXf all_gauss_y, auto angles, auto &dist) ;
    QPointF mod_privado(QVector<QLineF> lines,QPointF robot, float dist, float angle);
    void obtener_puntos_gausianas(auto personal_space, Eigen::VectorXf &all_gauss_x, Eigen::VectorXf &all_gauss_y);
    void draw_laser(std::vector<float> angles, std::vector<float> dist, QGraphicsScene* viewer_2d);
    void update_social_laser(const std::vector<float> &dist, const std::vector<float> &angles);


	// laser stuff
	//optional<Eigen::Vector3d> transform_robot_to_world(float dist, float angle);
    //void modificar_laser(Eigen::VectorXf all_gauss_x, Eigen::VectorXf all_gauss_y, auto angles, auto &dist) ;
    //QPointF mod_privado(QVector<QLineF> lines,QPointF robot, float dist, float angle);
    //void obtener_puntos_gausianas(auto personal_space, Eigen::VectorXf &all_gauss_x, Eigen::VectorXf &all_gauss_y);
    std::vector<float> modify_laser(const std::vector<DSR::Node> &personal_spaces, const std::vector<float> &angles, const std::vector<float> &dist);

};

#endif
