/*
 *    Copyright (C) 2020 by YOUR NAME HERE
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
#include "custom_widget.h"
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include  "../../../etc/viriato_graph_names.h"
#include <doublebuffer/DoubleBuffer.h>
#include "collisions.h"
#include <octovis/OcTreeDrawer.h>
#include <octovis/ViewerWidget.h>
#include <octovis/OcTreeRecord.h>
#include <QGLViewer/qglviewer.h>

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
    void update_node_slot(const std::int32_t id, const std::string &type);

private:
	// DSR graph
	std::shared_ptr<DSR::DSRGraph> G;
    std::shared_ptr<DSR::InnerEigenAPI> inner_eigen;

	//DSR params
	std::string agent_name;
	int agent_id;
    std::shared_ptr<RoboCompCommonBehavior::ParameterList> conf_params;

	bool tree_view;
	bool graph_view;
	bool qscene_2d_view;
	bool osg_3d_view;

	// DSR graph viewerr
	std::unique_ptr<DSR::DSRViewer> graph_viewer;
	QHBoxLayout mainLayout;
	bool startup_check_flag;

    //local widget
    Custom_widget custom_widget;


    using LaserData = std::tuple<std::vector<float>, std::vector<float>>;  //<angles, dists>
    DoubleBuffer<LaserData, octomap::Pointcloud> laser_buffer;
    DoubleBuffer<octomap::point3d, octomap::point3d> robot_pose_buffer;
    //DoubleBuffer<std::vector<float>, octomap::Pointcloud> pointcloud_buffer;
    DoubleBuffer<std::vector<float>, std::vector<float>> pointcloud_buffer;

    std::shared_ptr<Collisions> collisions;

	// Octree
	//octomap::point3d robot_pose{0,0,0};
	octomap::OcTree *octo;
	void initialize_octomap(bool read_from_file = false, const std::string file_name = std::string());
    void show_OcTree();
	//octomap::OcTreeDrawer octo_drawer;
	//octomap::ViewerWidget *octo_viewer;
    octomap::OcTreeRecord otr;


    struct Dimensions
    {
        int TILE_SIZE = 100;
        float MAX_HEIGHT = 1600;
        float HMIN = -2500, VMIN = -2500, WIDTH = 2500, HEIGHT = 2500;
    };

};

#endif
