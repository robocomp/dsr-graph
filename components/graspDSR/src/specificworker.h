/*
 *    Copyright (C) 2020 by Mohamed Shawky
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
#include <custom_widget.h>
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <Eigen/Dense>
#include "../../../etc/viriato_graph_names.h"

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

	// DSR params
	std::string agent_name;
	
	int agent_id;

	bool tree_view;
	bool graph_view;
	bool qscene_2d_view;
	bool osg_3d_view;

	// DSR graph viewer
	std::unique_ptr<DSR::DSRViewer> graph_viewer;
	QHBoxLayout mainLayout;
	QWidget window;
	bool startup_check_flag;

    // Local widget
    Custom_widget custom_widget;

	// Pose Estimation & Grasping

	// Grasp attributes
    std::string grasp_object;
    std::map<std::string,std::vector<std::vector<float>>> objects_pcl;

	// Geometry utilities
	vector<float> quat_to_euler(vector<float> quat);
	vector<vector<float>> quat_to_rotm(vector<float> quat);
	vector<float> interpolate_trans(vector<float> src, vector<float> dest, float factor);
    vector<vector<float>> project_vertices(vector<vector<float>> vertices, vector<vector<float>> rot, vector<float> trans, vector<vector<float>> intrinsics);

	// G read utilities
	RoboCompCameraRGBDSimple::TImage get_rgb_from_G();
	RoboCompCameraRGBDSimple::TDepth get_depth_from_G();
	std::vector<std::vector<float>> get_camera_intrinsics();

	// G injection utilities
	void inject_estimated_poses(RoboCompObjectPoseEstimationRGBD::PoseType poses);
    void update_node_slot(const std::int32_t id, const std::string &type);

	// IO utilities
    std::map<std::string,std::vector<std::vector<float>>> read_pcl_from_file();

	// Display utilities
    void show_image(cv::Mat &img, RoboCompObjectPoseEstimationRGBD::PoseType poses);
    void draw_vertices(cv::Mat &img, std::vector<std::vector<float>> vertices_2d);
};

#endif
