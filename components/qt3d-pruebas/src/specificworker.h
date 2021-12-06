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
#include "dsr/api/dsr_inner_eigen_api.h"
#include "dsr/api/dsr_geometry_queries_api.h"
#include "dsr/api/dsr_camera_api.h"
#include <dsr/api/GHistorySaver.h>

#include "3D_view.h"

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/highgui.hpp>


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
    geometry_queries_api geom;
    std::unique_ptr<DSR::CameraAPI> cam;
    std::shared_ptr<DSR::InnerEigenAPI> inner;
    //std::unique_ptr<GSerializer> gsr;
    std::unordered_map<std::string, QLabel*> contornos;
    std::unordered_map<std::string, std::unique_ptr<cv::Mat>> contornos_img;

    //DSR params
	std::string agent_name;
	int agent_id;

	bool tree_view;
	bool graph_view;
	bool qscene_2d_view;
	bool osg_3d_view;

	// DSR graph viewer
	std::unique_ptr<DSR::DSRViewer> graph_viewer;
    std::unique_ptr<World> world_view;

    QHBoxLayout mainLayout;
	void add_or_assign_node_slot(std::uint64_t, const std::string &type){};
	void add_or_assign_attrs_slot(std::uint64_t id, const std::map<std::string, DSR::Attribute> &attribs){};
	void add_or_assign_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type);

    void virtual_structure_validation(const std::string& name, const Mat::Vector3d& pos, const std::vector<std::pair<std::string, float>>& near);
	void del_edge_slot(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){
	    if (edge_tag == "visible") {
            std::cout << "close" << std::endl;
            std::string name = G->get_name_from_id(to).value_or("");
            if (contornos[name]) {
                contornos[name]->close();
                delete contornos[name];
            }
            contornos.erase(name);
            contornos_img.erase(name);
        }
	};
	void del_node_slot(std::uint64_t from){};     
	bool startup_check_flag;

};

#endif
