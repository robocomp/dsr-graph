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
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"

#include <custom_widget.h>

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

	//custom_widget
    void drawPersonalSpace();

    //graph signals
    void update_edge_slot(const std::uint64_t from, const std::uint64_t to, const std::string &type);

private:
    void updatePeopleInModel();
    void checkInteractions();
    void applySocialRules();
    void updatePersonalSpacesInGraph();
    void people_space_computation();
    //utils
    void convert_polyline_to_vector(const RoboCompSocialNavigationGaussian::SNGPolyline &poly, std::vector<float> &x_values, std::vector<float> &z_values);

private:
	// DSR graph
	std::shared_ptr<DSR::DSRGraph> G;

    std::shared_ptr<DSR::InnerAPI> inner_api;

	//DSR params
	std::string agent_name;
	int agent_id;

	bool tree_view;
	bool graph_view;
	bool qscene_2d_view;
	bool osg_3d_view;

	// DSR graph viewer
	std::unique_ptr<DSR::DSRViewer> dsr_viewer;
	QHBoxLayout mainLayout;
	bool startup_check_flag;

    //local widget
    Custom_widget custom_widget;


    std::map<uint64_t , RoboCompSocialNavigationGaussian::SNGPerson> map_people;
    //groups
    std::map<uint64_t, int32_t> person_to_group;
    struct group_space
    {
        std::vector<uint64_t> people_ids;
        RoboCompSocialNavigationGaussian::SNGPersonSeq people_seq;
        RoboCompSocialNavigationGaussian::SNGPolylineSeq intimatePolylines;
        RoboCompSocialNavigationGaussian::SNGPolylineSeq personalPolylines;
        RoboCompSocialNavigationGaussian::SNGPolylineSeq socialPolylines;
    };
    std::map<int32_t, group_space> people_groups; //People grouped on interactions


};

#endif
