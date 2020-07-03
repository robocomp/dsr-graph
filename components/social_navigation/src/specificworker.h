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
#include <Laser.h>
#include <navigation.h>
#include <grid.h>
#include <controller.h>
#include <custom_widget.h>

#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"
#include "dsr/gui/viewers/qscene_2d_viewer/qscene_2d_viewer.h"

#include <localPerson.h>
#include <cppitertools/zip.hpp>

#include <algorithm>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	using retPersonalSpaces = std::tuple <vector<QPolygonF>,vector<QPolygonF>,vector<QPolygonF>>;
	using retAffordanceSpaces = std::tuple <std::map<float, vector<QPolygonF>>,vector<QPolygonF>,vector<QPolygonF>>;

	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);


	RoboCompLaser::TLaserData updateLaser();
	void getPersonsFromModel();
	retPersonalSpaces getPolylinesFromModel();
	retAffordanceSpaces getAffordancesFromModel();
	void checkHumanBlock();


	void SocialRules_objectsChanged(RoboCompSocialRules::SRObjectSeq objectsAffordances);
	void SocialRules_personalSpacesChanged(RoboCompSocialNavigationGaussian::SNGPolylineSeq intimateSpaces, RoboCompSocialNavigationGaussian::SNGPolylineSeq personalSpaces, RoboCompSocialNavigationGaussian::SNGPolylineSeq socialSpaces);

public slots:
	void compute();
	int startup_check();
	void initialize(int period);
	void checkRobotAutoMovState();
    void moveRobot();
    void sendRobotTo();
	void forcesSliderChanged(int value = 0);


private:
	std::shared_ptr<DSR::InnerAPI> innermodel;
	// DSR graph
	std::shared_ptr<DSR::DSRGraph> G;

	//DSR params
	std::string agent_name;
	int agent_id;
	bool read_dsr;
	std::string dsr_input_file;

	// DSR graph viewer
	std::unique_ptr<DSR::GraphViewer> graph_viewer;
	QHBoxLayout mainLayout;
	QWidget window;
	bool startup_check_flag;

	//navigation
	std::shared_ptr<RoboCompCommonBehavior::ParameterList> confParams;
    Navigation<Grid<>,Controller> navigation;

 	RoboCompSocialNavigationGaussian::SNGPolylineSeq intimate_seq, personal_seq, social_seq;
    RoboCompSocialRules::SRObjectSeq objects_seq;

    bool personalSpacesChanged = false;
    bool affordancesChanged = false;

	std::vector <int32_t> prev_blockingIDs = {};
	std::vector<std::vector<int32_t>> prev_softBlockingIDs = {};

	localPersonsVec totalPersons;

	//local widget
	Custom_widget custom_widget;

};

#endif
