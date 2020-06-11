/*
 *    Copyright (C)2020 by YOUR NAME HERE
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
#include <innermodel/innermodel.h>
#include "doublebuffer/DoubleBuffer.h"
#include "../../../graph-related-classes/CRDT.h"
#include "../../../graph-related-classes/CRDT_graphviewer.h"
#include <QHBoxLayout>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
private: 
    std::unique_ptr<CRDT::InnerAPI> innermodel;

    int agent_id;
    std::string dsr_output_path;
    std::vector<std::string> COCO_IDS{"nose", "left_eye", "right_eye", "left_ear", "right_ear", "left_shoulder", "right_shoulder", "left_elbow",
            "right_elbow", "left_wrist", "right_wrist", "left_hip", "right_hip", "left_knee", "right_knee", "left_ankle", "right_ankle"};
    
    struct JOINT_CONNECTION
    {
        std::string parent_name;
        std::vector<float> translation;
    };
    std::map<std::string, JOINT_CONNECTION> jointMap{ 
        {"nose", JOINT_CONNECTION{"person",{0.0, 300.0, 0.0}}}, 
        {"left_eye", JOINT_CONNECTION{"nose",{-30.0, 30.0, 0.0}}}, 
        {"right_eye", JOINT_CONNECTION{"nose",{30.0, 30.0, 0.0}}}, 
        {"left_ear", JOINT_CONNECTION{"nose",{-100.0, 0.0, 0.0}}}, 
        {"right_ear", JOINT_CONNECTION{"nose",{100.0, 0.0, 0.0}}}, 

        {"left_shoulder", JOINT_CONNECTION{"person",{-250.0, 150.0, 0.0}}},
        {"left_elbow", JOINT_CONNECTION{"left_shoulder",{-230.0, 0.0, 0.0}}},
        {"left_wrist", JOINT_CONNECTION{"left_elbow",{-230.0, 0.0, 0.0}}},
        {"right_shoulder", JOINT_CONNECTION{"person",{250.0, 150.0, 0.0}}},
        {"right_elbow", JOINT_CONNECTION{"right_shoulder",{230.0, 0.0, 0.0}}},
        {"right_wrist", JOINT_CONNECTION{"right_elbow",{230.0, 0.0, 0.0}}},

        {"left_hip", JOINT_CONNECTION{"person",{-200.0, -400.0, 0.0}}},
        {"left_knee", JOINT_CONNECTION{"left_hip",{0.0, -400.0, 0.0}}},
        {"left_ankle", JOINT_CONNECTION{"left_knee",{0.0, -400.0, 0.0}}},
        {"right_hip", JOINT_CONNECTION{"person",{200.0, -400.0, 0.0}}},
        {"right_knee", JOINT_CONNECTION{"right_hip",{0.0, -400.0, 0.0}}},
        {"right_ankle", JOINT_CONNECTION{"right_knee",{0.0, -400.0, 0.0}}}
    };
    DoubleBuffer<RoboCompHumanToDSR::PeopleData, RoboCompHumanToDSR::PeopleData> people_data_buffer;
public:
    std::string agent_name;
    std::shared_ptr<CRDT::CRDTGraph> G;
    std::unique_ptr<DSR::GraphViewer> graph_viewer;    
    QHBoxLayout mainLayout;
    QWidget window;
    
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

    void HumanToDSR_newPeopleData(RoboCompHumanToDSR::PeopleData people);    

private: 
    int get_new_node_id();
    std::optional<Node> create_node(std::string type, std::string name, int parent_idz);
    void process_people_data(RoboCompHumanToDSR::PeopleData people);    
    

public slots:
	void compute();
	void initialize(int period);
	    
};

#endif
