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
#include "doublebuffer/DoubleBuffer.h"
#include <unordered_map>
#include <genericworker.h>
#include <innermodel/innermodel.h>
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"
#include <QHBoxLayout>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
private: 
    std::unique_ptr<DSR::InnerAPI> innermodel;


    int agent_id;
    std::string dsr_output_path;
    std::vector<std::string> COCO_IDS{"nose", "left_eye", "right_eye", "left_ear", "right_ear", "left_shoulder", "right_shoulder", "left_elbow",
            "right_elbow", "left_wrist", "right_wrist", "left_hip", "right_hip", "left_knee", "right_knee", "left_ankle", "right_ankle"};


    struct JOINT_CONNECTION
    {
        std::string parent_name;
        std::vector<float> translation;
    };
    const int MAXTIME = 2000; //Maximum time elapsed without seen a person before deleted    
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
    DoubleBuffer<RoboCompHumanToDSRPub::PeopleData, RoboCompHumanToDSRPub::PeopleData> people_data_buffer;


    std::unordered_map<int, int> G_person_id;

public:
    std::string agent_name;
    std::shared_ptr<DSR::DSRGraph> G;
    std::unique_ptr<DSR::GraphViewer> graph_viewer;    
    QHBoxLayout mainLayout;
    QWidget window;
    
    std::map<int, std::chrono::system_clock::time_point> people_last_seen; 

	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

    void HumanToDSRPub_newPeopleData(RoboCompHumanToDSRPub::PeopleData people);    

private: 
    std::optional<Node> create_node(const std::string &type, const std::string &name, int person_id,  int parent_idz);
    std::optional<Node> create_node_mesh(const std::string &name, const std::string &path, int parent_id);
    void process_people_data(RoboCompHumanToDSRPub::PeopleData people);    
    void check_unseen_people();

    const std::string person_path = "/home/robocomp/robocomp/components/robocomp-viriato/files/osgModels/";
    const std::string person1_path = person_path + "human04.3ds";
    const std::string person2_path = person_path + "human02.3ds";
    const std::string abuelito_path = "/home/pbustos/robocomp/components/robocomp-viriato/files/osgModels/abuelito.ive";
    

public slots:
	void compute();
	void initialize(int period);
	    
};

#endif
