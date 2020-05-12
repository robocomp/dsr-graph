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
#include "../../../graph-related-classes/CRDT.h"
#include "../../../graph-related-classes/CRDT_graphviewer.h"

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
    std::string agent_name;
    std::shared_ptr<CRDT::CRDTGraph> G;
    std::unique_ptr<DSR::GraphViewer> graph_viewer;    
    QHBoxLayout mainLayout;
    QWidget window;
    
    
	SpecificWorker(TuplePrx tprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

    void HumanToDSR_newPeopleData(PeopleData people);    

private: 
    int get_new_node_id();
    std::optional<Node> create_node(std::string type, std::string name, int parent_id);
    
public slots:
	void compute();
	void initialize(int period);
private:
	std::shared_ptr<InnerModel> innerModel;
    
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

    std::unique_ptr<CRDT::InnerAPI> innermodel;
};

#endif
/*

SKELETON_CONNECTIONS = [("left_ankle", "left_knee"),
                        ("left_knee", "left_hip"),
                        ("right_ankle", "right_knee"),
                        ("right_knee", "right_hip"),
                        ("left_hip", "right_hip"),
                        ("left_shoulder", "left_hip"),
                        ("right_shoulder", "right_hip"),
                        ("left_shoulder", "right_shoulder"),
                        ("left_shoulder", "left_elbow"),
                        ("right_shoulder", "right_elbow"),
                        ("left_elbow", "left_wrist"),
                        ("right_elbow", "right_wrist"),
                        ("left_eye", "right_eye"),
                        ("nose", "left_eye"),
                        ("nose", "right_eye"),
                        ("left_eye", "left_ear"),
                        ("right_eye", "right_ear"),
                        ("left_ear", "left_shoulder"),
                        ("right_ear", "right_shoulder")]

*/