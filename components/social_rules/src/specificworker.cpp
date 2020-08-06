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
		G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id); // Init nodes
		std::cout<< __FUNCTION__ << "Graph loaded" << std::endl;

        //Inner Api
        inner_api = G->get_inner_api();

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

		// custom_widget
		graph_viewer->add_custom_widget_to_dock("Social Rules", &custom_widget);

        connect(custom_widget.draw_personalSpace_button,SIGNAL(clicked()),this, SLOT(drawPersonalSpace()));
/*        connect(custom_widget.save_data_button,SIGNAL(clicked()),this, SLOT(recordData()));
        connect(custom_widget.object_slider, SIGNAL (valueChanged(int)),this,SLOT(affordanceSliderChanged(int)));
        connect(custom_widget.currentTime_timeEdit, SIGNAL (timeChanged(const QTime)),this,SLOT(affordanceTimeEditChanged(const QTime)));
        connect(custom_widget.setTherapy_button, SIGNAL (clicked()),this,SLOT(programTherapy()));
        connect(custom_widget.removeT_button, SIGNAL (clicked()),this,SLOT(removeTherapy()));
        connect(custom_widget.currtime_slider, SIGNAL (valueChanged(int)),this,SLOT(affordanceTimeSliderChanged(int)));

        checkObjectAffordance();

        auto timeValue = custom_widget.currtime_slider->value();
        QTime currentTime = QTime(timeValue / 60, timeValue % 60);
        custom_widget.currentTime_timeEdit->setTime(currentTime);

        QString hour = currentTime.toString(Qt::SystemLocaleShortDate);
        for (auto const &map : mapIdObjects)
        {
            mapCostsPerHour[hour].push_back(map.second.cost);
        }
*/

		this->Period = period;
		timer.start(Period);
	}

}

void SpecificWorker::compute()
{
 //   if (worldModelChanged)
    {
        updatePeopleInModel();
        checkInteractions();
//        checkObjectAffordance();
        applySocialRules();

        updatePersonalSpacesInGraph();
//        updateAffordancesInGraph();

//        worldModelChanged = false;
//        costChanged = false;
    }

 /*   else if (costChanged)
    {
        updateAffordancesInGraph();

//        publishAffordances();

        costChanged = false;
    }*/

//    checkRobotmov();

	
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

void SpecificWorker::drawPersonalSpace(){
    if (!people_groups.empty()){
        for (auto per: people_groups){
            RoboCompSocialNavigationGaussian::SNGPolylineSeq intimate_result, personal_result, social_result;
            socialnavigationgaussian_proxy->getAllPersonalSpaces(per.second.people_seq, true, intimate_result, personal_result, social_result);
        }
    }
}


void SpecificWorker::updatePeopleInModel(){
    qDebug()<< __FUNCTION__;

    map_people.clear();

    auto vectorPersons = G->get_nodes_by_type("person");
    for (auto p: vectorPersons) {
        RoboCompSocialNavigationGaussian::SNGPerson person;

        std::optional<QVec> pose = inner_api->transformS6D("world", p.name());
        if(pose.has_value()) {
            person.id = p.id();
            person.x = pose.value().x();
            person.z = pose.value().z();
            person.angle = pose.value().ry();
            cout << "[FOUND] Person " << person.id << " x = " << person.x << " z = " << person.z << " rot "
                 << person.angle << endl;

            map_people[person.id] = person;
        }
    }
}

// evaluate person interaction
// alone person are evaluated as single group
//TODO: evaluate with several people groups
void SpecificWorker::checkInteractions(){
    qDebug()<< __FUNCTION__;
    //clear previous data
    //TODO: check if something could be reused => node signals
    people_groups.clear();
    person_to_group.clear();

    std::vector<std::vector<int32_t>> interactingId;
    for (const auto &[id, sng] : map_people){
        std::vector<int32_t> people_to_include={id};
        Node p1 = G->get_node(id).value();
        std::vector<Edge> edges = G->get_node_edges_by_type(p1, "interacting");
        if(edges.size() > 0){
            for(Edge edge: edges){
                Node to_node = G->get_node(edge.to()).value();
                if(to_node.type() == "person")
                {
                    people_to_include.push_back(to_node.id());
                }
            }
        }
        //check where to include
        //People are included in the same slot as already one of them has been included
        //If there is not any of them already included, new slot is created
        bool found = false;
        for(const int32_t &new_id: people_to_include){
            std::map<int32_t, int32_t>::iterator  it = person_to_group.find(new_id);
            if( it != person_to_group.end() ){
                found = true;
                //include new people ids
                for(const int32_t &new_id: people_to_include)
                {
                    //check if person is already included
                    if(std::find(people_groups[it->second].people_ids.begin(),
                                 people_groups[it->second].people_ids.end(), new_id)
                                 == people_groups[it->second].people_ids.end())
                    {
                        people_groups[it->second].people_ids.push_back(new_id);
                        people_groups[it->second].people_seq.push_back(map_people[new_id]);
                    }
                }
                break;
            }
        }
        if(not found){ //create new slot
            int group_id = people_groups.size();
            group_space new_group;
            new_group.people_ids = people_to_include;
            for(const int32_t &new_id: people_to_include){
                person_to_group[new_id] = group_id;
                new_group.people_seq.push_back(map_people[new_id]);
            }
            people_groups[group_id] = new_group;
        }
    }
}

void SpecificWorker::applySocialRules(){
    qDebug()<< __FUNCTION__;
    for (auto &group: people_groups) {
        RoboCompSocialNavigationGaussian::SNGPolylineSeq intimateResult, personalResult, socialResult;
        try {
            socialnavigationgaussian_proxy->getAllPersonalSpaces(group.second.people_seq, false, intimateResult, personalResult, socialResult);
        }
        catch( const Ice::Exception &e)
        {
            std::cout << e << std::endl;
        }

        for (auto s:intimateResult) {group.second.intimatePolylines.push_back(s);}
        for (auto s:personalResult) {group.second.personalPolylines.push_back(s);}
        for (auto s:socialResult) {group.second.socialPolylines.push_back(s);}
    }
}

//insert data on G
void SpecificWorker::updatePersonalSpacesInGraph() {
    qDebug() << __FUNCTION__;

    std::optional<Node> person_node;
    for(const auto &group: people_groups)
    {
        for(const int &person_id: group.second.people_ids)
        {
            person_node = G->get_node(person_id);
            if(not person_node.has_value())
                return;
            // Transform intimatePolylines into two std::vector<float>
            std::vector<float> aux1, aux2;
            //TODO: personal space could have more than one polyline for each space??¿¿
            if(group.second.intimatePolylines.size() > 0) {
                for (const auto &value: group.second.intimatePolylines[0]) {
                    aux1.push_back(value.x);
                    aux2.push_back(value.z);
                }
                G->add_or_modify_attrib_local(person_node.value(), "intimate_x_pos", aux1);
                G->add_or_modify_attrib_local(person_node.value(), "intimate_y_pos", aux2);
                aux1.clear();
                aux2.clear();
            }
            if(group.second.personalPolylines.size() > 0) {
                for (const auto &value: group.second.personalPolylines[0]) {
                    aux1.push_back(value.x);
                    aux2.push_back(value.z);
                }
                G->add_or_modify_attrib_local(person_node.value(), "personal_x_pos", aux1);
                G->add_or_modify_attrib_local(person_node.value(), "personal_y_pos", aux2);
                aux1.clear();
                aux2.clear();
            }
            if(group.second.socialPolylines.size() > 0) {
                for (const auto &value: group.second.socialPolylines[0]) {
                    aux1.push_back(value.x);
                    aux2.push_back(value.z);
                }
                G->add_or_modify_attrib_local(person_node.value(), "social_x_pos", aux1);
                G->add_or_modify_attrib_local(person_node.value(), "social_y_pos", aux2);
            }
            aux1.clear();
            std::transform(group.second.people_ids.begin(), group.second.people_ids.end(), std::back_inserter(aux1), [](const auto &value) { return (float)value; });
            G->add_or_modify_attrib_local(person_node.value(), "sharedWith", aux1);
            G->update_node(person_node.value());
        }
    }
}