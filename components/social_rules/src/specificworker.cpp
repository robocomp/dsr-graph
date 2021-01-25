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

using namespace DSR;
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
		dsr_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
		setWindowTitle(QString::fromStdString(agent_name + "-") + QString::number(agent_id));

        connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::update_edge_slot);
		// custom_widget
        dsr_viewer->add_custom_widget_to_dock("Social Rules", &custom_widget);

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

        //launched on component initialization, afterward changes are received on node updates.
        people_space_computation();
	}

}



void SpecificWorker::update_edge_slot(const std::uint64_t from, const std::uint64_t to, const std::string &type)
{
    if (type == "RT")
    {
        //check if edge involves any person
        auto node = G->get_node(to);
        if (node.has_value() and node.value().type() == "person") {
            std::cout<<"Update edge from: "<<from<<" to person: "<<to<<std::endl;
            people_space_computation();
        }
    }
}

void SpecificWorker::compute()
{
}

//launched on component initialization, afterward changes are received on node updates.
void SpecificWorker::people_space_computation()
{
    std::cout<<"Update people personal space"<<std::endl;
    updatePeopleInModel();
    checkInteractions();
    applySocialRules();
    updatePersonalSpacesInGraph();
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

// Transform Polylines into two std::vector<float>
void SpecificWorker::convert_polyline_to_vector(const RoboCompSocialNavigationGaussian::SNGPolyline &poly, std::vector<float> &x_values, std::vector<float> &z_values)
{
    x_values.clear();
    z_values.clear();
    for (const auto &value: poly) {
        x_values.push_back(value.x);
        z_values.push_back(value.z);
    }
}

//insert data on G
void SpecificWorker::updatePersonalSpacesInGraph() {
    qDebug() << __FUNCTION__;
    std::vector<float> x_values, z_values;
    std::optional<Node> person_node;
    size_t polyline_position;
    for(const auto &group: people_groups)
    {
        size_t cont = 0;
        for(const uint64_t &person_id: group.second.people_ids)
        {
            person_node = G->get_node(person_id);
            if(not person_node.has_value())
                return;
//if each person has it own polyline that one is assigned, otherwise, first polyline is assigned to all
            if(group.second.intimatePolylines.size() > cont)
                polyline_position = cont;
            else
                polyline_position = 0;
            if(group.second.intimatePolylines.size() > polyline_position) { //TODO: va todo en el if?
                convert_polyline_to_vector(group.second.intimatePolylines[polyline_position], x_values, z_values);
                G->add_or_modify_attrib_local<person_intimate_x_pos_att>(person_node.value(), x_values);
                G->add_or_modify_attrib_local<person_intimate_y_pos_att>(person_node.value(), z_values);
            }
            if(group.second.personalPolylines.size() > cont)
                polyline_position = cont;
            else
                polyline_position = 0;
            if(group.second.personalPolylines.size() > polyline_position) {
                convert_polyline_to_vector(group.second.personalPolylines[polyline_position], x_values, z_values);
                G->add_or_modify_attrib_local<person_personal_x_pos_att>(person_node.value(), x_values);
                G->add_or_modify_attrib_local<person_personal_y_pos_att>(person_node.value(), z_values);
            }

            if(group.second.socialPolylines.size() > cont)
                polyline_position = cont;
            else
                polyline_position = 0;
            if(group.second.socialPolylines.size() > polyline_position) {
                convert_polyline_to_vector(group.second.socialPolylines[polyline_position], x_values, z_values);
                G->add_or_modify_attrib_local<person_social_x_pos_att>(person_node.value(),  x_values);
                G->add_or_modify_attrib_local<person_social_y_pos_att>(person_node.value(),  z_values);
            }
            x_values.clear();
            std::transform(group.second.people_ids.begin(), group.second.people_ids.end(), std::back_inserter(x_values), [](const auto &value) { return (float)value; });
            G->add_or_modify_attrib_local<person_sharedWidth_att>(person_node.value(), x_values);
            G->update_node(person_node.value());
            cont++;
        }
    }
}