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
    QLoggingCategory::setFilterRules("*.debug=false\n");
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
	confParams  = std::make_shared<RoboCompCommonBehavior::ParameterList>(params);
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
		// create graph
		G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id); // Init nodes
		std::cout<< __FUNCTION__ << "Graph loaded" << std::endl;  

        // Graph viewer
		using opts = DSR::DSRViewer::view;
		int current_opts = 0;
		//opts main = opts::none;
		if(tree_view)
			current_opts = current_opts | opts::tree;
		if(graph_view)
			current_opts = current_opts | opts::graph;
		if(qscene_2d_view)
			current_opts = current_opts | opts::scene;
		if(osg_3d_view)
			current_opts = current_opts | opts::osg;
		graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts);
        setWindowTitle(QString::fromStdString(agent_name + "-" + dsr_input_file));

		//Inner Api
		innermodel = G->get_inner_api();

		//Custom widget
		custom_widget.show();
		connect(custom_widget.autoMov_checkbox, SIGNAL(clicked()),this, SLOT(checkRobotAutoMovState()));
    	connect(custom_widget.robotMov_checkbox, SIGNAL(clicked()),this, SLOT(moveRobot()));
    	connect(custom_widget.ki_slider, SIGNAL (valueChanged(int)),this,SLOT(forcesSliderChanged(int)));
    	connect(custom_widget.ke_slider, SIGNAL (valueChanged(int)),this,SLOT(forcesSliderChanged(int)));
    	connect(custom_widget.send_button, SIGNAL(clicked()),this, SLOT(sendRobotTo()));
        connect(custom_widget.stop_button, SIGNAL(clicked()),this, SLOT(stopRobot()));

        forcesSliderChanged();
    	moveRobot();

		widget_2d = qobject_cast<DSR::QScene2dViewer*> (graph_viewer->get_widget(opts::scene));
		navigation.initialize(G, confParams, &widget_2d->scene, true, "viriato.grid");
        widget_2d->set_draw_laser(true);
		connect(widget_2d, SIGNAL(mouse_right_click(int, int, int)), this, SLOT(new_target_from_mouse(int,int,int)));

		this->Period = 160;
		timer.start(Period);
	}
}

void SpecificWorker::compute()
{
	//bool needsReplaning = false;
    navigation.update();
}

void  SpecificWorker::moveRobot()
{
    qDebug() << __FUNCTION__;
    if(custom_widget.robotMov_checkbox->checkState() == Qt::CheckState(2))
    {
        custom_widget.autoMov_checkbox->setEnabled(true);
        navigation.moveRobot = true;
		navigation.stopMovingRobot = false;
    }
    else
    {
        if(navigation.current_target.active.load())
			navigation.stopMovingRobot = true;
        else
		{
            navigation.moveRobot = false;
			navigation.stopMovingRobot = false;
		}
        custom_widget.autoMov_checkbox->setEnabled(false);
    }
}

void  SpecificWorker::checkRobotAutoMovState()
{
	qDebug()<<__FUNCTION__;

	if(custom_widget.autoMov_checkbox->checkState() == Qt::CheckState(2))
	{
		navigation.robotAutoMov = true;
		navigation.newRandomTarget();
	}
	else
    {
        navigation.robotAutoMov = false;
    }
}

void SpecificWorker::new_target_from_mouse(int pos_x, int pos_y, int id)
{
    std::cout << "New target:" << pos_x << " " << pos_y << " " << id << std::endl;
    navigation.newTarget(QPointF(pos_x, pos_y));
}

///////////////////////////////////////////////////
/// GUI
//////////////////////////////////////////////////

void SpecificWorker::sendRobotTo()
{
    auto x =  custom_widget.x_spinbox->value();
    auto z =  custom_widget.z_spinbox->value();
    navigation.newTarget(QPointF(x,z));
}

void SpecificWorker::stopRobot()
{
    navigation.stopRobot();
}

void SpecificWorker::forcesSliderChanged(int value)
{
    navigation.KI = (float) custom_widget.ki_slider -> sliderPosition();
    navigation.KE = (float) custom_widget.ke_slider -> sliderPosition();
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

/**************************************/
// From the RoboCompSocialRules you can use this types:
// RoboCompSocialRules::SRObject

