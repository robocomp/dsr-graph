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
	confParams  = std::make_shared<RoboCompCommonBehavior::ParameterList>(params);
	agent_name = params["agent_name"].value;
	agent_id = stoi(params["agent_id"].value);

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
		G = std::make_shared<CRDT::CRDTGraph>(0, agent_name, agent_id); // Init nodes
		std::cout<< __FUNCTION__ << "Graph loaded" << std::endl;  

		// Graph viewer
		using opts = DSR::GraphViewer::view;
		graph_viewer = std::make_unique<DSR::GraphViewer>(this, G, opts::scene|opts::graph|opts::tree|opts::osg);

		//Inner Api
		innermodel = G->get_inner_api();


		//Custom widget
		custom_widget.show();
		connect(custom_widget.autoMov_checkbox, SIGNAL(clicked()),this, SLOT(checkRobotAutoMovState()));
    	connect(custom_widget.robotMov_checkbox, SIGNAL(clicked()),this, SLOT(moveRobot()));

    	connect(custom_widget.ki_slider, SIGNAL (valueChanged(int)),this,SLOT(forcesSliderChanged(int)));
    	connect(custom_widget.ke_slider, SIGNAL (valueChanged(int)),this,SLOT(forcesSliderChanged(int)));

    	connect(custom_widget.send_button, SIGNAL(clicked()),this, SLOT(sendRobotTo()));

	    forcesSliderChanged();
    	moveRobot();

		navigation.initialize(G, confParams);
	
		this->Period = period;
//		timer.start(Period);
	}
}

void SpecificWorker::compute()
{
	bool needsReplaning = false;

    if(personalSpacesChanged)
	{
    	getPersonsFromModel();
        auto [intimate,personal,social] = getPolylinesFromModel();
    	navigation.updatePersonalPolylines(intimate,personal,social);
		personalSpacesChanged = false;
		needsReplaning = true;
	}

	if(affordancesChanged)
	{
        auto [mapCostObjects, totalAffordances, blockedAffordances] = getAffordancesFromModel();

        navigation.updateAffordancesPolylines(mapCostObjects,totalAffordances,blockedAffordances);
		affordancesChanged = false;
		needsReplaning = true;
	}

    
    RoboCompLaser::TLaserData laserData = updateLaser();

	navigation.update(totalPersons, laserData, needsReplaning);

//    static QTime reloj = QTime::currentTime();

//    viewer->run();
//    qDebug()<< "viewer " << reloj.restart();


	if (navigation.isCurrentTargetActive())
		checkHumanBlock();
	
	
}


void SpecificWorker::checkHumanBlock()
{
	/*QMutexLocker lockIM(mutex);

	AGMModel::SPtr newModel = AGMModel::SPtr(new AGMModel(worldModel));

	bool edgesChanged = false;

	auto blockingIDs = navigation.blockIDs;
    auto softBlockingIDs = navigation.softBlockIDs;

    if((prev_blockingIDs != blockingIDs) or (prev_softBlockingIDs != softBlockingIDs))
    {

		qDebug()<< "blocking - prev: " << prev_blockingIDs << " current: " << blockingIDs;
		qDebug()<< "SOFT blocking - prev: " << prev_softBlockingIDs << " current: " << softBlockingIDs;

        auto robotID = newModel->getIdentifierByType("robot");

        //////////////////////// block /////////////////////////////

        string edgeName;

		vector<string> edgeNames{"block" , "strongInterBlock"};

		for (auto edgeName : edgeNames)
		{
			for(auto id: prev_blockingIDs)
			{
				try
				{
					newModel->removeEdgeByIdentifiers(id, robotID, edgeName);
					qDebug ()<<" Se elimina el enlace " << QString::fromStdString(edgeName) << " de " << id;
				}

				catch(...)
				{
					std::cout<<__FUNCTION__<<"No existe el enlace"<<std::endl;

				}
			}

		}

        if(blockingIDs.size() == 1) edgeName = "block";
        else edgeName = "strongInterBlock";

        for(auto id: blockingIDs)
        {
            try
            {
                newModel->addEdgeByIdentifiers(id, robotID, edgeName);
				qDebug ()<<" Se añade el enlace " << QString::fromStdString(edgeName) << " de " << id;
            }

            catch(...)
            {
                std::cout<<__FUNCTION__<<"No existe el enlace"<<std::endl;

            }
        }

        ////////////////////////// softBlock //////////////////////////////

        for(auto groupID: prev_softBlockingIDs)
        {
			vector<string> edgeNames{"softBlock" , "softInterBlock"};

			for (auto edgeName : edgeNames)
			{
				for(auto id : groupID)
				{
					try
					{
						newModel->removeEdgeByIdentifiers(id, robotID, edgeName);
						qDebug ()<<" Se elimina el enlace " << QString::fromStdString(edgeName) << " de " << id;
					}

					catch(...)
					{
						std::cout<<__FUNCTION__<<"No existe el enlace"<<std::endl;

					}
				}
			}
        }

        for(auto groupID: softBlockingIDs)
        {
			if(groupID.size() == 1) edgeName = "softBlock";
			else edgeName = "softInterBlock";

        	for(auto id : groupID)
			{
				try
				{
					newModel->addEdgeByIdentifiers(id, robotID, edgeName);
					qDebug ()<<" Se añade el enlace " << QString::fromStdString(edgeName) << " de " << id;
				}

				catch(...)
				{
					std::cout<<__FUNCTION__<<"Ya existe el enlace"<<std::endl;

				}
			}

        }

        ///////////////////////////////////////////////////////////////////

        prev_blockingIDs = blockingIDs;
        prev_softBlockingIDs = softBlockingIDs;

		edgesChanged = true;
    }

    else
		edgesChanged = false;

	if(edgesChanged){
		try
		{
			sendModificationProposal(worldModel, newModel);
		}
		catch(...)
		{
			std::cout<<"No se puede actualizar worldModel"<<std::endl;
		}
	}
*/
}

RoboCompLaser::TLaserData  SpecificWorker::updateLaser()
{
//	qDebug()<<__FUNCTION__;

	RoboCompLaser::TLaserData laserData;
/*TODO
    try
    {
		laserData  = laser_proxy->getLaserData();
    }

    catch(const Ice::Exception &e){ std::cout <<"Can't connect to laser --" <<e.what() << std::endl; };
*/
    return laserData;
}


void SpecificWorker::getPersonsFromModel()
{
/*	totalPersons.clear();
	auto vectorPersons = worldModel->getSymbolsByType("person");

	for (auto p: vectorPersons) {
		localPerson person;

		auto id = p->identifier;
		AGMModelSymbol::SPtr personParent = worldModel->getParentByLink(id, "RT");
		AGMModelEdge& edgeRT = worldModel->getEdgeByIdentifiers(personParent->identifier, id, "RT");

		person.id = id;
		person.x = str2float(edgeRT.attributes["tx"]);
		person.z = str2float(edgeRT.attributes["tz"]);
		person.angle = str2float(edgeRT.attributes["ry"]);

		totalPersons.push_back(person);
	}*/
}

SpecificWorker::retPersonalSpaces SpecificWorker::getPolylinesFromModel()
{
/*	qDebug()<<__FUNCTION__;

    vector<QPolygonF> intimatePolygon;
    vector<QPolygonF> personalPolygon;
    vector<QPolygonF> socialPolygon;

    vector <vector<QPolygonF>> polylinesSeq {intimatePolygon, personalPolygon, socialPolygon};

    auto personalSpaces = worldModel->getSymbolsByType("personalSpace");
    vector<int> IDsAlreadyIncluded;

    for( auto space : personalSpaces)
    {
        int owner = -1;

        for (AGMModelSymbol::iterator edge = space->edgesBegin(worldModel);
             edge!=space->edgesEnd(worldModel);
             edge++) {
            if (edge->getLabel()=="has") {
                const std::pair<int32_t, int32_t> symbolPair = edge->getSymbolPair();
                owner = symbolPair.first;
            }
        }

        QString intimate = QString::fromStdString(space->getAttribute("intimate"));
        QString personal = QString::fromStdString(space->getAttribute("personal"));
        QString social = QString::fromStdString(space->getAttribute("social"));

        QString sharedWith = QString::fromStdString(space->getAttribute("sharedWith"));
        vector<int> sharedWithIDs;

        for (auto sh : sharedWith.split(" ")) {
            if (sh.toInt() != 0)
                sharedWithIDs.push_back(sh.toInt());
        }

        if(!sharedWithIDs.empty())
        {
            qDebug()<< "Searching for " << owner;
            if( std::find(std::begin(IDsAlreadyIncluded), std::end(IDsAlreadyIncluded), owner) != std::end(IDsAlreadyIncluded))
                continue;
        }


        vector<QString> polylinesStr = {intimate,personal,social};

        for (auto &&[str, polygonSeq] : iter::zip(polylinesStr, polylinesSeq))
        {
            for(auto pol: str.split(";;"))
            {
                if(pol.size() == 0)
                    continue;

                QPolygonF polygon;

                for (auto pxz : pol.split(";"))
                {
                    auto p = pxz.split(" ");

                    if (p.size() != 2)
                        continue;

                    auto x = std::stof(p[0].toStdString());
                    auto z = std::stof(p[1].toStdString());

                    polygon << QPointF(x,z);
                }

                polygonSeq.push_back(polygon);
            }
		}


		for(auto id : sharedWithIDs) {
            if (std::find(std::begin(IDsAlreadyIncluded), std::end(IDsAlreadyIncluded), id)==std::end(IDsAlreadyIncluded)) {
                IDsAlreadyIncluded.push_back(id);
            }
        }


    }
    return std::make_tuple(polylinesSeq[0],polylinesSeq[1],polylinesSeq[2]);
*/
}

SpecificWorker::retAffordanceSpaces SpecificWorker::getAffordancesFromModel()
{
/*
    qDebug()<<__FUNCTION__;

    vector<QPolygonF> totalAffordances;
    vector<QPolygonF> blockedAffordances;
    std::map<float,vector<QPolygonF>> mapCostObjects;

    auto affordanceSpaces = worldModel->getSymbolsByType("affordanceSpace");


    for( auto affordance : affordanceSpaces)
    {
        QPolygonF object;

        QString polyline = QString::fromStdString(affordance->getAttribute("affordance"));
        float cost = std::stof(affordance->getAttribute("cost"));
        bool interacting = (affordance->getAttribute("interacting") == "1");

        for(auto pol: polyline.split(";;"))
        {
            if(pol.size() == 0)
                continue;

            for (auto pxz : pol.split(";"))
            {
                auto p = pxz.split(" ");

                if (p.size() != 2)
                    continue;

                auto x = std::stof(p[0].toStdString());
                auto z = std::stof(p[1].toStdString());

                object<< QPointF(x,z);
            }

            mapCostObjects[cost].push_back(object);
            totalAffordances.push_back(object);
            if(interacting) blockedAffordances.push_back(object);

        }
    }


    qDebug()<< "END "<< __FUNCTION__;

    return std::make_tuple(mapCostObjects,totalAffordances,blockedAffordances);*/
}


void  SpecificWorker::moveRobot()
{
    qDebug()<<__FUNCTION__;

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


void SpecificWorker::sendRobotTo()
{
    auto x =  custom_widget.x_spinbox->value();
    auto z =  custom_widget.z_spinbox->value();

    navigation.newTarget(QPointF(x,z));

}


void SpecificWorker::
forcesSliderChanged(int value)
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


//SUBSCRIPTION to objectsChanged method from SocialRules interface
void SpecificWorker::SocialRules_objectsChanged(RoboCompSocialRules::SRObjectSeq objectsAffordances)
{
 	qDebug() << __FUNCTION__ << objectsAffordances.size();

    objects_seq = objectsAffordances;

	affordancesChanged = true;

}

//SUBSCRIPTION to personalSpacesChanged method from SocialRules interface
void SpecificWorker::SocialRules_personalSpacesChanged(RoboCompSocialNavigationGaussian::SNGPolylineSeq intimateSpaces, RoboCompSocialNavigationGaussian::SNGPolylineSeq personalSpaces, RoboCompSocialNavigationGaussian::SNGPolylineSeq socialSpaces)
{
	qDebug() << __FUNCTION__;

	intimate_seq = intimateSpaces;
	personal_seq = personalSpaces;
	social_seq = socialSpaces;

	personalSpacesChanged = true;

}



/**************************************/
// From the RoboCompSocialRules you can use this types:
// RoboCompSocialRules::SRObject

