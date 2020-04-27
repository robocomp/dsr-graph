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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx) : GenericWorker(tprx)
{}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	try
	{
//		RoboCompCommonBehavior::Parameter par = params.at("DSRPath");
//		std::string dsr_path = par.value;
		
	}
	catch(const std::exception &e) { qFatal("Error reading config params"); }


	initialize();
	return true;
}

void SpecificWorker::initialize()
{
	QJsonObject json = read_json_file();
	qDebug()<<"Max value"<< get_max_id_from_json(json);
}

QJsonObject SpecificWorker::read_json_file()
{
	QFile file;
    file.setFileName("grafo.json");
    file.open(QIODevice::ReadOnly | QIODevice::Text);
    QString val = file.readAll();
    file.close();
	QJsonDocument doc = QJsonDocument::fromJson(val.toUtf8());
	QJsonObject jObject = doc.object();
	return jObject;
}

int SpecificWorker::get_max_id_from_json(QJsonObject jObject)
{
	int max_id = -9999;

	QJsonObject dsrobject = jObject.value("DSRModel").toObject();
	QJsonArray jsonArray = dsrobject.value("symbol").toArray();

	foreach (const QJsonValue & value, jsonArray) {
		 QJsonObject obj = value.toObject();
		 qDebug()<<"Value"<<obj.value("id");
		 if (obj.value("id").toString().toInt() > max_id)
		 	max_id = obj.value("id").toString().toInt();
	}
	return max_id;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	timer.start(Period);

}

void SpecificWorker::compute()
{
//computeCODE
//QMutexLocker locker(mutex);
//	try
//	{
//		camera_proxy->getYImage(0,img, cState, bState);
//		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
//		searchTags(image_gray);
//	}
//	catch(const Ice::Exception &e)
//	{
//		std::cout << "Error reading from Camera" << e << std::endl;
//	}
}




int SpecificWorker::DSRGetID_getID()
{
//implementCODE

}


