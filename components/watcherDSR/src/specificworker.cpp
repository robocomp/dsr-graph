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
	qRegisterMetaType<std::int32_t>("std::int32_t");
	qRegisterMetaType<std::string>("std::string");
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	G->write_to_json_file("./"+agent_name+"_final.json");
	G.reset();
	QFile jsonFile(QString::fromStdString("./"+agent_name+"_updates.json"));
	jsonFile.open(QFile::WriteOnly);
	QJsonDocument updates_json_doc(updates_json_object);
	jsonFile.write(compress(updates_json_doc.toJson()));
	jsonFile.close();
	auto now_c = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	qDebug() << __FILE__ << " " << __FUNCTION__ << "File: " << QString::fromStdString("./"+agent_name+"_updates.json")<< " written to disk at " << now_c;
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
		connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::add_or_assign_node_SLOT);
		connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::add_or_assign_edge_SLOT);
		connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_SLOT);
		connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_SLOT);
		std::cout<< __FUNCTION__ << "Graph loaded" << std::endl;
		G->write_to_json_file("./"+agent_name+"_initial.json");

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
		    main = opts::graph;
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

		this->Period = period;
		timer.start(Period);
	}

}

void SpecificWorker::compute()
{
	//computeCODE
	//QMutexLocker locker(mutex);
	//try
	//{
	//  camera_proxy->getYImage(0,img, cState, bState);
	//  memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
	//  searchTags(image_gray);
	//}
	//catch(const Ice::Exception &e)
	//{
	//  std::cout << "Error reading from Camera" << e << std::endl;
	//}
	
	
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

void SpecificWorker::add_or_assign_node_SLOT(const std::int32_t id, const std::string &type)
{
	auto node = G->get_node(id);
	if(node)
	{
		// TODO: Look for a better way to filter nodes
		if(node->type()!="rgbd" and node->type()!="laser")
		{
			QJsonObject json_node = DSR::Utilities::Node_to_QObject(node.value());
			QString timestamp_string =  QDateTime::currentDateTime().toString("yyyyMMdd@HHmmss.zzz");
			cout<<"Adding update: "<<timestamp_string.toStdString()<<endl;
			updates_json_object[timestamp_string] = json_node;
		}
	}
}
void SpecificWorker::add_or_assign_edge_SLOT(const std::int32_t from, const std::int32_t to, const std::string& type)
{
	auto edge = G->get_edge(from, to, type);
	if(edge)
	{
			QJsonObject json_edge = DSR::Utilities::Edge_to_QObject(edge.value());
			QString timestamp_string =  QDateTime::currentDateTime().toString("yyyyMMdd@HHmmss.zzz");
			cout<<"Adding edge update: "<<timestamp_string.toStdString()<<endl;
			updates_json_object[timestamp_string] = json_edge;
	}
}
void SpecificWorker::del_edge_SLOT(const std::int32_t from, const std::int32_t to,  const std::string &edge_tag)
{
	cout<<__FUNCTION__<<endl;
}
void SpecificWorker::del_node_SLOT(int id)
{
	cout<<__FUNCTION__<<endl;
}

QByteArray SpecificWorker::compress(const QByteArray& data)
{
	auto compressedData = qCompress(data);
	return compressedData;
}




