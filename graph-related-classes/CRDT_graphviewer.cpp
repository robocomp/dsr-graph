/*
 * Copyright 2018 <copyright holder> <email>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "CRDT_graphviewer.h"
#include <cppitertools/range.hpp>
#include <qmat/QMatAll>
#include <QDesktopWidget>
#include <QGLViewer/qglviewer.h>
#include <QApplication>
#include <QTableWidget>
#include "CRDT_graphnode.h"
#include "CRDT_graphedge.h"
#include "specificworker.h"

using namespace DSR;

GraphViewer::GraphViewer(QMainWindow * widget, std::shared_ptr<CRDT::CRDTGraph> G_, int options, view main) : QObject()
{
	G = G_;
    qRegisterMetaType<std::int32_t>("std::int32_t");
    qRegisterMetaType<std::string>("std::string");
 	QRect availableGeometry(QApplication::desktop()->availableGeometry());
 	this->window = widget;
 	window->move((availableGeometry.width() - window->width()) / 2, (availableGeometry.height() - window->height()) / 2);
	
	// QSettings settings("RoboComp", "DSR");
    // settings.beginGroup("MainWindow");
    // 	graphicsView->resize(settings.value("size", QSize(400, 400)).toSize());
    // 	graphicsView->move(settings.value("pos", QPoint(200, 200)).toPoint());
    // settings.endGroup();
	// settings.beginGroup("QGraphicsView");
	// 	graphicsView->setTransform(settings.value("matrix", QTransform()).value<QTransform>());
	// settings.endGroup();

	//MenuBar
    viewMenu = window->menuBar()->addMenu(window->tr("&View"));

	initialize_views(options, main);
}

GraphViewer::~GraphViewer()
{
	QSettings settings("RoboComp", "DSR");
    settings.beginGroup("MainWindow");
		settings.setValue("size", window->size());
		settings.setValue("pos", window->pos());
    settings.endGroup();
}


void GraphViewer::initialize_views(int options, view central){
	//Create docks view and main widget
	std::map<view,QString> valid_options{{view::graph,"Graph"}, {view::tree,"Tree"}, {view::osg,"3D"}, {view::scene, "2D"}};

	for (auto option: valid_options) {
		auto viewer = create_widget(option.first);
		if(option.first == central)
		{
			window->setCentralWidget(viewer);
			this->main_widget = viewer;
		}
		else if(options & option.first ) {
			create_dock_and_menu(QString(option.second), viewer);
		}
	}

	QDockWidget * previous = nullptr;
	for(auto dock: docks) {
		if (previous)
			window->tabifyDockWidget(previous, dock.second);
		previous = dock.second;
	}
	qDebug()<<(docks.count(QString("Tree"))==1)<<(qobject_cast<DSRtoGraphViewer*>(this->main_widget));
	if(docks.count(QString("Tree"))==1)
	{
		auto graph_widget = qobject_cast<DSRtoGraphViewer*>(this->main_widget);
		if(graph_widget)
		{
			DSRtoTreeViewer * tree_widget= qobject_cast<DSRtoTreeViewer*>(docks["Tree"]->widget());
			GraphViewer::connect(
					tree_widget,
					&DSRtoTreeViewer::node_check_state_changed,
					graph_widget,
					[=] (int value, int id, const std::string &type, QTreeWidgetItem*) {graph_widget->hide_show_node_SLOT(id, value==2);});
		}

	}
}


QWidget* GraphViewer::create_widget(view type){

	QWidget * widget_view = nullptr;
	switch(type) {
//		graph
		case view::graph:
			widget_view = new DSR::DSRtoGraphViewer(G);
			break;
//		3D
		case view::osg:
			widget_view = new DSR::DSRtoOSGViewer(G, 1, 1);
			break;
//		Tree
		case view::tree:
			widget_view = new DSR::DSRtoTreeViewer(G);
			break;
//		2D
		case view::scene:
			widget_view = new DSR::DSRtoGraphicsceneViewer(G);
			break;
	}
	return widget_view;
}

void GraphViewer::create_dock_and_menu(QString name, QWidget* view){
//	TODO: Check if name exists in docks
	QDockWidget* dock_widget;
	if(this->docks.count(name)) {
		dock_widget = this->docks[name];
		window->removeDockWidget(dock_widget);
	}
	else{
		dock_widget = new QDockWidget(name);
		viewMenu->addAction(dock_widget->toggleViewAction());
		this->docks[name] = dock_widget;
	}
	dock_widget->setWidget(view);
	window->addDockWidget(Qt::RightDockWidgetArea, dock_widget);
}



////////////////////////////////////////
/// UI slots
////////////////////////////////////////
void GraphViewer::saveGraphSLOT()
{ 
	emit saveGraphSIGNAL(); 
}

void GraphViewer::toggleSimulationSLOT()
{
	this->do_simulate = !do_simulate;
	if(do_simulate)
	   timerId = window->startTimer(1000 / 25);
}

///////////////////////////////////////

void GraphViewer::itemMoved()
{
	//std::cout << "timerId " << timerId << std::endl;
	//if(do_simulate and timerId == 0)
    //if (timerId == 0)
    //   timerId = startTimer(1000 / 25);
}

void GraphViewer::timerEvent(QTimerEvent *event)
{
    // Q_UNUSED(event)

	// for( auto &[k,node] : gmap)
	// {
	// 	(void)k;
	//     node->calculateForces();
	// }
	// bool itemsMoved = false;
	
	// for( auto &[k,node] : gmap)
	// {
	// 	(void)k;
    //     if (node->advancePosition())
    //         itemsMoved = true;
    // }
	// if (!itemsMoved) 
	// {
    //     killTimer(timerId);
    //     timerId = 0;
    // }
}

/////////////////////////
///// Qt Events
/////////////////////////

void GraphViewer::keyPressEvent(QKeyEvent* event) 
{
	if (event->key() == Qt::Key_Escape)
		emit closeWindowSIGNAL();
}

