/*
 *    Copyright (C)2018 by YOUR NAME HERE
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
#include "../../graph-related-classes/graph.h"
#include "../../graph-related-classes/graphviewer.h"
#include "../../graph-related-classes/innermodelapi.h"
#include "../../graph-related-classes/graphcrdt.h"

class SpecificWorker : public GenericWorker
{
	Q_OBJECT
	public:
		SpecificWorker(TuplePrx mprx);
		~SpecificWorker();
		bool setParams(RoboCompCommonBehavior::ParameterList params);
		std::shared_ptr<DSR::Graph> getGraph() const { return graph;};
		

	public slots:
		void compute();
		void initialize(int period);
		void saveGraphSLOT()			{ graph->saveToFile("caca.xml");};

	signals:
		void addNodeSIGNAL(std::int32_t id, const std::string &name, const std::string &type, float posx, float posy, const std::string &color);
		void addEdgeSIGNAL(std::int32_t from, std::int32_t to, const std::string &ege_tag);

	private:
		InnerModel *innerModel;
		void walkTree(InnerModelNode *node);
		//void drawGraph();
		std::unique_ptr<DSR::GraphViewer> graph_viewer;
		InnerModelAPI innerapi;
		std::unique_ptr<DSR::GraphCRDT> gcrdt; 
		std::shared_ptr<DSR::Graph> graph;
};

#endif
