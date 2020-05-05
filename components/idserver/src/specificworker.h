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
	SpecificWorker(TuplePrx tprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void initialize();
	
	// DSR
	std::shared_ptr<CRDT::CRDTGraph> G;
	std::string agent_name;

	//Interface DSRGetID
	int DSRGetID_getID();

public slots:
	void compute();
	void initialize(int period);

private:
	std::unique_ptr<DSR::GraphViewer> graph_viewer;
	int node_id = -9999;

	//params
	int agent_id;
	std::string dsr_input_file;
	std::string dsr_output_path;
	int output_file_count = 0;
	bool dsr_write_to_file;

	void get_max_id_from_G();

};

#endif
