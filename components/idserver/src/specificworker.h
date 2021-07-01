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
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"
#include <QHBoxLayout>

// Use (void) to silent unused warnings.
#define assertm(exp, msg) assert(((void)msg, exp))


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);


	//Interface DSRGetID
	int DSRGetID_getID();
private:
    void initialize();
    void check_rt_tree(const DSR::Node &node);

public slots:
	void compute();
	void initialize(int period);
    int startup_check();

private:
    std::shared_ptr<DSR::DSRGraph> G;
    std::string agent_name;
    bool startup_check_flag;
    std::unique_ptr<DSR::DSRViewer> dsr_viewer;
    std::unique_ptr<DSR::AgentInfoAPI> dsr_agent_info;
	QHBoxLayout mainLayout;
	QWidget window;
    uint64_t node_id = 0;

	//params
	int agent_id;
	std::string dsr_input_file;
	std::string dsr_output_path;
	int output_file_count = 0;
	bool dsr_write_to_file;
    int tree_view;
    int graph_view;
    int qscene_2d_view;
    int osg_3d_view;

	void get_max_id_from_G();
};

#endif
