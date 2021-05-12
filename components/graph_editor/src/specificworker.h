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

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <QLineEdit>
#include <QInputDialog>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <genericworker.h>
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"
#include "GraphEditorView.h"
#include "QAttributeBrowser.h"



class SpecificWorker : public GenericWorker
{
Q_OBJECT

public:
    std::shared_ptr<DSR::DSRGraph> G;
    std::string agent_name;

private:
    bool startup_check_flag;
    int agent_id{};
    std::string dsr_input_file;
    std::string dsr_output_file;
    std::string test_output_file;
    std::string dsr_output_path;

    // graph viewer
    std::unique_ptr<DSR::DSRViewer> dsr_viewer;
    GraphEditorView *editor_view{};
    QAttributeBrowser *attribute_browser{};
    QTemporaryFile new_graph_file;
public:
    SpecificWorker(TuplePrx tprx, bool startup_check);
    ~SpecificWorker();
    bool setParams(RoboCompCommonBehavior::ParameterList params) override;

public slots:
    void compute() override;
    void initialize(int period) override;


private:
    int tree_view{};
    int graph_view{};
    int qscene_2d_view{};
    int osg_3d_view{};


};
#endif
