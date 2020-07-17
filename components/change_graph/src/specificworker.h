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


class SpecificWorker : public GenericWorker
{
Q_OBJECT

	public:
		std::shared_ptr<DSR::DSRGraph> G;
		std::string agent_name;

	private:
		bool startup_check_flag;
		int agent_id;
		std::string dsr_input_file;
		std::string dsr_output_file;
		std::string test_output_file;
		
		// graph viewer
		std::unique_ptr<DSR::GraphViewer> graph_viewer;
        std::map<int, QString> node_combo_names;
        std::map<std::string, QString> edge_combo_names;
	public:
		SpecificWorker(TuplePrx tprx, bool startup_check);
		~SpecificWorker();
		bool setParams(RoboCompCommonBehavior::ParameterList params);

	public slots:
		void compute();
		void initialize(int period);
		void change_node_slot(int id);
		void save_node_slot();
		void delete_node_slot();
		void change_edge_slot(int id);
		void delete_edge_slot();
		void save_edge_slot();
        void new_edge_slot();
        void new_node_slot();
        void new_node_attrib_slot();
        void new_edge_attrib_slot();
        void del_node_attrib_slot();
        void del_edge_attrib_slot();

        //G signals
        void G_add_or_assign_node_slot(const std::int32_t id, const std::string &type);
        void G_add_or_assign_edge_slot(const std::int32_t from, const std::int32_t to, const std::string& type);
        void G_del_node_slot(const std::int32_t id);
        void G_del_edge_slot(const std::int32_t from, const std::int32_t to, const std::string &edge_tag);


	private:
		void fill_table(QTableWidget *table_widget, std::map<std::string, Attrib> attrib);
		std::map<std::string, Attrib> get_table_content(QTableWidget *table_widget, std::map<std::string, Attrib> attrs);


};
Q_DECLARE_METATYPE(Node);
Q_DECLARE_METATYPE(Edge);

#endif
