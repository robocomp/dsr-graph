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

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <random>
#include <thread>
#include <chrono>
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"


#define LAPS 50
#define NODES 1500

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);



    public slots:
		void autokill();
		void compute();
		int startup_check();
		void initialize(int period);

	signals:
		void addNodeSIGNAL(std::int32_t id, const std::string &name, const std::string &type, float posx, float posy, const std::string &color);
		void addEdgeSIGNAL(std::int32_t from, std::int32_t to, const std::string &ege_tag);

	private:
		InnerModel *innerModel;
		bool startup_check_flag;
		bool read_file;
		bool write_string;
		int agent_id;
		std::string agent_name;
		std::unique_ptr<DSR::DSRViewer> dsr_viewer;
        std::shared_ptr<DSR::DSRGraph> G;
        QHBoxLayout mainLayout;
        QWidget window;

		// Tests
		//void tester();
		//void test_nodes_mov();


		//void write_test_output(std::string result);
		std::string test_output_file;
		std::string dsr_output_file;
		std::string dsr_input_file;
        std::string dsr_test_file;
        std::string dsr_empty_test_file;
        std::string test_name;
		std::string MARKER = ";";

		//void test_node_random();
		// Random
		std::random_device rd;
		std::mt19937 mt;
		std::uniform_real_distribution<float> dist;
        std::uniform_int_distribution<int> randomNode, random_selector, node_selector, random_pos;

		//threadss
		std::vector<std::thread> threads;
		std::vector<int> created_nodos;
        std::vector<std::pair<int, int>> created_edges;

        pair<int, int> removeEdge();

        int newID();
		int removeID();
        int getID();
		std::mutex mut;
		QTimer autokill_timer;


		//std::shared_ptr<Test_utils> test;
		//CRDT_G_api_test G_api_test;
		//CRDT_insert_remove_node concurrent_test;
        //DSR_test dst_test;

};

#endif
