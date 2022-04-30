/*
 *    Copyright (C) 2022 by YOUR NAME HERE
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
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"
#include <doublebuffer/DoubleBuffer.h>
#include <variant>
#include <fstream>
#include <filesystem>

// The struct Overload can have arbitrary many base classes (Ts ...).
// It derives from each class public and brings the call operator (Ts::operator...) of each base class into its scope.
//The base classes need an overloaded call operator (Ts::operator()).
//Lambdas provide this call operator.
template<typename ... Ts>
struct Overload : Ts ... { using Ts::operator() ...; };
template<class... Ts> Overload(Ts...) -> Overload<Ts...>;   // custom deduction rule

class SpecificWorker : public GenericWorker
{
    Q_OBJECT
    public:
        SpecificWorker(TuplePrx tprx, bool startup_check);
        ~SpecificWorker();
        bool setParams(RoboCompCommonBehavior::ParameterList params);

    public slots:
        void compute();
        int startup_check();
        void initialize(int period);
        void modify_node_slot(std::uint64_t, const std::string &type){};
        void modify_node_attrs_slot(std::uint64_t id, const std::vector<std::string>& att_names);
        void modify_edge_attrs_slot(std::uint64_t from, std::uint64_t to, const std::string &type, const std::vector<std::string>& att_names);
        void modify_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type){};
        void del_edge_slot(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){};
        void del_node_slot(std::uint64_t from){};

    private:
        // DSR graph
        std::shared_ptr<DSR::DSRGraph> G;
        std::unique_ptr<DSR::AgentInfoAPI> agent_info_api;

        //DSR params
        std::string agent_name;
        int agent_id;

        bool tree_view;
        bool graph_view;
        bool qscene_2d_view;
        bool osg_3d_view;

        // DSR graph viewer
        std::unique_ptr<DSR::DSRViewer> graph_viewer;
        QHBoxLayout mainLayout;

        bool startup_check_flag;

        std::ofstream log_file;
        std::string file_name = "log_file.txt";
        bool display = true;
        std::filesystem::path path;
        std::vector<std::string> list_of_excluded_nodes, list_of_excluded_edges;
        std::string visitor(DSR::ValType &var, unsigned int vector_elements_to_write = 0);
        std::uint64_t line_counter = 0;

};

#endif
