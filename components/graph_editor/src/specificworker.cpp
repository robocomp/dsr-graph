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
#include <thread>
#include <QFileInfo>


using namespace DSR;
/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
    this->startup_check_flag = startup_check;
//	QLoggingCategory::setFilterRules("*.debug=false\n");
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker() {
    std::cout << "Destroying SpecificWorker" << std::endl;
    //G.reset();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params) 
{

    try
    {
        agent_name = params["agent_name"].value;
        agent_id = stoi(params["agent_id"].value);
        dsr_output_file = params["dsr_output_file"].value;
        dsr_input_file = params["dsr_input_file"].value;
        dsr_output_path = params["dsr_output_path"].value;
        this->Period = stoi(params["period"].value);
        tree_view = (params["tree_view"].value == "true") ? DSR::DSRViewer::view::tree : 0;
        graph_view = (params["graph_view"].value == "true") ? DSR::DSRViewer::view::graph : 0;
        qscene_2d_view = (params["2d_view"].value == "true") ? DSR::DSRViewer::view::scene : 0;
        osg_3d_view = (params["3d_view"].value == "true") ? DSR::DSRViewer::view::osg : 0;
        test_output_file = params["test_output_file"].value;
        dsr_input_file = params["dsr_input_file"].value;
    }
    catch(const std::exception &e) { qFatal("Error reading config params"); }
    return true;

    //dsr_input_file = params["dsr_input_file"].value;
}

void SpecificWorker::initialize(int period) 
{
    qRegisterMetaType<std::uint32_t>("std::uint32_t");
    qRegisterMetaType<std::uint64_t>("std::uint64_t");
    qRegisterMetaType<std::string>("std::string");
    std::cout << "Initialize worker" << std::endl;
    std::optional<uint64_t>  new_world_node_id = std::nullopt;
    // check if path given in config for dsr_input_file
    if (QFileInfo::exists(QString::fromStdString(dsr_input_file)))
    {
        // create graph with input file
        open_dsr_file(dsr_input_file);
    }
    else {
       new_world_node_id = create_new_dsr_file();
    }

    // Graph viewer
    using opts = DSR::DSRViewer::view;
    int current_opts = tree_view | graph_view | qscene_2d_view | osg_3d_view;
    opts main = opts::none;
    if (graph_view)
    {
        main = opts::graph;
    }
    dsr_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
    setWindowTitle(QString::fromStdString(agent_name + "-" + dsr_input_file));
    editor_view = new GraphEditorView(G, this);
//    dsr_viewer->add_custom_widget_to_dock("Editor", editor_view);
//    TODO: set Editor as main
    this->setCentralWidget(editor_view);
    this->Period = 200;
    attribute_browser = new QAttributeBrowser(G, this);
    dsr_viewer->add_custom_widget_to_dock("Browser", attribute_browser );
    timer.start();
    connect(editor_view, &GraphEditorView::graph_node_clicked,
            attribute_browser, &QAttributeBrowser::update_node_combobox);

    if(new_world_node_id.has_value())
        attribute_browser->add_node_to_combobox(new_world_node_id.value());


}
void SpecificWorker::open_dsr_file(const string& dsr_file)
{
    if (QFileInfo::exists(QString::fromStdString(dsr_file)))
    {
        if(G!=nullptr)
            G->reset();
        G = make_shared<DSRGraph>(0, agent_name, agent_id, dsr_file);
    }
}

std::optional<uint64_t> SpecificWorker::create_new_dsr_file()
{// If not
// Ask if want to create new file or try to load from the network
// Create new tempfile
    if(G!=nullptr)
        G->reset();
    if (new_graph_file.open()) {
        // file.fileName() returns the unique file name
        QTextStream outStream(&new_graph_file);
        outStream << "{}";
        G = make_shared<DSRGraph>(0, agent_name, agent_id, new_graph_file.fileName().toStdString());
        DSR::Node node;
        node.type("world");
        node.name("World");
        this->G->add_or_modify_attrib_local<pos_x_att>(node, float(0));
        this->G->add_or_modify_attrib_local<pos_y_att>(node, float(0));
        this->G->add_or_modify_attrib_local<color_att>(node, std::string("GoldenRod"));
        G->add_or_modify_attrib_local<level_att>(node, 0);
        try
        {
            return this->G->insert_node(node);
        }
        catch(const std::exception& e)
        {
            std::cout << __FUNCTION__ <<  e.what() << std::endl;
            return std::nullopt;
        }
    }
    // Create empty json file in path
}

void SpecificWorker::compute()
{
    auto hzs = fps_counter.print("");
    dsr_viewer->set_external_hz(hzs);
}