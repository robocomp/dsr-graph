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

    // check if path given in config for dsr_input_file
    if (QFileInfo::exists(QString::fromStdString(dsr_input_file)))
    {
        // create graph with input file
        G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, dsr_input_file);
    }
    else {
        // If not
        // Ask if want to create new file or try to load from the network
        // Crreate new tempfile
        if (this->new_graph_file.open()) {
            // file.fileName() returns the unique file name
            QTextStream outStream(&this->new_graph_file);
            outStream << "{}";
            G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, this->new_graph_file.fileName().toStdString());
        }


        // Create empty json file in path
    }


    rt = G->get_rt_api();
    // Graph viewer
    //	graph_viewer = std::make_unique<DSR::GraphViewer>(G);

    //initialize node combobox
    auto map = G->getCopy();
	for(const auto &[k, node] : map)
    {
        QVariant data;
        data.setValue(node);
	    this->node_cb->addItem(QString::fromStdString(node.name()), data);
        node_combo_names[node.id()] = QString::fromStdString(node.name());
        //edges
        for(const auto &[key, edge] : node.fano())
        {
            QVariant edge_data;
            edge_data.setValue(edge);
            QString from = QString::fromStdString(G->get_node(edge.from()).value().name());
            QString to = QString::fromStdString(G->get_node(edge.to()).value().name());
            QString name = from + "_" + to + "_" + QString::fromStdString(edge.type());
            this->edge_cb->addItem(name, edge_data);
            edge_combo_names[std::to_string(edge.from())+"_"+std::to_string(edge.to())+"_"+edge.type()] = name;
        }
    }

    node_attrib_tw->setColumnCount(2);
    edge_attrib_tw->setColumnCount(2);
    QStringList horzHeaders;
    horzHeaders <<"Attribute"<< "Value";
    node_attrib_tw->setHorizontalHeaderLabels( horzHeaders );
    node_attrib_tw->verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);
    node_attrib_tw->verticalHeader()->setDefaultSectionSize(40);
    node_attrib_tw->setSelectionBehavior(QAbstractItemView::SelectRows);

    edge_attrib_tw->setHorizontalHeaderLabels( horzHeaders );
    edge_attrib_tw->verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);
    edge_attrib_tw->verticalHeader()->setDefaultSectionSize(40);
    edge_attrib_tw->setSelectionBehavior(QAbstractItemView::SelectRows);

//    connect(node_cb, SIGNAL(currentIndexChanged(int)), this, SLOT(change_node_slot(int)));
//    connect(save_node_pb, SIGNAL(clicked()), this, SLOT(save_node_clicked()));
//    connect(edge_cb, SIGNAL(currentIndexChanged(int)), this, SLOT(change_edge_slot(int)));
//    connect(save_edge_pb, SIGNAL(clicked()), this, SLOT(save_edge_slot()));
//    connect(del_node_pb, SIGNAL(clicked()), this, SLOT(delete_node_slot()));
//    connect(del_edge_pb, SIGNAL(clicked()), this, SLOT(delete_edge_slot()));
//    connect(new_node_pb, SIGNAL(clicked()), this, SLOT(new_node_slot()));
//    connect(new_edge_pb, SIGNAL(clicked()), this, SLOT(new_edge_slot()));
//    connect(new_node_attrib_pb, SIGNAL(clicked()), this, SLOT(new_node_attrib_slot()));
//    connect(new_edge_attrib_pb, SIGNAL(clicked()), this, SLOT(new_edge_attrib_slot()));
//    connect(del_node_attrib_pb, SIGNAL(clicked()), this, SLOT(del_node_attrib_slot()));
//    connect(del_edge_attrib_pb, SIGNAL(clicked()), this, SLOT(del_edge_attrib_slot()));

    //G signals
//    connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::G_add_or_assign_node_slot);
//    connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::G_add_or_assign_edge_slot);
//    connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::G_del_edge_slot);
//    connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::G_del_node_slot);
	
	// create graph
    std::cout << __FUNCTION__ << "Graph loaded" << std::endl;

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
    connect(editor_view, &GraphEditorView::graph_node_clicked,
            attribute_browser, &QAttributeBrowser::update_node_combobox);


}



void SpecificWorker::compute()
{

}


void SpecificWorker::G_add_or_assign_node_slot(const std::uint64_t id, const std::string &type) {
//    Node node = G->get_node(id).value();
//    QVariant data;
//    data.setValue(node);
//    int pos = this->node_cb->findText(QString::fromStdString(node.name()));
//    if (pos == -1) { //insert item
//        this->node_cb->addItem(QString::fromStdString(node.name()), data);
//        node_combo_names[node.id()] = QString::fromStdString(node.name());
//    }
//    else //update item
//        this->node_cb->setItemData(pos, data);
}
void SpecificWorker::G_add_or_assign_edge_slot(const std::uint64_t from, const std::uint64_t to, const std::string& type){
//    Edge edge = G->get_edge(from, to, type).value();
//    QVariant edge_data;
//    edge_data.setValue(edge);
//    QString from_s = QString::fromStdString(G->get_node(edge.from()).value().name());
//    QString to_s = QString::fromStdString(G->get_node(edge.to()).value().name());
//    QString name = from_s + "_" + to_s + "_" + QString::fromStdString(type);
//
//    int pos = this->edge_cb->findText(name);
//    if (pos == -1) { //insert item
//        this->edge_cb->addItem(name, edge_data);
//        edge_combo_names[std::to_string(edge.from())+"_"+std::to_string(edge.to())+"_"+edge.type()] = name;
//    }
//    else //update item
//        this->edge_cb->setItemData(pos, edge_data);

}
void SpecificWorker::G_del_node_slot(const std::uint64_t id){
//    QString combo_name = node_combo_names[id];
//    qDebug()<<"G_del_node"<<id<<combo_name;
//    int pos = this->node_cb->findText(combo_name);
//    if (pos != -1) {
//        this->node_cb->removeItem(pos);
//        node_combo_names.erase(id);
//    }
}
void SpecificWorker::G_del_edge_slot(const std::uint64_t from, const std::uint64_t to, const std::string &type){
//    QString combo_name = edge_combo_names[std::to_string(from)+"_"+std::to_string(to)+"_"+type];
//    int pos = this->edge_cb->findText(combo_name);
//    if (pos != -1)
//    {
//        this->edge_cb->removeItem(pos);
//        edge_combo_names.erase(std::to_string(from)+"_"+std::to_string(to)+"_"+type);
//    }
}
