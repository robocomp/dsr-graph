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
#include "specificworker.h"
#include <thread>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx) : GenericWorker(tprx) {
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

    agent_id = stoi(params["agent_id"].value);
    dsr_output_file = params["dsr_output_file"].value;
    //dsr_input_file = params["dsr_input_file"].value;
    test_output_file = params["test_output_file"].value;
    dsr_input_file = params["dsr_input_file"].value;    
    return true;
}

void SpecificWorker::initialize(int period) 
{
    std::cout << "Initialize worker" << std::endl;

    // create graph
    G = std::make_shared<CRDT::CRDTGraph>(0, agent_name, agent_id, ""); // Init nodes

     
    // Graph viewer
//	graph_viewer = std::make_unique<DSR::GraphViewer>(G);

    //initialize node combobox
    auto map = G->getCopy();
	for(const auto &[k, node] : map)
    {
        QVariant data;
        data.setValue(node);
	    this->node_cb->addItem(QString::fromStdString(node.name()), data);

        //edges
        for(const auto &[key, edge] : node.fano())
        {
            QVariant edge_data;
            edge_data.setValue(edge);
            QString from = QString::fromStdString(G->get_node(edge.from()).value().name());
            QString to = QString::fromStdString(G->get_node(edge.to()).value().name());
            QString name = from + "_" + to + "_" + QString::fromStdString(edge.type());
            this->edge_cb->addItem(name, edge_data);
        }

    }

    node_attrib_tw->setColumnCount(2);
    edge_attrib_tw->setColumnCount(2);
    QStringList horzHeaders;
    horzHeaders <<"Attribute"<< "Value";
    node_attrib_tw->setHorizontalHeaderLabels( horzHeaders );
    node_attrib_tw->horizontalHeader()->setStretchLastSection(true);

    edge_attrib_tw->setHorizontalHeaderLabels( horzHeaders );
    edge_attrib_tw->horizontalHeader()->setStretchLastSection(true);

    connect(node_cb, SIGNAL(currentIndexChanged(int)), this, SLOT(change_node_slot(int)));
    connect(save_node_pb, SIGNAL(clicked()), this, SLOT(save_node_slot()));
    connect(edge_cb, SIGNAL(currentIndexChanged(int)), this, SLOT(change_edge_slot(int)));
    connect(save_edge_pb, SIGNAL(clicked()), this, SLOT(save_edge_slot()));
}


  

void SpecificWorker::compute()
{

}

void SpecificWorker::change_node_slot(int id)
{
    qDebug()<<id;
    Node node = node_cb->itemData(id).value<Node>();

    fill_table(node_attrib_tw, node.attrs());
}

void SpecificWorker::change_edge_slot(int id)
{
    Edge edge = edge_cb->itemData(id).value<Edge>();
    fill_table(edge_attrib_tw, edge.attrs());
}


void SpecificWorker::fill_table(QTableWidget *table_widget, std::map<std::string, Attrib> attribs)
{
    table_widget->setRowCount(0);
    for(auto [key, value] : attribs)
    {
        table_widget->insertRow( table_widget->rowCount() );
        table_widget->setItem(table_widget->rowCount()-1, 0, new QTableWidgetItem(QString::fromStdString(key)));
        QVariant variant;
        switch (value.value()._d())
        {
            case 0: 
                variant = QString::fromStdString(value.value().str());       
                break;
            case 1:
                variant = value.value().dec();
                break;
            case 2:
                variant = std::round(static_cast<double>(value.value().fl()) *1000000)/ 1000000 ;
                break;
            case 3:
                if (!value.value().float_vec().empty())
                {
                    QString aux;
                    for(std::size_t i = 0; i < value.value().float_vec().size(); ++i)
                    {
                        aux += QString::number(value.value().float_vec()[i]);
                        if (i != value.value().float_vec().size()-1)
                            aux += ",";
                    }
                    variant = aux;
                }
                break;
            case 4:
                variant = value.value().bl();
                break;
        }
        table_widget->setItem(table_widget->rowCount()-1, 1, new QTableWidgetItem(variant.toString()));
    }
}


std::map<std::string, Attrib> SpecificWorker::get_table_content(QTableWidget *table_widget, std::map<std::string, Attrib> attrs)
{
    for (int row=0; row < table_widget->rowCount(); row++)
    {
        std::string key = table_widget->item(row, 0)->text().toStdString();
        QString value = table_widget->item(row, 1)->text();
        switch(attrs[key].value()._d())
        {
            case 0: 
                attrs[key].value().str(value.toStdString());       
                break;
            case 1:
                attrs[key].value().dec(value.toInt());       
                break;
            case 2:
                attrs[key].value().fl(value.toFloat());       
                break;
            case 4:
                attrs[key].value().bl(value.contains("true"));       
                break;
            case 3:
                std::vector<float> r;
                for (auto element: value.split(','))
                    r.push_back(element.toFloat());
                
                attrs[key].value().float_vec(r);       
                break;                
        }       
    }
    return attrs;
}


void SpecificWorker::save_node_slot()
{
    Node node = node_cb->itemData(node_cb->currentIndex()).value<Node>();

    std::map<std::string, Attrib> new_attrs = get_table_content(node_attrib_tw, node.attrs());
    node.attrs(new_attrs);
    
    if(G->insert_or_assign_node(node))
        qDebug()<<"Node saved";
    else
    {
        qDebug()<<"Error saving node";
    }
}


void SpecificWorker::save_edge_slot()
{
    Edge edge = edge_cb->itemData(edge_cb->currentIndex()).value<Edge>();

    std::map<std::string, Attrib> new_attrs = get_table_content(edge_attrib_tw, edge.attrs());
    edge.attrs(new_attrs);
    
    if(G->insert_or_assign_edge(edge))
        qDebug()<<"Edge saved";
    else
    {
        qDebug()<<"Error saving edge";
    }
  
}