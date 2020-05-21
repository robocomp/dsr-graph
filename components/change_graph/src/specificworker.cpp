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
    }

    node_attrib_tw->setColumnCount(2);
    QStringList horzHeaders;
    horzHeaders <<"Attribute"<< "Value";
    node_attrib_tw->setHorizontalHeaderLabels( horzHeaders );

    connect(node_cb, SIGNAL(currentIndexChanged(int)), this, SLOT(change_node_slot(int)));
    //node_cb->currentData().toInt()
}


  

void SpecificWorker::compute()
{

}

void SpecificWorker::change_node_slot(int id)
{
    qDebug()<<id;
    node_attrib_tw->setRowCount(0);

    Node node = node_cb->itemData(id).value<Node>();
    
    for(auto [key, value] : node.attrs())
    {
        node_attrib_tw->insertRow( node_attrib_tw->rowCount() );
        node_attrib_tw->setItem(node_attrib_tw->rowCount()-1, 0, new QTableWidgetItem(QString::fromStdString(key)));
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
            case 4:
                variant = value.value().bl();
                break;
        }
        node_attrib_tw->setItem(node_attrib_tw->rowCount()-1, 1, new QTableWidgetItem(variant.toString()));

    }

    std::cout<<node.name()<<std::endl;
    //m_pTableWidget->setItem(0, 1, new QTableWidgetItem("Hello"));
    //itleComboBox->itemData(0)
}
