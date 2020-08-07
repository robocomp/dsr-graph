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
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx, startup_check) {
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
    G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, "", dsrgetid_proxy); // Init nodes

     
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

    connect(node_cb, SIGNAL(currentIndexChanged(int)), this, SLOT(change_node_slot(int)));
    connect(save_node_pb, SIGNAL(clicked()), this, SLOT(save_node_slot()));
    connect(edge_cb, SIGNAL(currentIndexChanged(int)), this, SLOT(change_edge_slot(int)));
    connect(save_edge_pb, SIGNAL(clicked()), this, SLOT(save_edge_slot()));
    connect(del_node_pb, SIGNAL(clicked()), this, SLOT(delete_node_slot()));
    connect(del_edge_pb, SIGNAL(clicked()), this, SLOT(delete_edge_slot()));
    connect(new_node_pb, SIGNAL(clicked()), this, SLOT(new_node_slot()));
    connect(new_edge_pb, SIGNAL(clicked()), this, SLOT(new_edge_slot()));
    connect(new_node_attrib_pb, SIGNAL(clicked()), this, SLOT(new_node_attrib_slot()));
    connect(new_edge_attrib_pb, SIGNAL(clicked()), this, SLOT(new_edge_attrib_slot()));
    connect(del_node_attrib_pb, SIGNAL(clicked()), this, SLOT(del_node_attrib_slot()));
    connect(del_edge_attrib_pb, SIGNAL(clicked()), this, SLOT(del_edge_attrib_slot()));

    //G signals
    connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::G_add_or_assign_node_slot);
    connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::G_add_or_assign_edge_slot);
    connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::G_del_edge_slot);
    connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::G_del_node_slot);
}


  

void SpecificWorker::compute()
{

}

void SpecificWorker::change_node_slot(int id)
{
    Node node = node_cb->itemData(id).value<Node>();
    node_id_label->setText("("+QString::number(node.id())+":"+QString::fromStdString(node.type())+")");
    fill_table(node_attrib_tw, node.attrs());
}

void SpecificWorker::change_edge_slot(int id)
{
    Edge edge = edge_cb->itemData(id).value<Edge>();
    edge_id_label->setText("("+QString::number(edge.from())+"-"+QString::number(edge.to())+":"+QString::fromStdString(edge.type())+")");
    fill_table(edge_attrib_tw, edge.attrs());
}


void SpecificWorker::fill_table(QTableWidget *table_widget, std::map<std::string, Attrib> attribs)
{
    table_widget->setRowCount(0);
    for(auto [key, value] : attribs)
    {
        table_widget->insertRow( table_widget->rowCount() );
        QTableWidgetItem *item =new QTableWidgetItem(QString::fromStdString(key));
        item->setFlags(Qt::ItemIsSelectable|Qt::ItemIsEnabled);
        table_widget->setItem(table_widget->rowCount()-1, 0, item);
        switch (value.value()._d())
        {
            case 0:
                {
                    QLineEdit *ledit = new QLineEdit(QString::fromStdString(value.value().str()));
                    table_widget->setCellWidget(table_widget->rowCount()-1, 1, ledit);
                }
                break;
            case 1:
                {
                    QSpinBox *spin = new QSpinBox();
                    spin->setMinimum(-10000);
                    spin->setMaximum(10000);
                    spin->setValue(value.value().dec());
                    table_widget->setCellWidget(table_widget->rowCount()-1, 1, spin);
                }
                break;
            case 2:
                {
                    QDoubleSpinBox *spin = new QDoubleSpinBox();
                    spin->setMinimum(-10000);
                    spin->setMaximum(10000);
                    spin->setValue(std::round(static_cast<double>(value.value().fl()) *1000000)/ 1000000);
                    table_widget->setCellWidget(table_widget->rowCount()-1, 1, spin);
                }
                break;
            case 3:
                {
                    QWidget  *widget = new QWidget();
                    QHBoxLayout *layout = new QHBoxLayout;
                    widget->setLayout(layout);
                    if (!value.value().float_vec().empty())
                    {
                        for(std::size_t i = 0; i < value.value().float_vec().size(); ++i)
                        {
                            QDoubleSpinBox *spin = new QDoubleSpinBox();
                            spin->setMinimum(-10000);
                            spin->setMaximum(10000);
                            spin->setValue(value.value().float_vec()[i]);
                            layout->addWidget(spin);
                        }
                        table_widget->setCellWidget(table_widget->rowCount()-1, 1, widget);
                    }
                }
                break;
            case 4:
                {
                    QComboBox *combo = new QComboBox();
                    combo->addItem("true");
                    combo->addItem("false");
                    if (value.value().bl())
                        combo->setCurrentText("true");
                    else
                        combo->setCurrentText("false");
                    table_widget->setCellWidget(table_widget->rowCount()-1, 1, combo);
                }
                break;
        }
    }
    table_widget->resizeColumnsToContents();
    table_widget->horizontalHeader()->setStretchLastSection(true);
}


std::map<std::string, Attrib> SpecificWorker::get_table_content(QTableWidget *table_widget, std::map<std::string, Attrib> attrs)
{
    for (int row=0; row < table_widget->rowCount(); row++)
    {
        std::string key = table_widget->item(row, 0)->text().toStdString();
        switch(attrs[key].value()._d())
        {
            case 0:
                {
                    QLineEdit *ledit = (QLineEdit*)table_widget->cellWidget(row, 1);
                    attrs[key].value().str(ledit->text().toStdString());       
                }
                break;
            case 1:
                {
                    QSpinBox *spin = (QSpinBox*)table_widget->cellWidget(row, 1);
                    attrs[key].value().dec(spin->value());       
                }
                break;
            case 2:
                {
                    QDoubleSpinBox *spin = (QDoubleSpinBox*)table_widget->cellWidget(row, 1);
                    attrs[key].value().fl(spin->value());       
                }
                break;
            case 4:
                {
                    QComboBox *combo = (QComboBox*)table_widget->cellWidget(row, 1);
                    attrs[key].value().bl(combo->currentText().contains("true"));       
                }
                break;
            case 3:
                {
                    qDebug()<<__LINE__;
                    std::vector<float> r;
                    for (QDoubleSpinBox *spin: table_widget->cellWidget(row, 1)->findChildren<QDoubleSpinBox *>())
                    {
                        r.push_back(spin->value());
                    }
                    attrs[key].value().float_vec(r);
                    std::cout<<r;
                }
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
    
    if(G->update_node(node))
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

void SpecificWorker::delete_edge_slot()
{
    Edge edge = edge_cb->itemData(edge_cb->currentIndex()).value<Edge>();
    if (not G->delete_edge(edge.from(), edge.to(), edge.type()))
        qDebug()<<"Edge ("<<edge.from()<<"=>"<<edge.to()<<") could not be deleted";
}

void SpecificWorker::delete_node_slot()
{
    Node node = node_cb->itemData(node_cb->currentIndex()).value<Node>();
    if( not  G->delete_node(node.id()))
       qDebug()<<"Node"<<QString::fromStdString(node.name())<<"could not be deleted";
}


void SpecificWorker::new_node_slot()
{
    //get node type
    bool ok;
    QStringList items;
    items << tr("plane") << tr("transform") << tr("mesh") << tr("person")<< tr("omnirobot")<< tr("rgbd");
    QString node_type = QInputDialog::getItem(this, tr("New node"), tr("Attrib name:"), items, 0, false, &ok);
    if(not ok or node_type.isEmpty())
        return;
    
    Node node;
    node.type(node_type.toStdString());
    G->add_or_modify_attrib_local(node, "pos_x", 100.0);
    G->add_or_modify_attrib_local(node, "pos_y", 130.0);
    G->add_or_modify_attrib_local(node, "color", std::string("GoldenRod"));
    try
    {     
        G->insert_node(node);
    }
    catch(const std::exception& e)
    {
        std::cout << __FUNCTION__ <<  e.what() << std::endl;
    }
}

void SpecificWorker::new_node_attrib_slot()
{
    bool ok1, ok2;
    QString attrib_name = QInputDialog::getText(this, tr("New node attrib"),
                                         tr("Attrib name:"), QLineEdit::Normal,
                                         "name", &ok1);
    QStringList items;
    items << tr("int") << tr("float") << tr("string") << tr("bool");
    QString attrib_type = QInputDialog::getItem(this, tr("New node attrib"), tr("Attrib type:"), items, 0, false, &ok2);
    
    if(not ok1 or not ok2 or attrib_name.isEmpty() or attrib_type.isEmpty())
        return;
    
    Node node = node_cb->itemData(node_cb->currentIndex()).value<Node>();
    if(attrib_type == "int")
        G->insert_or_assign_attrib_by_name(node, attrib_name.toStdString(),0);
    else if(attrib_type == "float")
        G->insert_or_assign_attrib_by_name(node, attrib_name.toStdString(),0.0);
    else if(attrib_type == "bool")
        G->insert_or_assign_attrib_by_name(node, attrib_name.toStdString(),false);
    else if(attrib_type == "string")        
        G->insert_or_assign_attrib_by_name(node, attrib_name.toStdString(),std::string(""));
        
    fill_table(node_attrib_tw, node.attrs());    
}

void SpecificWorker::new_edge_slot() {
    bool ok1, ok2, ok3;
    int from = QInputDialog::getInt(this, tr("New edge"), "From  node id:", 0, 1, 50000, 1, &ok1);
    int to = QInputDialog::getInt(this, tr("New edge"), "To  node id:", 0, 1, 50000, 1, &ok2);
    QStringList items;
    items << tr("RT") << tr("interacting");
    QString edge_type = QInputDialog::getItem(this, tr("New edge"), tr("Edge type:"), items, 0, false, &ok3);

    if (not ok1 or not ok2 or not ok3 or edge_type.isEmpty())
        return;

    std::optional<Node> from_node = G->get_node(from);
    std::optional<Node> to_node = G->get_node(to);
    if (from_node.has_value() and to_node.has_value()) {
        if(edge_type == "RT")
        {
            try {
                std::vector<float> trans{0.f, 0.f, 0.f};
                std::vector<float> rot{0, 0.f, 0};
                G->insert_or_assign_edge_RT(from_node.value(), to, trans, rot);
            }
            catch (const std::exception &e) {
                std::cout << __FUNCTION__ << e.what() << std::endl;
            }
        }
        else if(edge_type == "interacting")
        {
            Edge edge;
            edge.type(edge_type.toStdString());
            //get two ids
            edge.from(from);
            edge.to(to);
            if(not G->insert_or_assign_edge(edge))
                std::cout<<"Error inserting new edge: "<<from<<"->"<<to<<" type: "<<edge_type.toStdString()<<std::endl;
        }
    }
    else{
        qDebug()<<"Selected node from or to does not exist";
    }
}
void SpecificWorker::new_edge_attrib_slot()
{
    bool ok1, ok2;
    QString attrib_name = QInputDialog::getText(this, tr("New edge attrib"),
                                                tr("Attrib name:"), QLineEdit::Normal,
                                                "name", &ok1);
    QStringList items;
    items << tr("int") << tr("float") << tr("string") << tr("bool") <<tr("vector");
    QString attrib_type = QInputDialog::getItem(this, tr("New edge attrib"), tr("Attrib type:"), items, 0, false, &ok2);

    if(not ok1 or not ok2 or attrib_name.isEmpty() or attrib_type.isEmpty())
        return;

    Edge edge = edge_cb->itemData(edge_cb->currentIndex()).value<Edge>();
    if(attrib_type == "int")
        G->insert_or_assign_attrib_by_name(edge, attrib_name.toStdString(),0);
    else if(attrib_type == "float")
        G->insert_or_assign_attrib_by_name(edge, attrib_name.toStdString(),0.0);
    else if(attrib_type == "bool")
        G->insert_or_assign_attrib_by_name(edge, attrib_name.toStdString(),false);
    else if(attrib_type == "string")
        G->insert_or_assign_attrib_by_name(edge, attrib_name.toStdString(),std::string(""));
    else if(attrib_type == "vector") {
        std::vector<float> zeros{0.f,0.f,0.f};
        G->insert_or_assign_attrib_by_name(edge, attrib_name.toStdString(), zeros);
    }
    fill_table(edge_attrib_tw, edge.attrs());
}
void SpecificWorker::del_node_attrib_slot() {
    Node node = node_cb->itemData(node_cb->currentIndex()).value<Node>();
    std::string attrib_name = node_attrib_tw->currentItem()->text().toStdString();
    if(G->remove_attrib_by_name(node, attrib_name))
    {
        fill_table(node_attrib_tw, node.attrs());
    }
    else
        qDebug()<<"Attribute"<<QString::fromStdString(attrib_name)<<"could not be deleted";
}

void SpecificWorker::del_edge_attrib_slot() {
    Edge edge = node_cb->itemData(edge_cb->currentIndex()).value<Edge>();
    std::string attrib_name = edge_attrib_tw->currentItem()->text().toStdString();
    if(G->remove_attrib_by_name(edge, attrib_name))
    {
        fill_table(edge_attrib_tw, edge.attrs());
    }
    else
        qDebug()<<"Attribute"<<QString::fromStdString(attrib_name)<<"could not be deleted";
}

void SpecificWorker::G_add_or_assign_node_slot(const std::int32_t id, const std::string &type) {
    Node node = G->get_node(id).value();
    QVariant data;
    data.setValue(node);
    int pos = this->node_cb->findText(QString::fromStdString(node.name()));
    if (pos == -1) { //insert item
        this->node_cb->addItem(QString::fromStdString(node.name()), data);
        node_combo_names[node.id()] = QString::fromStdString(node.name());
    }
    else //update item
        this->node_cb->setItemData(pos, data);
}
void SpecificWorker::G_add_or_assign_edge_slot(const std::int32_t from, const std::int32_t to, const std::string& type){
    Edge edge = G->get_edge(from, to, type).value();
    QVariant edge_data;
    edge_data.setValue(edge);
    QString from_s = QString::fromStdString(G->get_node(edge.from()).value().name());
    QString to_s = QString::fromStdString(G->get_node(edge.to()).value().name());
    QString name = from_s + "_" + to_s + "_" + QString::fromStdString(type);

    int pos = this->edge_cb->findText(name);
    if (pos == -1) { //insert item
        this->edge_cb->addItem(name, edge_data);
        edge_combo_names[std::to_string(edge.from())+"_"+std::to_string(edge.to())+"_"+edge.type()] = name;
    }
    else //update item
        this->edge_cb->setItemData(pos, edge_data);

}
void SpecificWorker::G_del_node_slot(const std::int32_t id){
    QString combo_name = node_combo_names[id];
    qDebug()<<"G_del_node"<<id<<combo_name;
    int pos = this->node_cb->findText(combo_name);
    if (pos != -1) {
        this->node_cb->removeItem(pos);
        node_combo_names.erase(id);
    }
}
void SpecificWorker::G_del_edge_slot(const std::int32_t from, const std::int32_t to, const std::string &type){
    QString combo_name = edge_combo_names[std::to_string(from)+"_"+std::to_string(to)+"_"+type];
    int pos = this->edge_cb->findText(combo_name);
    if (pos != -1)
    {
        this->edge_cb->removeItem(pos);
        edge_combo_names.erase(std::to_string(from)+"_"+std::to_string(to)+"_"+type);
    }
}
