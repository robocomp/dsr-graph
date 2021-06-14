//
// Created by Robolab on 18/4/21.
//

#include "QAttributeBrowser.h"
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QMainWindow>

Q_DECLARE_METATYPE(DSR::Node)
Q_DECLARE_METATYPE(DSR::Edge)
QAttributeBrowser::QAttributeBrowser(const std::shared_ptr<DSR::DSRGraph>& G, QWidget* parent)
{
    this->setupUi(this);
    this->G = G;
    qRegisterMetaType<std::uint32_t>("std::uint32_t");
    qRegisterMetaType<std::uint64_t>("std::uint64_t");
    qRegisterMetaType<std::string>("std::string");
    std::cout << "Initialize worker" << std::endl;

    node_attrib_tw->setColumnCount(2);
    edge_attrib_tw->setColumnCount(2);
    QStringList horzHeaders;
    horzHeaders << "Attribute" << "Value";
    node_attrib_tw->setHorizontalHeaderLabels(horzHeaders);
    node_attrib_tw->verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);
    node_attrib_tw->verticalHeader()->setDefaultSectionSize(40);
    node_attrib_tw->setSelectionBehavior(QAbstractItemView::SelectRows);

    edge_attrib_tw->setHorizontalHeaderLabels(horzHeaders);
    edge_attrib_tw->verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);
    edge_attrib_tw->verticalHeader()->setDefaultSectionSize(40);
    edge_attrib_tw->setSelectionBehavior(QAbstractItemView::SelectRows);
    new_node_attrib_pb->setEnabled(false);
    connect(node_cb, SIGNAL(currentIndexChanged(int)), this, SLOT(node_combobox_changed(int)));
    connect(edge_cb, SIGNAL(currentIndexChanged(int)), this, SLOT(edge_combobox_changed(int)));
    connect(save_node_pb, SIGNAL(clicked()), this, SLOT(save_node_clicked()));
    connect(save_edge_pb, SIGNAL(clicked()), this, SLOT(save_edge_clicked()));
//    connect(edge_cb, SIGNAL(currentIndexChanged(int)), this, SLOT(change_edge_slot(int)));
//    connect(del_node_pb, SIGNAL(clicked()), this, SLOT(delete_node_clicked()));
//    connect(del_edge_pb, SIGNAL(clicked()), this, SLOT(delete_edge_clicked()));
//    connect(new_node_pb, SIGNAL(clicked()), this, SLOT(new_node_slot()));
//    connect(new_edge_pb, SIGNAL(clicked()), this, SLOT(new_edge_slot()));
    connect(new_node_attrib_pb, SIGNAL(clicked()), this, SLOT(new_node_attrib_clicked()));
    connect(new_edge_attrib_pb, SIGNAL(clicked()), this, SLOT(new_edge_attrib_clicked()));
//    connect(del_node_attrib_pb, SIGNAL(clicked()), this, SLOT(del_node_attrib_slot()));
//    connect(del_edge_attrib_pb, SIGNAL(clicked()), this, SLOT(del_edge_attrib_slot()));
    connect(G.get(), &DSR::DSRGraph::update_node_signal,
            this, &QAttributeBrowser::add_node_to_combobox);
    connect(G.get(), &DSR::DSRGraph::del_node_signal,
            this, &QAttributeBrowser::G_node_deleted);
    connect(G.get(), &DSR::DSRGraph::update_edge_signal,
            this, &QAttributeBrowser::update_edge_slot);
    connect(G.get(), &DSR::DSRGraph::del_edge_signal,
            this, &QAttributeBrowser::G_edge_deleted);
//    connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &QAttributeBrowser::update_node_combobox);
//    connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &QAttributeBrowser::update_node_combobox);

}
void QAttributeBrowser::node_combobox_changed(DSR::Node node)
{
    node_id_label->setText("("+QString::number(node.id())+":"+QString::fromStdString(node.type())+")");
    fill_table(node_attrib_tw, node.attrs());
}

void QAttributeBrowser::node_combobox_changed(int node_combobox_index)
{
    auto node = qvariant_cast<DSR::Node>(this->node_cb->currentData());
    this->node_combobox_changed(node.id());
    this->update_edge_combobox(node_combobox_index);
}

void QAttributeBrowser::node_combobox_changed(uint64_t id)
{
    qDebug() << __FUNCTION__;
    auto node_opt = G->get_node(id);
    auto combo_str = this->node_combo_names[id];
    node_attrib_tw->setRowCount(0);
    if (node_opt.has_value()) {
        DSR::Node node = node_opt.value();
        this->node_combobox_changed(node);
    }
}

void QAttributeBrowser::edge_combobox_changed(int edge_combobox_index)
{
    auto edge = edge_cb->itemData(edge_combobox_index).value<DSR::Edge>();
    edge_id_label->setText("("+QString::number(edge.from())+"-"+QString::number(edge.to())+":"+QString::fromStdString(edge.type())+")");
    fill_table(edge_attrib_tw, edge.attrs());
}



//void QAttributeBrowser::change_edge_slot(int id)
//{
//    DSR::Edge edge = edge_cb->itemData(id).value<DSR::Edge>();
//    edge_id_label->setText("("+QString::number(edge.from())+"-"+QString::number(edge.to())+":"+QString::fromStdString(edge.type())+")");
//    fill_table(edge_attrib_tw, edge.attrs());
//}


void QAttributeBrowser::fill_table(QTableWidget* table_widget, const std::map<std::string, DSR::Attribute>& attribs)
{
    table_widget->setRowCount(0);
    for (auto[key, value] : attribs) {
        table_widget->insertRow(table_widget->rowCount());
        auto* item = new QTableWidgetItem(QString::fromStdString(key));
        item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
        table_widget->setItem(table_widget->rowCount()-1, 0, item);
        switch (value.selected()) {
        case 0: {
            auto* ledit = new QLineEdit(QString::fromStdString(value.str()));
            table_widget->setCellWidget(table_widget->rowCount()-1, 1, ledit);
        }
            break;
        case 1: {
            auto* spin = new QSpinBox();
            spin->setMinimum(-10000);
            spin->setMaximum(10000);
            spin->setValue(value.dec());
            table_widget->setCellWidget(table_widget->rowCount()-1, 1, spin);
        }
            break;
        case 6: {
            auto* spin = new QSpinBox();
            spin->setMinimum(-10000);
            spin->setMaximum(10000);
            spin->setValue(value.uint());
            table_widget->setCellWidget(table_widget->rowCount()-1, 1, spin);
        }
            break;
        case 2: {
            auto* spin = new QDoubleSpinBox();
            spin->setMinimum(-10000);
            spin->setMaximum(10000);
            spin->setValue(static_cast<double>(value.fl())*1000000/1000000);
            table_widget->setCellWidget(table_widget->rowCount()-1, 1, spin);
        }
            break;
        case 3: {
            auto* widget = new QWidget();
            auto* layout = new QHBoxLayout;
            widget->setLayout(layout);
            if (!value.float_vec().empty()) {
                for (float i : value.float_vec()) {
                    auto* spin = new QDoubleSpinBox();
                    spin->setMinimum(-10000);
                    spin->setMaximum(10000);
                    spin->setValue(i);
                    layout->addWidget(spin);
                }
                table_widget->setCellWidget(table_widget->rowCount()-1, 1, widget);
            }
        }
            break;
        case 4: {
            auto* combo = new QComboBox();
            combo->addItem("true");
            combo->addItem("false");
            if (value.bl())
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

void QAttributeBrowser::update_node_combobox(uint64_t node_id)
{
    if(node_id and this->node_combo_names.count(node_id)>0)
    {
        auto node_combo_name = this->node_combo_names[node_id];
        int index = this->node_cb->findText(node_combo_name);
        if ( index != -1 ) { // -1 for not found
            QVariant data;
            DSR::Node the_node = G->get_node(node_id).value();
            data.setValue(the_node);
            this->node_cb->setItemData(index, data);
            this->node_cb->setCurrentIndex(index);
            node_combobox_changed(index);
        }
    }

    if(this->node_cb->count()>0)
        this->new_node_attrib_pb->setEnabled(true);
    else
        this->new_node_attrib_pb->setEnabled(false);

}
void QAttributeBrowser::add_node_to_combobox(uint64_t node_id)
{
    if(node_id and this->node_combo_names.count(node_id) == 0) {
        auto node_opt = G->get_node(node_id);
        if (node_opt.has_value()) {
            auto node = node_opt.value();
            QVariant data;
            data.setValue(node);
            auto node_name = QString::fromStdString(node.name());
            this->node_combo_names[node_id] = node_name;
            node_cb->addItem(node_name, data);
            node_combobox_changed(0);
            update_edge_combobox(0);
        }
    }
}


void QAttributeBrowser::update_edge_slot(uint64_t from, uint64_t to,  const std::string &type)
{
    update_node_combobox(from);
}

void QAttributeBrowser::update_edge_combobox(int node_cb_index=-1)
{
    DSR::Node node;
    if(node_cb_index!=-1) {
         node = qvariant_cast<DSR::Node>(this->node_cb->itemData(node_cb_index));
    }
    else {
        node = qvariant_cast<DSR::Node>(this->node_cb->currentData());
    }
    this->update_edge_combobox(node);

}

void QAttributeBrowser::update_edge_combobox(DSR::Node node)
{
    this->edge_cb->clear();
    this->edge_attrib_tw->setRowCount(0);
    for (const auto &[key, edge] : node.fano()) {
        QVariant edge_data;
        edge_data.setValue(edge);
        QString from = QString::fromStdString(G->get_node(edge.from()).value().name());
        QString to = QString::fromStdString(G->get_node(edge.to()).value().name());
        QString name = from+"_"+to+"_"+QString::fromStdString(edge.type());
        this->edge_cb->addItem(name, edge_data);
        edge_combobox_changed(0);
    }
    if(edge_cb->count()>0)
        edges_groupbox->setChecked(true);
    else
        edges_groupbox->setChecked(false);
}

std::map<std::string, DSR::Attribute> QAttributeBrowser::get_table_content(QTableWidget *table_widget, std::map<std::string, DSR::Attribute> attrs)
{
    for (int row=0; row < table_widget->rowCount(); row++)
    {
        std::string key = table_widget->item(row, 0)->text().toStdString();
        switch(attrs.at(key).selected())
        {
        case Types::STRING:
        {
            auto *ledit = (QLineEdit*)table_widget->cellWidget(row, 1);
            attrs[key].str(ledit->text().toStdString());
        }
            break;
        case Types::INT:
        case Types::UINT:
        {
            auto *spin = (QSpinBox*)table_widget->cellWidget(row, 1);
            attrs[key].dec(spin->value());
        }
            break;
        case Types::FLOAT:
        {
            auto *spin = (QDoubleSpinBox*)table_widget->cellWidget(row, 1);
            attrs[key].fl(spin->value());
        }
            break;
        case Types::BOOL:
        {
            auto *combo = (QComboBox*)table_widget->cellWidget(row, 1);
            attrs[key].bl(combo->currentText().contains("true"));
        }
            break;
        case Types::FLOAT_VEC:
        {
            qDebug()<<__LINE__;
            std::vector<float> r;
            for (QDoubleSpinBox *spin: table_widget->cellWidget(row, 1)->findChildren<QDoubleSpinBox *>())
            {
                r.push_back(spin->value());
            }
            attrs[key].float_vec(r);
            std::cout<<r;
        }
            break;
        }
    }
    return attrs;
}


void QAttributeBrowser::save_node_clicked()
{
    auto node = node_cb->itemData(node_cb->currentIndex()).value<DSR::Node>();

    std::map<std::string, DSR::Attribute> new_attrs = get_table_content(node_attrib_tw, node.attrs());
    node.attrs(new_attrs);

    if(G->update_node(node))
        qDebug()<<"Node saved";
    else
    {
        qDebug()<<"Error saving node";
    }
}


void QAttributeBrowser::save_edge_clicked()
{
    auto edge = edge_cb->itemData(edge_cb->currentIndex()).value<DSR::Edge>();

    std::map<std::string, DSR::Attribute> new_attrs = get_table_content(edge_attrib_tw, edge.attrs());
    edge.attrs(new_attrs);

    if(G->insert_or_assign_edge(edge))
        qDebug()<<"Edge saved";
    else
    {
        qDebug()<<"Error saving edge";
    }

}




template<class Ta>
void QAttributeBrowser::new_attrib_clicked()
{
    bool ok;
    QStringList results = GraphNewAttributeDialog::getAttribute(this, ok);
    QString attrib_name = results[0];
    QString attrib_type = results[1];

    if(not ok or attrib_name.isEmpty() or attrib_type.isEmpty())
        return;

    Ta element;
    QComboBox* combo;
    QTableWidget* table;
    if constexpr(std::is_same_v<Ta, DSR::Node>)
    {
        combo = node_cb;
        table = node_attrib_tw;
    }
    else
    {
        combo = edge_cb;
        table = edge_attrib_tw;
    }
    element = combo->itemData(combo->currentIndex()).value<Ta>();
    if(attrib_type == "int")
        G->runtime_checked_add_or_modify_attrib_local(element, attrib_name.toStdString(),0);
    else if(attrib_type == "float")
        G->runtime_checked_add_or_modify_attrib_local(element, attrib_name.toStdString(),0.0f);
    else if(attrib_type == "bool")
        G->runtime_checked_add_or_modify_attrib_local(element, attrib_name.toStdString(),false);
    else if(attrib_type == "string")
        G->runtime_checked_add_or_modify_attrib_local(element, attrib_name.toStdString(),std::string(""));
    else if(attrib_type == "vector") {
        std::vector<float> zeros{0.f,0.f,0.f};
        G->runtime_checked_add_or_modify_attrib_local(element, attrib_name.toStdString(), zeros);
    }
    else {
        QMessageBox msgBox;
        msgBox.setText("Could not create specific attribute \""+attrib_name+"\"");
        msgBox.setInformativeText("Widget not found for type \""+attrib_type+"\"");
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.setDefaultButton(QMessageBox::Ok);
        msgBox.exec();
    }

    fill_table(table, element.attrs());
    // Updates information in the local edge
    QVariant data;
    data.setValue(element);
    combo->setItemData(combo->currentIndex(),data);
}


void QAttributeBrowser::new_edge_attrib_clicked()
{
    new_attrib_clicked<DSR::Edge>();
}

void QAttributeBrowser::new_node_attrib_clicked()
{
    new_attrib_clicked<DSR::Node>();
}

void QAttributeBrowser::G_node_deleted(uint64_t node_id)
{
    QString combo_name = node_combo_names[node_id];
    qDebug()<<"G_del_node"<<node_id<<combo_name;
    int pos = this->node_cb->findText(combo_name);
    if (pos != -1) {
        this->node_cb->removeItem(pos);
        node_combo_names.erase(node_id);
    }
}
void QAttributeBrowser::G_edge_deleted(const std::uint64_t from, const std::uint64_t to, const std::string &type)
{
    QString combo_name = edge_combo_names[std::to_string(from)+"_"+std::to_string(to)+"_"+type];
    int pos = this->edge_cb->findText(combo_name);
    if (pos != -1)
    {
        this->edge_cb->removeItem(pos);
        edge_combo_names.erase(std::to_string(from)+"_"+std::to_string(to)+"_"+type);
    }
}




