//
// Created by robolab on 18/4/21.
//

#include "QAttributeBrowser.h"
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QSpinBox>


QAttributeBrowser::QAttributeBrowser(QWidget *parent)
{

}

void QAttributeBrowser::change_node_slot(int id)
{
    DSR::Node node = node_cb->itemData(id).value<DSR::Node>();
    node_id_label->setText("("+QString::number(node.id())+":"+QString::fromStdString(node.type())+")");
    fill_table(node_attrib_tw, node.attrs());
}

void QAttributeBrowser::change_edge_slot(int id)
{
    DSR::Edge edge = edge_cb->itemData(id).value<DSR::Edge>();
    edge_id_label->setText("("+QString::number(edge.from())+"-"+QString::number(edge.to())+":"+QString::fromStdString(edge.type())+")");
    fill_table(edge_attrib_tw, edge.attrs());
}


void QAttributeBrowser::fill_table(QTableWidget *table_widget, std::map<std::string, DSR::Attribute> attribs)
{
    table_widget->setRowCount(0);
    for(auto [key, value] : attribs)
    {
        table_widget->insertRow( table_widget->rowCount() );
        QTableWidgetItem *item =new QTableWidgetItem(QString::fromStdString(key));
        item->setFlags(Qt::ItemIsSelectable|Qt::ItemIsEnabled);
        table_widget->setItem(table_widget->rowCount()-1, 0, item);
        switch (value.selected())
        {
        case 0:
        {
            QLineEdit *ledit = new QLineEdit(QString::fromStdString(value.str()));
            table_widget->setCellWidget(table_widget->rowCount()-1, 1, ledit);
        }
            break;
        case 1:
        {
            QSpinBox *spin = new QSpinBox();
            spin->setMinimum(-10000);
            spin->setMaximum(10000);
            spin->setValue(value.dec());
            table_widget->setCellWidget(table_widget->rowCount()-1, 1, spin);
        }
            break;
        case 6:
        {
            QSpinBox *spin = new QSpinBox();
            spin->setMinimum(-10000);
            spin->setMaximum(10000);
            spin->setValue(value.uint());
            table_widget->setCellWidget(table_widget->rowCount()-1, 1, spin);
        }
            break;
        case 2:
        {
            QDoubleSpinBox *spin = new QDoubleSpinBox();
            spin->setMinimum(-10000);
            spin->setMaximum(10000);
            spin->setValue(std::round(static_cast<double>(value.fl()) *1000000)/ 1000000);
            table_widget->setCellWidget(table_widget->rowCount()-1, 1, spin);
        }
            break;
        case 3:
        {
            QWidget  *widget = new QWidget();
            QHBoxLayout *layout = new QHBoxLayout;
            widget->setLayout(layout);
            if (!value.float_vec().empty())
            {
                for(std::size_t i = 0; i < value.float_vec().size(); ++i)
                {
                    QDoubleSpinBox *spin = new QDoubleSpinBox();
                    spin->setMinimum(-10000);
                    spin->setMaximum(10000);
                    spin->setValue(value.float_vec()[i]);
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