//
// Created by robolab on 10/4/21.
//

#ifndef TESTCOMP_GRAPHNEWELEMENTDIALOG_H
#define TESTCOMP_GRAPHNEWELEMENTDIALOG_H

#include "QdsrTypesCombobox.h"
#include "ui_GraphNewElementDialog.h"



class GraphNewElementDialog : public QDialog, private Ui::NewElementDlg
{
    public:
        explicit GraphNewElementDialog(QWidget *parent):QDialog(parent)
        {
            this->setupUi(this);
            node_type_cmb = new QDSRTypesCombobox<DSR::Node>(node_group_2);
            node_type_cmb->setObjectName(QString::fromUtf8("node_type_cmb"));
            formLayout->setWidget(1, QFormLayout::FieldRole, node_type_cmb);
            node_type_cmb_2 = new QDSRTypesCombobox<DSR::Node>(node_group_2);
            node_type_cmb_2->setObjectName(QString::fromUtf8("node_type_cmb_2"));
            formLayout_4->setWidget(1, QFormLayout::FieldRole, node_type_cmb_2);
            edge_type_cmb = new QDSRTypesCombobox<DSR::Edge>(edge_group);
            edge_type_cmb->setObjectName(QString::fromUtf8("edge_type_cmb"));
            formLayout_2->setWidget(0, QFormLayout::FieldRole, edge_type_cmb);
        }

        static QStringList getElementsParams(QWidget * parent, bool node1, bool node2, bool edge, bool &ok)
        {
            GraphNewElementDialog dlg(parent);
            if (!node1)
                dlg.node_group->setVisible(false);
            if (!node2)
                dlg.node_group_2->setVisible(false);
            if (!edge)
                dlg.edge_group->setVisible(false);

            dlg.adjustSize();

            ok = dlg.exec();
            QStringList results;
            if (node1) {
                results.append(dlg.node_name_lbl->text());
                results.append(dlg.node_type_cmb->currentText());
            }
            if (node2) {
                results.append(dlg.node_name_lbl_2->text());
                results.append(dlg.node_type_cmb_2->currentText());
            }
            if (edge)
            {
                results.append(dlg.edge_type_cmb->currentText());
            }
            return results;
        }

    static QStringList getNodeParams(QWidget * parent,  bool &ok)
    {
        return getElementsParams(parent, true, false, false, ok);
    }

    static QStringList getTwoNodeParams(QWidget * parent,  bool &ok)
    {
        return getElementsParams(parent, true, true, false, ok);
    }

    static QStringList getConnectedNodeParams(QWidget * parent, bool &ok)
    {
        return getElementsParams(parent, true, false, true, ok);
    }

    static QStringList getEdgeParams(QWidget * parent, bool &ok)
    {
        return getElementsParams(parent, false, false, true, ok);
    }

    static QStringList getAllParams(QWidget * parent, bool &ok)
    {
        return getElementsParams(parent, true, true, true, ok);
    }

//private:
    QDSRTypesCombobox<DSR::Edge>* edge_type_cmb;
    QDSRTypesCombobox<DSR::Node>* node_type_cmb;
    QDSRTypesCombobox<DSR::Node>* node_type_cmb_2;

};

#endif //TESTCOMP_GRAPHNEWELEMENTDIALOG_H
