//
// Created by robolab on 10/4/21.
//

#ifndef TESTCOMP_GRAPHNEWELEMENTDIALOG_H
#define TESTCOMP_GRAPHNEWELEMENTDIALOG_H

#include "ui_NewGraphElementDlg.h"



class NewGraphElementDlg2 : public QDialog, private Ui::NewElementDlg
{
    public:
        NewGraphElementDlg2(QWidget *parent): QDialog(parent)
        {
            this->setupUi(this);
            this->node_type_cmb->addItems(
                    QStringList()<<
                    tr("plane") <<
                    tr("transform") <<
                    tr("mesh") <<
                    tr("person")<<
                    tr("omnirobot")<<
                    tr("rgbd"));
            this->node_type_cmb_2->addItems(
                    QStringList()<<
                                 tr("plane") <<
                                 tr("transform") <<
                                 tr("mesh") <<
                                 tr("person")<<
                                 tr("omnirobot")<<
                                 tr("rgbd"));
            this->edge_type_cmb->addItems(
                    QStringList()<<
                    tr("RT") <<
                    tr("interacting"));
        }

        static QStringList getElementsParams(QWidget * parent, bool node1, bool node2, bool edge, bool &ok)
        {
            NewGraphElementDlg2 dlg(parent);
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



};

#endif //TESTCOMP_GRAPHNEWELEMENTDIALOG_H
