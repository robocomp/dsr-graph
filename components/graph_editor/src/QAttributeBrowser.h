//
// Created by robolab on 18/4/21.
//

#ifndef TESTCOMP_QATTRIBUTEBROWSER_H
#define TESTCOMP_QATTRIBUTEBROWSER_H

#pragma once

#include <dsr/core/types/user_types.h>
#include <QWidget>
#include "ui_AttributeBrowser.h"


class QAttributeBrowser : public QWidget, private Ui::AttributeBrowserWidget
{
    Q_OBJECT

public:
    QAttributeBrowser(QWidget *parent = Q_NULLPTR);
    void change_node_slot(int id);
    void change_edge_slot(int id);
    void fill_table(QTableWidget *table_widget, std::map<std::string, DSR::Attribute> attribs);
};


#endif //TESTCOMP_QATTRIBUTEBROWSER_H
