//
// Created by robolab on 18/4/21.
//

#ifndef TESTCOMP_QATTRIBUTEBROWSER_H
#define TESTCOMP_QATTRIBUTEBROWSER_H

#pragma once

#include <dsr/core/types/user_types.h>
#include <QWidget>
#include <QMessageBox>
#include "ui_AttributeBrowser.h"
#include "GraphNewAttributeDialog.h"
#include <dsr/api/dsr_api.h>

class QAttributeBrowser : public QWidget, private Ui::AttributeBrowserWidget
{
    Q_OBJECT
public:
    explicit QAttributeBrowser(const std::shared_ptr<DSR::DSRGraph>& G, QWidget *parent = Q_NULLPTR);
    static void fill_table(QTableWidget *table_widget, const std::map<std::string, DSR::Attribute>& attribs);
public slots:
    //    void change_edge_slot(int id);
    void node_combobox_changed(uint64_t id);
    void update_node_combobox(uint64_t node_id=0);
    void update_edge_combobox(DSR::Node node);
    void update_edge_combobox(int node_cb_index);
    void node_combobox_changed(DSR::Node node);
    void node_combobox_changed(int node_combobox_index);
    void edge_combobox_changed(int edge_combobox_index);
    void G_node_deleted(uint64_t node_id);
    void G_edge_deleted(std::uint64_t from, std::uint64_t to, const std::string &type);
    void add_node_to_combobox(uint64_t node_id);



    void new_node_attrib_clicked();
    void new_edge_attrib_clicked();
    void save_node_clicked();
    void save_edge_clicked();
    void update_edge_slot(uint64_t from, uint64_t to, const std::string& type);



private:
    std::shared_ptr<DSR::DSRGraph> G;
    std::map<std::uint64_t, QString> node_combo_names;
    std::map<std::string, QString> edge_combo_names;
    static std::map<std::string, DSR::Attribute>
    get_table_content(QTableWidget* table_widget, std::map<std::string, DSR::Attribute> attrs);
    template<class Ta> void new_attrib_clicked();
};


#endif //TESTCOMP_QATTRIBUTEBROWSER_H
