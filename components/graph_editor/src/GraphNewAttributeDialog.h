//
// Created by robolab on 10/4/21.
//

#ifndef TESTCOMP_GRAPHNEWELEMENTDIALOG_H
#define TESTCOMP_GRAPHNEWELEMENTDIALOG_H
#include <iostream>
#include <stdexcept>
#include <sstream>
#include <string>

#include "ui_GraphNewAttributeDialog.h"
#include "QdsrTypesCombobox.h"


class GraphNewAttributeDialog : public QDialog, private Ui::NewAttributeDialog
{
Q_OBJECT
private:
    std::map<std::string, std::string> name_to_type;
    QDSRTypesCombobox<DSR::Attribute> *att_names_cmb;
    public:
        GraphNewAttributeDialog(QWidget *parent):QDialog(parent)
        {

            this->setupUi(this);
            att_names_cmb = new QDSRTypesCombobox<DSR::Attribute>(node_group);
            att_names_cmb->setObjectName(QString::fromUtf8("att_names_cmb"));
            formLayout->setWidget(0, QFormLayout::FieldRole, att_names_cmb);
            connect(att_names_cmb, SIGNAL(currentIndexChanged(int)), this, SLOT(attr_combobox_changed(int)));
            connect(filter_types_checkbox, SIGNAL(stateChanged(int)), this, SLOT(set_filter(int)));
            attr_combobox_changed(0);
           //            this->node_type_cmb->addItems(
//                    QStringList()<<
//                    tr("plane") <<
//                    tr("transform") <<
//                    tr("mesh") <<
//                    tr("person")<<
//                    tr("omnirobot")<<
//                    tr("rgbd"));
//            this->node_type_cmb_2->addItems(
//                    QStringList()<<
//                                 tr("plane") <<
//                                 tr("transform") <<
//                                 tr("mesh") <<
//                                 tr("person")<<
//                                 tr("omnirobot")<<
//                                 tr("rgbd"));
//            this->edge_type_cmb->addItems(
//                    QStringList()<<
//                    tr("RT") <<
//                    tr("interacting"));
        }

        static QStringList getAttribute(QWidget * parent, bool &ok)
        {
            GraphNewAttributeDialog dlg(parent);
            dlg.adjustSize();
            ok = dlg.exec();
            QStringList results;
            results<<dlg.att_names_cmb->currentText()<<dlg.att_type->text();
            return results;
        }

    void update_combo_box(bool filter=true)
    {
        att_names_cmb->clear();
        std::string output_string = exec_command("grep \"REGISTER_TYPE\" /usr/local/include/dsr/core/types/type_checking/dsr_attr_name.h | sed \"s/(/ /g\"  | cut -d',' -f1,2 | cut -d' ' -f2-");
        std::istringstream output(output_string);
        std::string    file_line;
        std::getline(output, file_line);
        while(std::getline(output, file_line))
        {
            bool filter_condition = (file_line.find("std::vector")!=std::string::npos
                    or file_line.find("std::map")!=std::string::npos);
            if (filter and filter_condition)
                continue;
            std::string delimiter = ",";
            size_t pos = 0;
            std::string att_name;
            while ((pos = file_line.find(delimiter))!=std::string::npos) {
                att_name = file_line.substr(0, pos);
                att_name.erase(att_name.find_last_not_of(" \n\r\t")+1);
                file_line.erase(0, pos+delimiter.length());
            }
            if(file_line.find("std::vector")!=std::string::npos
                    or file_line.find("std::map")!=std::string::npos)
                att_name="["+att_name+"]";
            att_names_cmb->addItem(QString::fromStdString(att_name));
            if (file_line.find("std::reference_wrapper<const std::string>")!=std::string::npos)
                    name_to_type[att_name] = "string";
                else
                    name_to_type[att_name] = QString::fromStdString(file_line).simplified().toStdString();
        }
    }



        static std::string exec_command(const char* cmd) {
        char buffer[128];
        std::string result = "";
        FILE* pipe = popen(cmd, "r");
        if (!pipe) throw std::runtime_error("popen() failed!");
        try {
            while (fgets(buffer, sizeof buffer, pipe) != NULL) {
                result += buffer;
            }
        } catch (...) {
            pclose(pipe);
            throw;
        }
        pclose(pipe);
        return result;
    }
private slots:
    void attr_combobox_changed(int index)
    {
        QString combo_text = att_names_cmb->currentText();
        att_type->setText(QString::fromStdString(name_to_type[combo_text.toStdString()]));
    }

    void set_filter(int value)
    {

        update_combo_box(value!=0);
    }

};

#endif //TESTCOMP_GRAPHNEWELEMENTDIALOG_H
