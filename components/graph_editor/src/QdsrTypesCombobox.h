//
// Created by robolab on 24/5/21.
//

#ifndef GRAPH_EDITOR_QDSRTYPESCOMBOBOX_H
#define GRAPH_EDITOR_QDSRTYPESCOMBOBOX_H

#include <QGroupBox>


template<class Ta>
class QDSRTypesCombobox: public QComboBox{
    const std::string TYPES_FILES_PATH = "/usr/local/include/dsr/core/types/type_checking";
public:
    void update_combo_box(std::string file_name, std::string type_reg_name, bool filter=true)
    {
        this->clear();
        std::string command_to_execute = "grep \""+type_reg_name+"\" "+TYPES_FILES_PATH+"/"+file_name+" | sed \"s/[()]/ /g\"  | cut -d',' -f1,2 | cut -d' ' -f2-";
        std::string output_string = exec_command(command_to_execute);
        qDebug()<<__FUNCTION__<<QString::fromStdString(command_to_execute);
        std::istringstream output(output_string);
        std::string  file_line;
        std::getline(output, file_line);
        while(std::getline(output, file_line))
        {
            bool filter_condition = (file_line.find("std::vector")!=std::string::npos
                    or file_line.find("std::map")!=std::string::npos);
            if (filter and filter_condition)
                continue;
            std::string delimiter = ",";
            size_t pos;
            std::string att_name;
            if constexpr(std::is_same_v<Ta, DSR::Attribute>) {
                while ((pos = file_line.find(delimiter))!=std::string::npos) {
                    att_name = file_line.substr(0, pos);
                    att_name.erase(att_name.find_last_not_of(" \n\r\t")+1);
                    file_line.erase(0, pos+delimiter.length());
                }
                if (file_line.find("std::vector")!=std::string::npos
                        or file_line.find("std::map")!=std::string::npos)
                    att_name = "["+att_name+"]";
            }
            else
            {
                att_name = file_line;
                att_name.erase(att_name.find_last_not_of(" \n\r\t")+1);
            }
            if(att_name.find_first_not_of(" \t\n\v\f\r") != std::string::npos) {
                this->addItem(QString::fromStdString(att_name));
            }
            if (file_line.find("std::reference_wrapper<const std::string>")!=std::string::npos)
                name_to_type[att_name] = "string";
            else
                name_to_type[att_name] = QString::fromStdString(file_line).simplified().toStdString();
        }
    }


    static std::string exec_command(const std::string& cmd) {
        char buffer[128];
        std::string result;
        FILE* pipe = popen(cmd.c_str(), "r");
        if (!pipe) throw std::runtime_error("popen() failed!");
        try {
            while (fgets(buffer, sizeof buffer, pipe) != nullptr) {
                result += buffer;
            }
        } catch (...) {
            pclose(pipe);
            throw;
        }
        pclose(pipe);
        return result;
    }

    explicit QDSRTypesCombobox(QGroupBox* pBox, const std::string& ="attributes"):QComboBox(pBox)
    {
        if constexpr(std::is_same_v<Ta, DSR::Node>)
        {
            update_combo_box("dsr_node_type.h", "REGISTER_NODE_TYPE");
        }
        else if constexpr(std::is_same_v<Ta, DSR::Edge>)
        {
            update_combo_box("dsr_edge_type.h", "REGISTER_EDGE_TYPE");
        }
        else if constexpr(std::is_same_v<Ta, DSR::Attribute>)
        {
            update_combo_box("dsr_attr_name.h", "REGISTER_TYPE");
        }
    };
private:
    std::map<std::string, std::string> name_to_type;
//    template<typename Ta> std::map<Ta, std::pair<std::string, std::string>> files =
//            {
//             {DSR::Node, {"dsr_node_type.h", "REGISTER_NODE_TYPE"}},
//             {DSR::Edge, {"dsr_edge_type.h", "REGISTER_EDGE_TYPE"}},
//             {DSR::Attribute, {"dsr_edge_type.h", "REGISTER_TYPE"}}
//            };

};


#endif //GRAPH_EDITOR_QDSRTYPESCOMBOBOX_H
