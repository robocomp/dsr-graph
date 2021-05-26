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

    explicit QDSRTypesCombobox(QGroupBox* pBox, const std::string& ="attributes"):QComboBox(pBox)
    {
        std::unordered_set<std::string_view>  types_set;
        if constexpr(std::is_same_v<Ta, DSR::Node>)
        {
            types_set = edge_types::get_all();
        }
        else if constexpr(std::is_same_v<Ta, DSR::Edge>)
        {
            types_set = node_types::get_all();
        }
        else if constexpr(std::is_same_v<Ta, DSR::Attribute>)
        {
            types_set = attribute_types::get_all();
        }
        this->clear();
        auto types = QStringList();
        for(auto const& type_t: types_set) {
            types.append(QString::fromStdString(type_t.data()));
        }
        this->addItems(types);
        qDebug()<<types;
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
