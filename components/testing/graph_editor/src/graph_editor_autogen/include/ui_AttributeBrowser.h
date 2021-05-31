/********************************************************************************
** Form generated from reading UI file 'AttributeBrowser.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ATTRIBUTEBROWSER_H
#define UI_ATTRIBUTEBROWSER_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_AttributeBrowserWidget
{
public:
    QHBoxLayout *horizontalLayout_5;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QLabel *node_id_label;
    QComboBox *node_cb;
    QLabel *label_2;
    QTableWidget *node_attrib_tw;
    QHBoxLayout *horizontalLayout_6;
    QSpacerItem *horizontalSpacer_3;
    QPushButton *new_node_attrib_pb;
    QPushButton *del_node_attrib_pb;
    QPushButton *save_node_pb;
    QGroupBox *edges_groupbox;
    QVBoxLayout *verticalLayout_3;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_3;
    QLabel *edge_id_label;
    QComboBox *edge_cb;
    QLabel *label_4;
    QTableWidget *edge_attrib_tw;
    QHBoxLayout *horizontalLayout_7;
    QSpacerItem *horizontalSpacer_4;
    QPushButton *new_edge_attrib_pb;
    QPushButton *del_edge_attrib_pb;
    QPushButton *save_edge_pb;

    void setupUi(QWidget *AttributeBrowserWidget)
    {
        if (AttributeBrowserWidget->objectName().isEmpty())
            AttributeBrowserWidget->setObjectName(QString::fromUtf8("AttributeBrowserWidget"));
        AttributeBrowserWidget->resize(555, 441);
        horizontalLayout_5 = new QHBoxLayout(AttributeBrowserWidget);
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label = new QLabel(AttributeBrowserWidget);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout->addWidget(label);

        node_id_label = new QLabel(AttributeBrowserWidget);
        node_id_label->setObjectName(QString::fromUtf8("node_id_label"));

        horizontalLayout->addWidget(node_id_label);

        node_cb = new QComboBox(AttributeBrowserWidget);
        node_cb->setObjectName(QString::fromUtf8("node_cb"));

        horizontalLayout->addWidget(node_cb);


        verticalLayout->addLayout(horizontalLayout);

        label_2 = new QLabel(AttributeBrowserWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        verticalLayout->addWidget(label_2);

        node_attrib_tw = new QTableWidget(AttributeBrowserWidget);
        node_attrib_tw->setObjectName(QString::fromUtf8("node_attrib_tw"));

        verticalLayout->addWidget(node_attrib_tw);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_6->addItem(horizontalSpacer_3);

        new_node_attrib_pb = new QPushButton(AttributeBrowserWidget);
        new_node_attrib_pb->setObjectName(QString::fromUtf8("new_node_attrib_pb"));

        horizontalLayout_6->addWidget(new_node_attrib_pb);

        del_node_attrib_pb = new QPushButton(AttributeBrowserWidget);
        del_node_attrib_pb->setObjectName(QString::fromUtf8("del_node_attrib_pb"));

        horizontalLayout_6->addWidget(del_node_attrib_pb);

        save_node_pb = new QPushButton(AttributeBrowserWidget);
        save_node_pb->setObjectName(QString::fromUtf8("save_node_pb"));

        horizontalLayout_6->addWidget(save_node_pb);


        verticalLayout->addLayout(horizontalLayout_6);

        edges_groupbox = new QGroupBox(AttributeBrowserWidget);
        edges_groupbox->setObjectName(QString::fromUtf8("edges_groupbox"));
        edges_groupbox->setCheckable(true);
        edges_groupbox->setChecked(false);
        verticalLayout_3 = new QVBoxLayout(edges_groupbox);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label_3 = new QLabel(edges_groupbox);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        horizontalLayout_2->addWidget(label_3);

        edge_id_label = new QLabel(edges_groupbox);
        edge_id_label->setObjectName(QString::fromUtf8("edge_id_label"));

        horizontalLayout_2->addWidget(edge_id_label);

        edge_cb = new QComboBox(edges_groupbox);
        edge_cb->setObjectName(QString::fromUtf8("edge_cb"));

        horizontalLayout_2->addWidget(edge_cb);


        verticalLayout_2->addLayout(horizontalLayout_2);

        label_4 = new QLabel(edges_groupbox);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        verticalLayout_2->addWidget(label_4);

        edge_attrib_tw = new QTableWidget(edges_groupbox);
        edge_attrib_tw->setObjectName(QString::fromUtf8("edge_attrib_tw"));

        verticalLayout_2->addWidget(edge_attrib_tw);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_7->addItem(horizontalSpacer_4);

        new_edge_attrib_pb = new QPushButton(edges_groupbox);
        new_edge_attrib_pb->setObjectName(QString::fromUtf8("new_edge_attrib_pb"));

        horizontalLayout_7->addWidget(new_edge_attrib_pb);

        del_edge_attrib_pb = new QPushButton(edges_groupbox);
        del_edge_attrib_pb->setObjectName(QString::fromUtf8("del_edge_attrib_pb"));

        horizontalLayout_7->addWidget(del_edge_attrib_pb);

        save_edge_pb = new QPushButton(edges_groupbox);
        save_edge_pb->setObjectName(QString::fromUtf8("save_edge_pb"));

        horizontalLayout_7->addWidget(save_edge_pb);


        verticalLayout_2->addLayout(horizontalLayout_7);


        verticalLayout_3->addLayout(verticalLayout_2);


        verticalLayout->addWidget(edges_groupbox);


        horizontalLayout_5->addLayout(verticalLayout);


        retranslateUi(AttributeBrowserWidget);

        QMetaObject::connectSlotsByName(AttributeBrowserWidget);
    } // setupUi

    void retranslateUi(QWidget *AttributeBrowserWidget)
    {
        AttributeBrowserWidget->setWindowTitle(QApplication::translate("AttributeBrowserWidget", "Form", nullptr));
        label->setText(QApplication::translate("AttributeBrowserWidget", "Node", nullptr));
        node_id_label->setText(QApplication::translate("AttributeBrowserWidget", "(ID, type)", nullptr));
        label_2->setText(QApplication::translate("AttributeBrowserWidget", "Node attributes", nullptr));
        new_node_attrib_pb->setText(QApplication::translate("AttributeBrowserWidget", "New attrib", nullptr));
        del_node_attrib_pb->setText(QApplication::translate("AttributeBrowserWidget", "Del Atrrib", nullptr));
        save_node_pb->setText(QApplication::translate("AttributeBrowserWidget", "Save Node", nullptr));
        edges_groupbox->setTitle(QApplication::translate("AttributeBrowserWidget", "Edges", nullptr));
        label_3->setText(QApplication::translate("AttributeBrowserWidget", "Edge", nullptr));
        edge_id_label->setText(QApplication::translate("AttributeBrowserWidget", "(ID, type)", nullptr));
        label_4->setText(QApplication::translate("AttributeBrowserWidget", "Edge attibutes", nullptr));
        new_edge_attrib_pb->setText(QApplication::translate("AttributeBrowserWidget", "New attrib", nullptr));
        del_edge_attrib_pb->setText(QApplication::translate("AttributeBrowserWidget", "Del attrib", nullptr));
        save_edge_pb->setText(QApplication::translate("AttributeBrowserWidget", "Save edge", nullptr));
    } // retranslateUi

};

namespace Ui {
    class AttributeBrowserWidget: public Ui_AttributeBrowserWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ATTRIBUTEBROWSER_H
