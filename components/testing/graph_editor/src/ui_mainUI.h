/********************************************************************************
** Form generated from reading UI file 'mainUI.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINUI_H
#define UI_MAINUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_guiDlg
{
public:
    QAction *actionSave;
    QAction *actionSimulate;
    QAction *actionSaveToFile;
    QWidget *centralwidget;
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
    QHBoxLayout *horizontalLayout_3;
    QSpacerItem *horizontalSpacer;
    QPushButton *new_node_pb;
    QPushButton *del_node_pb;
    QPushButton *save_node_pb;
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
    QHBoxLayout *horizontalLayout_4;
    QSpacerItem *horizontalSpacer_2;
    QPushButton *new_edge_pb;
    QPushButton *del_edge_pb;
    QPushButton *save_edge_pb;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *guiDlg)
    {
        if (guiDlg->objectName().isEmpty())
            guiDlg->setObjectName(QString::fromUtf8("guiDlg"));
        guiDlg->resize(800, 600);
        actionSave = new QAction(guiDlg);
        actionSave->setObjectName(QString::fromUtf8("actionSave"));
        actionSimulate = new QAction(guiDlg);
        actionSimulate->setObjectName(QString::fromUtf8("actionSimulate"));
        actionSimulate->setCheckable(true);
        actionSaveToFile = new QAction(guiDlg);
        actionSaveToFile->setObjectName(QString::fromUtf8("actionSaveToFile"));
        centralwidget = new QWidget(guiDlg);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        horizontalLayout_5 = new QHBoxLayout(centralwidget);
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label = new QLabel(centralwidget);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout->addWidget(label);

        node_id_label = new QLabel(centralwidget);
        node_id_label->setObjectName(QString::fromUtf8("node_id_label"));

        horizontalLayout->addWidget(node_id_label);

        node_cb = new QComboBox(centralwidget);
        node_cb->setObjectName(QString::fromUtf8("node_cb"));

        horizontalLayout->addWidget(node_cb);


        verticalLayout->addLayout(horizontalLayout);

        label_2 = new QLabel(centralwidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        verticalLayout->addWidget(label_2);

        node_attrib_tw = new QTableWidget(centralwidget);
        node_attrib_tw->setObjectName(QString::fromUtf8("node_attrib_tw"));

        verticalLayout->addWidget(node_attrib_tw);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_6->addItem(horizontalSpacer_3);

        new_node_attrib_pb = new QPushButton(centralwidget);
        new_node_attrib_pb->setObjectName(QString::fromUtf8("new_node_attrib_pb"));

        horizontalLayout_6->addWidget(new_node_attrib_pb);

        del_node_attrib_pb = new QPushButton(centralwidget);
        del_node_attrib_pb->setObjectName(QString::fromUtf8("del_node_attrib_pb"));

        horizontalLayout_6->addWidget(del_node_attrib_pb);


        verticalLayout->addLayout(horizontalLayout_6);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer);

        new_node_pb = new QPushButton(centralwidget);
        new_node_pb->setObjectName(QString::fromUtf8("new_node_pb"));

        horizontalLayout_3->addWidget(new_node_pb);

        del_node_pb = new QPushButton(centralwidget);
        del_node_pb->setObjectName(QString::fromUtf8("del_node_pb"));

        horizontalLayout_3->addWidget(del_node_pb);

        save_node_pb = new QPushButton(centralwidget);
        save_node_pb->setObjectName(QString::fromUtf8("save_node_pb"));

        horizontalLayout_3->addWidget(save_node_pb);


        verticalLayout->addLayout(horizontalLayout_3);


        horizontalLayout_5->addLayout(verticalLayout);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label_3 = new QLabel(centralwidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        horizontalLayout_2->addWidget(label_3);

        edge_id_label = new QLabel(centralwidget);
        edge_id_label->setObjectName(QString::fromUtf8("edge_id_label"));

        horizontalLayout_2->addWidget(edge_id_label);

        edge_cb = new QComboBox(centralwidget);
        edge_cb->setObjectName(QString::fromUtf8("edge_cb"));

        horizontalLayout_2->addWidget(edge_cb);


        verticalLayout_2->addLayout(horizontalLayout_2);

        label_4 = new QLabel(centralwidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        verticalLayout_2->addWidget(label_4);

        edge_attrib_tw = new QTableWidget(centralwidget);
        edge_attrib_tw->setObjectName(QString::fromUtf8("edge_attrib_tw"));

        verticalLayout_2->addWidget(edge_attrib_tw);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_7->addItem(horizontalSpacer_4);

        new_edge_attrib_pb = new QPushButton(centralwidget);
        new_edge_attrib_pb->setObjectName(QString::fromUtf8("new_edge_attrib_pb"));

        horizontalLayout_7->addWidget(new_edge_attrib_pb);

        del_edge_attrib_pb = new QPushButton(centralwidget);
        del_edge_attrib_pb->setObjectName(QString::fromUtf8("del_edge_attrib_pb"));

        horizontalLayout_7->addWidget(del_edge_attrib_pb);


        verticalLayout_2->addLayout(horizontalLayout_7);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer_2);

        new_edge_pb = new QPushButton(centralwidget);
        new_edge_pb->setObjectName(QString::fromUtf8("new_edge_pb"));

        horizontalLayout_4->addWidget(new_edge_pb);

        del_edge_pb = new QPushButton(centralwidget);
        del_edge_pb->setObjectName(QString::fromUtf8("del_edge_pb"));

        horizontalLayout_4->addWidget(del_edge_pb);

        save_edge_pb = new QPushButton(centralwidget);
        save_edge_pb->setObjectName(QString::fromUtf8("save_edge_pb"));

        horizontalLayout_4->addWidget(save_edge_pb);


        verticalLayout_2->addLayout(horizontalLayout_4);


        horizontalLayout_5->addLayout(verticalLayout_2);

        guiDlg->setCentralWidget(centralwidget);
        menubar = new QMenuBar(guiDlg);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 800, 22));
        guiDlg->setMenuBar(menubar);
        statusbar = new QStatusBar(guiDlg);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        guiDlg->setStatusBar(statusbar);

        retranslateUi(guiDlg);

        QMetaObject::connectSlotsByName(guiDlg);
    } // setupUi

    void retranslateUi(QMainWindow *guiDlg)
    {
        guiDlg->setWindowTitle(QApplication::translate("guiDlg", "DSREditor", nullptr));
        actionSave->setText(QApplication::translate("guiDlg", "Save", nullptr));
#ifndef QT_NO_TOOLTIP
        actionSave->setToolTip(QApplication::translate("guiDlg", "<html><head/><body><p>Save file in JSON format</p></body></html>", nullptr));
#endif // QT_NO_TOOLTIP
        actionSimulate->setText(QApplication::translate("guiDlg", "Simulate", nullptr));
        actionSaveToFile->setText(QApplication::translate("guiDlg", "Save", nullptr));
        label->setText(QApplication::translate("guiDlg", "Node", nullptr));
        node_id_label->setText(QApplication::translate("guiDlg", "(ID, type)", nullptr));
        label_2->setText(QApplication::translate("guiDlg", "Node content", nullptr));
        new_node_attrib_pb->setText(QApplication::translate("guiDlg", "New attrib", nullptr));
        del_node_attrib_pb->setText(QApplication::translate("guiDlg", "Del Atrrib", nullptr));
        new_node_pb->setText(QApplication::translate("guiDlg", "New Node", nullptr));
        del_node_pb->setText(QApplication::translate("guiDlg", "Del Node", nullptr));
        save_node_pb->setText(QApplication::translate("guiDlg", "Save Node", nullptr));
        label_3->setText(QApplication::translate("guiDlg", "Edge", nullptr));
        edge_id_label->setText(QApplication::translate("guiDlg", "(ID, type)", nullptr));
        label_4->setText(QApplication::translate("guiDlg", "Edge content", nullptr));
        new_edge_attrib_pb->setText(QApplication::translate("guiDlg", "New attrib", nullptr));
        del_edge_attrib_pb->setText(QApplication::translate("guiDlg", "Del attrib", nullptr));
        new_edge_pb->setText(QApplication::translate("guiDlg", "New Edge", nullptr));
        del_edge_pb->setText(QApplication::translate("guiDlg", "Del edge", nullptr));
        save_edge_pb->setText(QApplication::translate("guiDlg", "Save edge", nullptr));
    } // retranslateUi

};

namespace Ui {
    class guiDlg: public Ui_guiDlg {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINUI_H
