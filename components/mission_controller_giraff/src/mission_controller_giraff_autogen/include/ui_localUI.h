/********************************************************************************
** Form generated from reading UI file 'localUI.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LOCALUI_H
#define UI_LOCALUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_local_guiDlg
{
public:
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    QFrame *frame;
    QWidget *layoutWidget;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *verticalLayout_5;
    QLabel *label;
    QComboBox *list_plan;
    QPlainTextEdit *current_plan;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout_4;
    QVBoxLayout *verticalLayout_3;
    QPushButton *pushButton_start_mission;
    QPushButton *pushButton_stop_mission;
    QPushButton *pushButton_cancel_mission;
    QLabel *labelX;
    QLabel *labelY;
    QPushButton *pushButton_save_coords;
    QLabel *label_rgb;
    QLabel *label_map;
    QSpinBox *coordX;
    QSpinBox *coordY;

    void setupUi(QWidget *local_guiDlg)
    {
        if (local_guiDlg->objectName().isEmpty())
            local_guiDlg->setObjectName(QString::fromUtf8("local_guiDlg"));
        local_guiDlg->resize(1642, 937);
        verticalLayout_2 = new QVBoxLayout(local_guiDlg);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        frame = new QFrame(local_guiDlg);
        frame->setObjectName(QString::fromUtf8("frame"));
        frame->setMinimumSize(QSize(300, 0));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        layoutWidget = new QWidget(frame);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(10, 10, 771, 351));
        horizontalLayout = new QHBoxLayout(layoutWidget);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        label = new QLabel(layoutWidget);
        label->setObjectName(QString::fromUtf8("label"));

        verticalLayout_5->addWidget(label);

        list_plan = new QComboBox(layoutWidget);
        list_plan->setObjectName(QString::fromUtf8("list_plan"));

        verticalLayout_5->addWidget(list_plan);

        current_plan = new QPlainTextEdit(layoutWidget);
        current_plan->setObjectName(QString::fromUtf8("current_plan"));
        current_plan->setEnabled(true);

        verticalLayout_5->addWidget(current_plan);


        horizontalLayout->addLayout(verticalLayout_5);

        groupBox = new QGroupBox(layoutWidget);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        verticalLayout_4 = new QVBoxLayout(groupBox);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        pushButton_start_mission = new QPushButton(groupBox);
        pushButton_start_mission->setObjectName(QString::fromUtf8("pushButton_start_mission"));

        verticalLayout_3->addWidget(pushButton_start_mission);

        pushButton_stop_mission = new QPushButton(groupBox);
        pushButton_stop_mission->setObjectName(QString::fromUtf8("pushButton_stop_mission"));

        verticalLayout_3->addWidget(pushButton_stop_mission);

        pushButton_cancel_mission = new QPushButton(groupBox);
        pushButton_cancel_mission->setObjectName(QString::fromUtf8("pushButton_cancel_mission"));

        verticalLayout_3->addWidget(pushButton_cancel_mission);


        verticalLayout_4->addLayout(verticalLayout_3);


        horizontalLayout->addWidget(groupBox);

        labelX = new QLabel(frame);
        labelX->setObjectName(QString::fromUtf8("labelX"));
        labelX->setGeometry(QRect(40, 400, 131, 17));
        labelY = new QLabel(frame);
        labelY->setObjectName(QString::fromUtf8("labelY"));
        labelY->setGeometry(QRect(260, 400, 101, 17));
        pushButton_save_coords = new QPushButton(frame);
        pushButton_save_coords->setObjectName(QString::fromUtf8("pushButton_save_coords"));
        pushButton_save_coords->setGeometry(QRect(480, 394, 161, 31));
        label_rgb = new QLabel(frame);
        label_rgb->setObjectName(QString::fromUtf8("label_rgb"));
        label_rgb->setGeometry(QRect(12, 496, 761, 401));
        label_map = new QLabel(frame);
        label_map->setObjectName(QString::fromUtf8("label_map"));
        label_map->setGeometry(QRect(800, 20, 761, 401));
        coordX = new QSpinBox(frame);
        coordX->setObjectName(QString::fromUtf8("coordX"));
        coordX->setGeometry(QRect(150, 390, 101, 41));
        coordY = new QSpinBox(frame);
        coordY->setObjectName(QString::fromUtf8("coordY"));
        coordY->setGeometry(QRect(370, 390, 101, 41));
        coordX->setRange(-12000, 12000);
        coordY->setRange(-34000, 34000);
        verticalLayout->addWidget(frame);


        verticalLayout_2->addLayout(verticalLayout);


        retranslateUi(local_guiDlg);

        QMetaObject::connectSlotsByName(local_guiDlg);
    } // setupUi

    void retranslateUi(QWidget *local_guiDlg)
    {
        local_guiDlg->setWindowTitle(QApplication::translate("local_guiDlg", "Controller", nullptr));
        label->setText(QApplication::translate("local_guiDlg", "Current mission", nullptr));
        groupBox->setTitle(QApplication::translate("local_guiDlg", "Plan Control", nullptr));
        pushButton_start_mission->setText(QApplication::translate("local_guiDlg", "Start mission", nullptr));
        pushButton_stop_mission->setText(QApplication::translate("local_guiDlg", "Stop mission", nullptr));
        pushButton_cancel_mission->setText(QApplication::translate("local_guiDlg", "Cancel mission", nullptr));
        labelX->setText(QApplication::translate("local_guiDlg", "Coordenada X:", nullptr));
        labelY->setText(QApplication::translate("local_guiDlg", "Coordenada Y:", nullptr));
        pushButton_save_coords->setText(QApplication::translate("local_guiDlg", "Guardar coordenadas", nullptr));
        label_rgb->setText(QApplication::translate("local_guiDlg", "TextLabel", nullptr));
        label_map->setText(QApplication::translate("local_guiDlg", "TextLabel", nullptr));
    } // retranslateUi

};

namespace Ui {
    class local_guiDlg: public Ui_local_guiDlg {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LOCALUI_H
