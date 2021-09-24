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
#include <QtWidgets/QFrame>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_local_guiDlg
{
public:
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    QFrame *frame;
    QWidget *widget;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *verticalLayout_5;
    QLabel *label;
    QPlainTextEdit *current_plan;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout_4;
    QVBoxLayout *verticalLayout_3;
    QPushButton *pushButton_start_mission;
    QPushButton *pushButton_stop_mission;
    QPushButton *pushButton_cancel_mission;
    QLabel *label_rgb;


    void setupUi(QWidget *local_guiDlg)
    {
        if (local_guiDlg->objectName().isEmpty())
            local_guiDlg->setObjectName(QString::fromUtf8("local_guiDlg"));
        local_guiDlg->resize(714, 773);
        verticalLayout_2 = new QVBoxLayout(local_guiDlg);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        frame = new QFrame(local_guiDlg);
        frame->setObjectName(QString::fromUtf8("frame"));
        frame->setMinimumSize(QSize(300, 0));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        widget = new QWidget(frame);
        widget->setObjectName(QString::fromUtf8("widget"));
        widget->setGeometry(QRect(30, 6, 401, 219));
        horizontalLayout = new QHBoxLayout(widget);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        label = new QLabel(widget);
        label->setObjectName(QString::fromUtf8("label"));

        verticalLayout_5->addWidget(label);

        current_plan = new QPlainTextEdit(widget);
        current_plan->setObjectName(QString::fromUtf8("current_plan"));

        verticalLayout_5->addWidget(current_plan);


        horizontalLayout->addLayout(verticalLayout_5);

        groupBox = new QGroupBox(widget);
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


        verticalLayout->addWidget(frame);

        label_rgb = new QLabel(local_guiDlg);
        label_rgb->setObjectName(QString::fromUtf8("label_rgb"));
        label_rgb->setMinimumSize(QSize(640, 0));

        verticalLayout->addWidget(label_rgb);


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
        label_rgb->setText(QApplication::translate("local_guiDlg", "TextLabel", nullptr));
    } // retranslateUi

};

namespace Ui {
    class local_guiDlg: public Ui_local_guiDlg {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LOCALUI_H
