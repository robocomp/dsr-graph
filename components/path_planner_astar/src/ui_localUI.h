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
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_local_guiDlg
{
public:
    QHBoxLayout *horizontalLayout_2;
    QGroupBox *groupBox;
    QHBoxLayout *horizontalLayout;
    QFormLayout *formLayout;
    QLabel *label;
    QLabel *label_2;
    QSpinBox *x_spinbox;
    QSpinBox *z_spinbox;
    QPushButton *send_button;
    QPushButton *stop_button;
    QSpacerItem *horizontalSpacer;
    QGroupBox *groupBox_2;
    QHBoxLayout *horizontalLayout_6;
    QVBoxLayout *verticalLayout_13;
    QCheckBox *robotMov_checkbox;
    QCheckBox *autoMov_checkbox;
    QSpacerItem *horizontalSpacer_2;
    QVBoxLayout *verticalLayout_4;
    QHBoxLayout *horizontalLayout_9;
    QLabel *label_5;
    QSlider *ki_slider;
    QHBoxLayout *horizontalLayout_8;
    QLabel *label_4;
    QSlider *ke_slider;
    QFrame *line;

    void setupUi(QWidget *local_guiDlg)
    {
        if (local_guiDlg->objectName().isEmpty())
            local_guiDlg->setObjectName(QString::fromUtf8("local_guiDlg"));
        local_guiDlg->resize(822, 336);
        horizontalLayout_2 = new QHBoxLayout(local_guiDlg);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        groupBox = new QGroupBox(local_guiDlg);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setMaximumSize(QSize(200, 16777215));
        horizontalLayout = new QHBoxLayout(groupBox);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        formLayout = new QFormLayout();
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        formLayout->setLabelAlignment(Qt::AlignCenter);
        label = new QLabel(groupBox);
        label->setObjectName(QString::fromUtf8("label"));

        formLayout->setWidget(0, QFormLayout::LabelRole, label);

        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        formLayout->setWidget(1, QFormLayout::LabelRole, label_2);

        x_spinbox = new QSpinBox(groupBox);
        x_spinbox->setObjectName(QString::fromUtf8("x_spinbox"));
        x_spinbox->setMaximumSize(QSize(150, 16777215));
        x_spinbox->setMinimum(-10000);
        x_spinbox->setMaximum(10000);

        formLayout->setWidget(0, QFormLayout::FieldRole, x_spinbox);

        z_spinbox = new QSpinBox(groupBox);
        z_spinbox->setObjectName(QString::fromUtf8("z_spinbox"));
        z_spinbox->setMaximumSize(QSize(150, 16777215));
        z_spinbox->setMinimum(-10000);
        z_spinbox->setMaximum(10000);

        formLayout->setWidget(1, QFormLayout::FieldRole, z_spinbox);

        send_button = new QPushButton(groupBox);
        send_button->setObjectName(QString::fromUtf8("send_button"));
        send_button->setMaximumSize(QSize(100, 16777215));

        formLayout->setWidget(2, QFormLayout::FieldRole, send_button);

        stop_button = new QPushButton(groupBox);
        stop_button->setObjectName(QString::fromUtf8("stop_button"));

        formLayout->setWidget(3, QFormLayout::FieldRole, stop_button);


        horizontalLayout->addLayout(formLayout);


        horizontalLayout_2->addWidget(groupBox);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Fixed, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer);

        groupBox_2 = new QGroupBox(local_guiDlg);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        horizontalLayout_6 = new QHBoxLayout(groupBox_2);
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        verticalLayout_13 = new QVBoxLayout();
        verticalLayout_13->setObjectName(QString::fromUtf8("verticalLayout_13"));
        robotMov_checkbox = new QCheckBox(groupBox_2);
        robotMov_checkbox->setObjectName(QString::fromUtf8("robotMov_checkbox"));
        robotMov_checkbox->setChecked(true);

        verticalLayout_13->addWidget(robotMov_checkbox);

        autoMov_checkbox = new QCheckBox(groupBox_2);
        autoMov_checkbox->setObjectName(QString::fromUtf8("autoMov_checkbox"));
        autoMov_checkbox->setChecked(false);

        verticalLayout_13->addWidget(autoMov_checkbox);


        horizontalLayout_6->addLayout(verticalLayout_13);

        horizontalSpacer_2 = new QSpacerItem(30, 20, QSizePolicy::Maximum, QSizePolicy::Minimum);

        horizontalLayout_6->addItem(horizontalSpacer_2);

        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        label_5 = new QLabel(groupBox_2);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        horizontalLayout_9->addWidget(label_5);

        ki_slider = new QSlider(groupBox_2);
        ki_slider->setObjectName(QString::fromUtf8("ki_slider"));
        ki_slider->setMaximum(500);
        ki_slider->setValue(320);
        ki_slider->setOrientation(Qt::Horizontal);

        horizontalLayout_9->addWidget(ki_slider);


        verticalLayout_4->addLayout(horizontalLayout_9);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        label_4 = new QLabel(groupBox_2);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        horizontalLayout_8->addWidget(label_4);

        ke_slider = new QSlider(groupBox_2);
        ke_slider->setObjectName(QString::fromUtf8("ke_slider"));
        ke_slider->setMaximum(500);
        ke_slider->setValue(40);
        ke_slider->setSliderPosition(40);
        ke_slider->setOrientation(Qt::Horizontal);

        horizontalLayout_8->addWidget(ke_slider);


        verticalLayout_4->addLayout(horizontalLayout_8);


        horizontalLayout_6->addLayout(verticalLayout_4);


        horizontalLayout_2->addWidget(groupBox_2);

        line = new QFrame(local_guiDlg);
        line->setObjectName(QString::fromUtf8("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        horizontalLayout_2->addWidget(line);


        retranslateUi(local_guiDlg);

        QMetaObject::connectSlotsByName(local_guiDlg);
    } // setupUi

    void retranslateUi(QWidget *local_guiDlg)
    {
        local_guiDlg->setWindowTitle(QApplication::translate("local_guiDlg", "navigationComp", nullptr));
        groupBox->setTitle(QApplication::translate("local_guiDlg", "Send to", nullptr));
        label->setText(QApplication::translate("local_guiDlg", "x       ", nullptr));
        label_2->setText(QApplication::translate("local_guiDlg", " z       ", nullptr));
        send_button->setText(QApplication::translate("local_guiDlg", "send", nullptr));
        stop_button->setText(QApplication::translate("local_guiDlg", "Stop", nullptr));
        groupBox_2->setTitle(QApplication::translate("local_guiDlg", "Robot", nullptr));
        robotMov_checkbox->setText(QApplication::translate("local_guiDlg", "Move Robot", nullptr));
        autoMov_checkbox->setText(QApplication::translate("local_guiDlg", "Auto movement", nullptr));
        label_5->setText(QApplication::translate("local_guiDlg", "Internal forces", nullptr));
        label_4->setText(QApplication::translate("local_guiDlg", "External forces", nullptr));
    } // retranslateUi

};

namespace Ui {
    class local_guiDlg: public Ui_local_guiDlg {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LOCALUI_H
