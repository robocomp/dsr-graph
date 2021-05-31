/********************************************************************************
** Form generated from reading UI file 'GraphNewElementDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_GRAPHNEWELEMENTDIALOG_H
#define UI_GRAPHNEWELEMENTDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_NewElementDlg
{
public:
    QVBoxLayout *verticalLayout;
    QGroupBox *node_group;
    QFormLayout *formLayout;
    QLabel *label;
    QLineEdit *node_name_lbl;
    QLabel *label_3;
    QComboBox *node_type_cmb;
    QGroupBox *node_group_2;
    QFormLayout *formLayout_4;
    QLabel *label_6;
    QLabel *label_7;
    QLineEdit *node_name_lbl_2;
    QComboBox *node_type_cmb_2;
    QGroupBox *edge_group;
    QFormLayout *formLayout_2;
    QLabel *label_4;
    QComboBox *edge_type_cmb;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *NewElementDlg)
    {
        if (NewElementDlg->objectName().isEmpty())
            NewElementDlg->setObjectName(QString::fromUtf8("NewElementDlg"));
        NewElementDlg->resize(416, 313);
        verticalLayout = new QVBoxLayout(NewElementDlg);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        node_group = new QGroupBox(NewElementDlg);
        node_group->setObjectName(QString::fromUtf8("node_group"));
        formLayout = new QFormLayout(node_group);
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        label = new QLabel(node_group);
        label->setObjectName(QString::fromUtf8("label"));

        formLayout->setWidget(0, QFormLayout::LabelRole, label);

        node_name_lbl = new QLineEdit(node_group);
        node_name_lbl->setObjectName(QString::fromUtf8("node_name_lbl"));

        formLayout->setWidget(0, QFormLayout::FieldRole, node_name_lbl);

        label_3 = new QLabel(node_group);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        formLayout->setWidget(1, QFormLayout::LabelRole, label_3);

        node_type_cmb = new QComboBox(node_group);
        node_type_cmb->setObjectName(QString::fromUtf8("node_type_cmb"));

        formLayout->setWidget(1, QFormLayout::FieldRole, node_type_cmb);


        verticalLayout->addWidget(node_group);

        node_group_2 = new QGroupBox(NewElementDlg);
        node_group_2->setObjectName(QString::fromUtf8("node_group_2"));
        formLayout_4 = new QFormLayout(node_group_2);
        formLayout_4->setObjectName(QString::fromUtf8("formLayout_4"));
        label_6 = new QLabel(node_group_2);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        formLayout_4->setWidget(0, QFormLayout::LabelRole, label_6);

        label_7 = new QLabel(node_group_2);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        formLayout_4->setWidget(1, QFormLayout::LabelRole, label_7);

        node_name_lbl_2 = new QLineEdit(node_group_2);
        node_name_lbl_2->setObjectName(QString::fromUtf8("node_name_lbl_2"));

        formLayout_4->setWidget(0, QFormLayout::FieldRole, node_name_lbl_2);

        node_type_cmb_2 = new QComboBox(node_group_2);
        node_type_cmb_2->setObjectName(QString::fromUtf8("node_type_cmb_2"));

        formLayout_4->setWidget(1, QFormLayout::FieldRole, node_type_cmb_2);


        verticalLayout->addWidget(node_group_2);

        edge_group = new QGroupBox(NewElementDlg);
        edge_group->setObjectName(QString::fromUtf8("edge_group"));
        formLayout_2 = new QFormLayout(edge_group);
        formLayout_2->setObjectName(QString::fromUtf8("formLayout_2"));
        label_4 = new QLabel(edge_group);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        formLayout_2->setWidget(0, QFormLayout::LabelRole, label_4);

        edge_type_cmb = new QComboBox(edge_group);
        edge_type_cmb->setObjectName(QString::fromUtf8("edge_type_cmb"));

        formLayout_2->setWidget(0, QFormLayout::FieldRole, edge_type_cmb);


        verticalLayout->addWidget(edge_group);

        buttonBox = new QDialogButtonBox(NewElementDlg);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

        verticalLayout->addWidget(buttonBox);


        retranslateUi(NewElementDlg);
        QObject::connect(buttonBox, SIGNAL(accepted()), NewElementDlg, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), NewElementDlg, SLOT(reject()));

        QMetaObject::connectSlotsByName(NewElementDlg);
    } // setupUi

    void retranslateUi(QDialog *NewElementDlg)
    {
        NewElementDlg->setWindowTitle(QApplication::translate("NewElementDlg", "Dialog", nullptr));
        node_group->setTitle(QApplication::translate("NewElementDlg", "Node", nullptr));
        label->setText(QApplication::translate("NewElementDlg", "Name", nullptr));
        label_3->setText(QApplication::translate("NewElementDlg", "Type", nullptr));
        node_group_2->setTitle(QApplication::translate("NewElementDlg", "Node", nullptr));
        label_6->setText(QApplication::translate("NewElementDlg", "Name", nullptr));
        label_7->setText(QApplication::translate("NewElementDlg", "Type", nullptr));
        edge_group->setTitle(QApplication::translate("NewElementDlg", "Edge", nullptr));
        label_4->setText(QApplication::translate("NewElementDlg", "Type", nullptr));
    } // retranslateUi

};

namespace Ui {
    class NewElementDlg: public Ui_NewElementDlg {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_GRAPHNEWELEMENTDIALOG_H
