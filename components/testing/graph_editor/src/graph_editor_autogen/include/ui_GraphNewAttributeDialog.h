/********************************************************************************
** Form generated from reading UI file 'GraphNewAttributeDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_GRAPHNEWATTRIBUTEDIALOG_H
#define UI_GRAPHNEWATTRIBUTEDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_NewAttributeDialog
{
public:
    QVBoxLayout *verticalLayout;
    QGroupBox *node_group;
    QFormLayout *formLayout;
    QLabel *label;
    QLabel *label_3;
    QComboBox *att_names_cmb;
    QLineEdit *att_type;
    QCheckBox *filter_types_checkbox;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *NewAttributeDialog)
    {
        if (NewAttributeDialog->objectName().isEmpty())
            NewAttributeDialog->setObjectName(QString::fromUtf8("NewAttributeDialog"));
        NewAttributeDialog->resize(492, 179);
        verticalLayout = new QVBoxLayout(NewAttributeDialog);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        node_group = new QGroupBox(NewAttributeDialog);
        node_group->setObjectName(QString::fromUtf8("node_group"));
        formLayout = new QFormLayout(node_group);
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        label = new QLabel(node_group);
        label->setObjectName(QString::fromUtf8("label"));

        formLayout->setWidget(0, QFormLayout::LabelRole, label);

        label_3 = new QLabel(node_group);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        formLayout->setWidget(1, QFormLayout::LabelRole, label_3);

        att_names_cmb = new QComboBox(node_group);
        att_names_cmb->setObjectName(QString::fromUtf8("att_names_cmb"));

        formLayout->setWidget(0, QFormLayout::FieldRole, att_names_cmb);

        att_type = new QLineEdit(node_group);
        att_type->setObjectName(QString::fromUtf8("att_type"));
        att_type->setEnabled(false);
        att_type->setReadOnly(true);

        formLayout->setWidget(1, QFormLayout::FieldRole, att_type);

        filter_types_checkbox = new QCheckBox(node_group);
        filter_types_checkbox->setObjectName(QString::fromUtf8("filter_types_checkbox"));
        filter_types_checkbox->setChecked(true);

        formLayout->setWidget(2, QFormLayout::FieldRole, filter_types_checkbox);


        verticalLayout->addWidget(node_group);

        buttonBox = new QDialogButtonBox(NewAttributeDialog);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

        verticalLayout->addWidget(buttonBox);


        retranslateUi(NewAttributeDialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), NewAttributeDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), NewAttributeDialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(NewAttributeDialog);
    } // setupUi

    void retranslateUi(QDialog *NewAttributeDialog)
    {
        NewAttributeDialog->setWindowTitle(QApplication::translate("NewAttributeDialog", "Dialog", nullptr));
        node_group->setTitle(QApplication::translate("NewAttributeDialog", "Attribute", nullptr));
        label->setText(QApplication::translate("NewAttributeDialog", "Name", nullptr));
        label_3->setText(QApplication::translate("NewAttributeDialog", "Type", nullptr));
        filter_types_checkbox->setText(QApplication::translate("NewAttributeDialog", "Filter not known types", nullptr));
    } // retranslateUi

};

namespace Ui {
    class NewAttributeDialog: public Ui_NewAttributeDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_GRAPHNEWATTRIBUTEDIALOG_H
