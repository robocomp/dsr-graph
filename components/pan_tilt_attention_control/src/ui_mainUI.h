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
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_guiDlg
{
public:
    QAction *actionSave;
    QAction *actionStart_Stop;
    QWidget *centralwidget;
    QVBoxLayout *verticalLayout;
    QSplitter *splitter_1;
    QSplitter *splitter_2;
    QMenuBar *menubar;
    QMenu *menuSimulation;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *guiDlg)
    {
        if (guiDlg->objectName().isEmpty())
            guiDlg->setObjectName(QString::fromUtf8("guiDlg"));
        guiDlg->resize(800, 600);
        actionSave = new QAction(guiDlg);
        actionSave->setObjectName(QString::fromUtf8("actionSave"));
        actionStart_Stop = new QAction(guiDlg);
        actionStart_Stop->setObjectName(QString::fromUtf8("actionStart_Stop"));
        centralwidget = new QWidget(guiDlg);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        verticalLayout = new QVBoxLayout(centralwidget);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        splitter_1 = new QSplitter(centralwidget);
        splitter_1->setObjectName(QString::fromUtf8("splitter_1"));
        splitter_1->setOrientation(Qt::Horizontal);
        splitter_2 = new QSplitter(splitter_1);
        splitter_2->setObjectName(QString::fromUtf8("splitter_2"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(splitter_2->sizePolicy().hasHeightForWidth());
        splitter_2->setSizePolicy(sizePolicy);
        splitter_2->setOrientation(Qt::Vertical);
        splitter_1->addWidget(splitter_2);

        verticalLayout->addWidget(splitter_1);

        guiDlg->setCentralWidget(centralwidget);
        menubar = new QMenuBar(guiDlg);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 800, 22));
        menuSimulation = new QMenu(menubar);
        menuSimulation->setObjectName(QString::fromUtf8("menuSimulation"));
        guiDlg->setMenuBar(menubar);
        statusbar = new QStatusBar(guiDlg);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        guiDlg->setStatusBar(statusbar);

        menubar->addAction(menuSimulation->menuAction());
        menuSimulation->addAction(actionStart_Stop);

        retranslateUi(guiDlg);

        QMetaObject::connectSlotsByName(guiDlg);
    } // setupUi

    void retranslateUi(QMainWindow *guiDlg)
    {
        guiDlg->setWindowTitle(QApplication::translate("guiDlg", "yolov4_tracker", nullptr));
        actionSave->setText(QApplication::translate("guiDlg", "Save", nullptr));
        actionStart_Stop->setText(QApplication::translate("guiDlg", "Start/Stop", nullptr));
        menuSimulation->setTitle(QApplication::translate("guiDlg", "Simulation", nullptr));
    } // retranslateUi

};

namespace Ui {
    class guiDlg: public Ui_guiDlg {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINUI_H
