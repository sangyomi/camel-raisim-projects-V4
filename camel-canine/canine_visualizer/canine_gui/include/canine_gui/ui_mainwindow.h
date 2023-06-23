/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QWidget>
#include "qcustomplot.hpp"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QGroupBox *groupBox_Init;
    QPushButton *BT_CAN_ON;
    QPushButton *BT_VISUAL_ON;
    QPushButton *BT_MOTOR_ON;
    QPushButton *BT_MOTOR_OFF;
    QLineEdit *LE_CAN_STATUS;
    QLineEdit *LE_VISUAL_STATUS;
    QLineEdit *LE_MOTOR_ON_STATUS;
    QLineEdit *LE_MOTOR_OFF_STATUS;

    QGroupBox *groupBox_Simul;
    QPushButton *BT_SIMUL_ON;
    QPushButton *BT_SIMUL_OFF;
    QLineEdit *LE_SIMUL_ON_STATUS;
    QLineEdit *LE_SIMUL_OFF_STATUS;

    QGroupBox *groupBox_Gait;
    QPushButton *BT_STAND;
    QPushButton *BT_TROT_SLOW;
    QPushButton *BT_TROT_FAST;
    QPushButton *BT_OVERLAP_TROT_FAST;
    QLineEdit *LE_STAND;
    QLineEdit *LE_TROT;
    QLineEdit *LE_CUSTOM_GAIT1;
    QLineEdit *LE_CUSTOM_GAIT2;

    QGroupBox *groupBox_control;
    QPushButton *BT_HOME;
    QPushButton *BT_READY;
    QPushButton *BT_CUSTOM1;
    QPushButton *BT_CUSTOM2;
    QLineEdit *LE_HOME;
    QLineEdit *LE_READY;
    QLineEdit *LE_CUSTOM1;
    QLineEdit *LE_CUSTOM2;

    QTableWidget *TW_MOTOR;
    QTabWidget *tabWidget;
    QWidget *TAB_LINEAR;
    QCustomPlot *PLOT_POS_BASE_Z;
    QCustomPlot *PLOT_VEL_BASE_Z;
    QCustomPlot *PLOT_POS_BASE_Y;
    QCustomPlot *PLOT_VEL_BASE_Y;
    QCustomPlot *PLOT_VEL_BASE_X;
    QCustomPlot *PLOT_POS_BASE_X;
    QWidget *TAB_ANGULAR;
    QCustomPlot *PLOT_POS_BASE_ROLL;
    QCustomPlot *PLOT_VEL_BASE_ROLL;
    QCustomPlot *PLOT_POS_BASE_PITCH;
    QCustomPlot *PLOT_VEL_BASE_PITCH;
    QCustomPlot *PLOT_POS_BASE_YAW;
    QCustomPlot *PLOT_VEL_BASE_YAW;
    QWidget *TAB_FORWARD;
    QCustomPlot *PLOT_VEL_LFKP;
    QCustomPlot *PLOT_VEL_RFKP;
    QCustomPlot *PLOT_VEL_LFHR;
    QCustomPlot *PLOT_VEL_RFHR;
    QCustomPlot *PLOT_VEL_LFHP;
    QCustomPlot *PLOT_VEL_RFHP;
    QWidget *TAB_BACKWARD;
    QCustomPlot *PLOT_VEL_LBHR;
    QCustomPlot *PLOT_VEL_LBHP;
    QCustomPlot *PLOT_VEL_RBHR;
    QCustomPlot *PLOT_VEL_LBKP;
    QCustomPlot *PLOT_VEL_RBHP;
    QCustomPlot *PLOT_VEL_RBKP;
    QWidget *TAB_CUSTOM;
    QCustomPlot *PLOT_CUSTOM_1;
    QCustomPlot *PLOT_CUSTOM_2;
    QCustomPlot *PLOT_CUSTOM_3;
    QCustomPlot *PLOT_CUSTOM_4;
    QCustomPlot *PLOT_CUSTOM_5;
    QCustomPlot *PLOT_CUSTOM_6;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1313, 814);
        QFont font1;
        font1.setPointSize(11);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        groupBox_Init = new QGroupBox(centralwidget);
        groupBox_Init->setObjectName(QString::fromUtf8("groupBox_Init"));
        groupBox_Init->setGeometry(QRect(10, 0, 431, 96));
        BT_CAN_ON = new QPushButton(groupBox_Init);
        BT_CAN_ON->setObjectName(QString::fromUtf8("BT_CAN_ON"));
        BT_CAN_ON->setGeometry(QRect(13, 35, 91, 51));
        BT_CAN_ON->setFont(font1);
        BT_VISUAL_ON = new QPushButton(groupBox_Init);
        BT_VISUAL_ON->setObjectName(QString::fromUtf8("BT_VISUAL_ON"));
        BT_VISUAL_ON->setGeometry(QRect(118, 35, 91, 51));
        BT_VISUAL_ON->setFont(font1);
        BT_MOTOR_ON = new QPushButton(groupBox_Init);
        BT_MOTOR_ON->setObjectName(QString::fromUtf8("BT_MOTOR_ON"));
        BT_MOTOR_ON->setGeometry(QRect(223, 35, 91, 51));
        BT_MOTOR_ON->setFont(font1);
        BT_MOTOR_OFF = new QPushButton(groupBox_Init);
        BT_MOTOR_OFF->setObjectName(QString::fromUtf8("BT_MOTOR_OFF"));
        BT_MOTOR_OFF->setGeometry(QRect(328, 35, 91, 51));
        BT_MOTOR_OFF->setFont(font1);
        BT_MOTOR_OFF->setAutoFillBackground(false);
        LE_CAN_STATUS = new QLineEdit(groupBox_Init);
        LE_CAN_STATUS->setObjectName(QString::fromUtf8("LE_CAN_STATUS"));
        LE_CAN_STATUS->setEnabled(false);
        LE_CAN_STATUS->setGeometry(QRect(37, 23, 43, 8));
        LE_CAN_STATUS->setMouseTracking(false);
        LE_VISUAL_STATUS = new QLineEdit(groupBox_Init);
        LE_VISUAL_STATUS->setObjectName(QString::fromUtf8("LE_VISUAL_STATUS"));
        LE_VISUAL_STATUS->setEnabled(false);
        LE_VISUAL_STATUS->setGeometry(QRect(142, 23, 43, 8));
        LE_VISUAL_STATUS->setMouseTracking(false);
        LE_MOTOR_ON_STATUS = new QLineEdit(groupBox_Init);
        LE_MOTOR_ON_STATUS->setObjectName(QString::fromUtf8("LE_MOTOR_ON_STATUS"));
        LE_MOTOR_ON_STATUS->setEnabled(false);
        LE_MOTOR_ON_STATUS->setGeometry(QRect(247, 23, 43, 8));
        LE_MOTOR_ON_STATUS->setMouseTracking(false);
        LE_MOTOR_OFF_STATUS = new QLineEdit(groupBox_Init);
        LE_MOTOR_OFF_STATUS->setObjectName(QString::fromUtf8("LE_MOTOR_OFF_STATUS"));
        LE_MOTOR_OFF_STATUS->setEnabled(false);
        LE_MOTOR_OFF_STATUS->setGeometry(QRect(352, 23, 43, 8));
        LE_MOTOR_OFF_STATUS->setMouseTracking(false);

        groupBox_Simul = new QGroupBox(centralwidget);
        groupBox_Simul->setObjectName(QString::fromUtf8("groupBox_Simul"));
        groupBox_Simul->setGeometry(QRect(10, 100, 431, 96));
        BT_SIMUL_ON = new QPushButton(groupBox_Simul);
        BT_SIMUL_ON->setObjectName(QString::fromUtf8("BT_SIMUL_ON"));
        BT_SIMUL_ON->setGeometry(QRect(13, 35, 91, 51));
        BT_SIMUL_ON->setFont(font1);
        BT_SIMUL_OFF = new QPushButton(groupBox_Simul);
        BT_SIMUL_OFF->setObjectName(QString::fromUtf8("BT_SIMUL_OFF"));
        BT_SIMUL_OFF->setGeometry(QRect(118, 35, 91, 51));
        BT_SIMUL_OFF->setFont(font1);
        LE_SIMUL_ON_STATUS = new QLineEdit(groupBox_Simul);
        LE_SIMUL_ON_STATUS->setObjectName(QString::fromUtf8("LE_SIMUL_ON_STATUS"));
        LE_SIMUL_ON_STATUS->setEnabled(false);
        LE_SIMUL_ON_STATUS->setGeometry(QRect(37, 23, 43, 8));
        LE_SIMUL_ON_STATUS->setMouseTracking(false);
        LE_SIMUL_OFF_STATUS = new QLineEdit(groupBox_Simul);
        LE_SIMUL_OFF_STATUS->setObjectName(QString::fromUtf8("LE_SIMUL_OFF_STATUS"));
        LE_SIMUL_OFF_STATUS->setEnabled(false);
        LE_SIMUL_OFF_STATUS->setGeometry(QRect(142, 23, 43, 8));
        LE_SIMUL_OFF_STATUS->setMouseTracking(false);

        groupBox_Gait = new QGroupBox(centralwidget);
        groupBox_Gait->setObjectName(QString::fromUtf8("groupBox_control"));
        groupBox_Gait->setGeometry(QRect(10, 200, 431, 101));
        BT_STAND = new QPushButton(groupBox_Gait);
        BT_STAND->setObjectName(QString::fromUtf8("BT_STAND"));
        BT_STAND->setGeometry(QRect(13, 39, 91, 51));
        BT_STAND->setFont(font1);
        BT_TROT_SLOW = new QPushButton(groupBox_Gait);
        BT_TROT_SLOW->setObjectName(QString::fromUtf8("BT_TROT_SLOW"));
        BT_TROT_SLOW->setGeometry(QRect(118, 39, 91, 51));
        BT_TROT_SLOW->setFont(font1);
        BT_TROT_FAST = new QPushButton(groupBox_Gait);
        BT_TROT_FAST->setObjectName(QString::fromUtf8("BT_TROT_FAST"));
        BT_TROT_FAST->setGeometry(QRect(223, 39, 91, 51));
        BT_TROT_FAST->setFont(font1);
        BT_OVERLAP_TROT_FAST = new QPushButton(groupBox_Gait);
        BT_OVERLAP_TROT_FAST->setObjectName(QString::fromUtf8("BT_OVERLAP_TROT_FAST"));
        BT_OVERLAP_TROT_FAST->setGeometry(QRect(328, 39, 91, 51));
        BT_OVERLAP_TROT_FAST->setFont(font1);
        BT_OVERLAP_TROT_FAST->setAutoFillBackground(false);
        LE_STAND = new QLineEdit(groupBox_Gait);
        LE_STAND->setObjectName(QString::fromUtf8("LE_STAND"));
        LE_STAND->setEnabled(false);
        LE_STAND->setGeometry(QRect(37, 27, 43, 8));
        LE_STAND->setMouseTracking(false);
        LE_TROT = new QLineEdit(groupBox_Gait);
        LE_TROT->setObjectName(QString::fromUtf8("LE_TROT"));
        LE_TROT->setEnabled(false);
        LE_TROT->setGeometry(QRect(142, 27, 43, 8));
        LE_TROT->setMouseTracking(false);
        LE_CUSTOM_GAIT1 = new QLineEdit(groupBox_Gait);
        LE_CUSTOM_GAIT1->setObjectName(QString::fromUtf8("LE_CUSTOM_GAIT1"));
        LE_CUSTOM_GAIT1->setEnabled(false);
        LE_CUSTOM_GAIT1->setGeometry(QRect(247, 27, 43, 8));
        LE_CUSTOM_GAIT1->setMouseTracking(false);
        LE_CUSTOM_GAIT2 = new QLineEdit(groupBox_Gait);
        LE_CUSTOM_GAIT2->setObjectName(QString::fromUtf8("LE_CUSTOM_GAIT2"));
        LE_CUSTOM_GAIT2->setEnabled(false);
        LE_CUSTOM_GAIT2->setGeometry(QRect(352, 27, 43, 8));
        LE_CUSTOM_GAIT2->setMouseTracking(false);

        groupBox_control = new QGroupBox(centralwidget);
        groupBox_control->setObjectName(QString::fromUtf8("groupBox_control"));
        groupBox_control->setGeometry(QRect(10, 300, 431, 101));
        BT_HOME = new QPushButton(groupBox_control);
        BT_HOME->setObjectName(QString::fromUtf8("BT_HOME"));
        BT_HOME->setGeometry(QRect(13, 39, 91, 51));
        BT_HOME->setFont(font1);
        BT_READY = new QPushButton(groupBox_control);
        BT_READY->setObjectName(QString::fromUtf8("BT_READY"));
        BT_READY->setGeometry(QRect(118, 39, 91, 51));
        BT_READY->setFont(font1);
        BT_CUSTOM1 = new QPushButton(groupBox_control);
        BT_CUSTOM1->setObjectName(QString::fromUtf8("BT_CUSTOM1"));
        BT_CUSTOM1->setGeometry(QRect(223, 39, 91, 51));
        BT_CUSTOM1->setFont(font1);
        BT_CUSTOM2 = new QPushButton(groupBox_control);
        BT_CUSTOM2->setObjectName(QString::fromUtf8("BT_CUSTOM2"));
        BT_CUSTOM2->setGeometry(QRect(328, 39, 91, 51));
        BT_CUSTOM2->setFont(font1);
        BT_CUSTOM2->setAutoFillBackground(false);
        LE_HOME = new QLineEdit(groupBox_control);
        LE_HOME->setObjectName(QString::fromUtf8("LE_HOME"));
        LE_HOME->setEnabled(false);
        LE_HOME->setGeometry(QRect(37, 27, 43, 8));
        LE_HOME->setMouseTracking(false);
        LE_READY = new QLineEdit(groupBox_control);
        LE_READY->setObjectName(QString::fromUtf8("LE_READY"));
        LE_READY->setEnabled(false);
        LE_READY->setGeometry(QRect(142, 27, 43, 8));
        LE_READY->setMouseTracking(false);
        LE_CUSTOM1 = new QLineEdit(groupBox_control);
        LE_CUSTOM1->setObjectName(QString::fromUtf8("LE_CUSTOM1"));
        LE_CUSTOM1->setEnabled(false);
        LE_CUSTOM1->setGeometry(QRect(247, 27, 43, 8));
        LE_CUSTOM1->setMouseTracking(false);
        LE_CUSTOM2 = new QLineEdit(groupBox_control);
        LE_CUSTOM2->setObjectName(QString::fromUtf8("LE_CUSTOM2"));
        LE_CUSTOM2->setEnabled(false);
        LE_CUSTOM2->setGeometry(QRect(352, 27, 43, 8));
        LE_CUSTOM2->setMouseTracking(false);

        TW_MOTOR = new QTableWidget(centralwidget);
        TW_MOTOR->setObjectName(QString::fromUtf8("TW_MOTOR"));
        TW_MOTOR->setGeometry(QRect(10, 420, 431, 101));

        tabWidget = new QTabWidget(centralwidget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setGeometry(QRect(450, 10, 851, 761));
        TAB_LINEAR = new QWidget();
        TAB_LINEAR->setObjectName(QString::fromUtf8("TAB_LINEAR"));
        PLOT_POS_BASE_Z = new QCustomPlot(TAB_LINEAR);
        PLOT_POS_BASE_Z->setObjectName(QString::fromUtf8("PLOT_POS_BASE_Z"));
        PLOT_POS_BASE_Z->setGeometry(QRect(10, 490, 411, 230));
        PLOT_VEL_BASE_Z = new QCustomPlot(TAB_LINEAR);
        PLOT_VEL_BASE_Z->setObjectName(QString::fromUtf8("PLOT_VEL_BASE_Z"));
        PLOT_VEL_BASE_Z->setGeometry(QRect(430, 490, 411, 230));
        PLOT_POS_BASE_Y = new QCustomPlot(TAB_LINEAR);
        PLOT_POS_BASE_Y->setObjectName(QString::fromUtf8("PLOT_POS_BASE_Y"));
        PLOT_POS_BASE_Y->setGeometry(QRect(10, 250, 411, 230));
        PLOT_VEL_BASE_Y = new QCustomPlot(TAB_LINEAR);
        PLOT_VEL_BASE_Y->setObjectName(QString::fromUtf8("PLOT_VEL_BASE_Y"));
        PLOT_VEL_BASE_Y->setGeometry(QRect(430, 250, 411, 230));
        PLOT_VEL_BASE_X = new QCustomPlot(TAB_LINEAR);
        PLOT_VEL_BASE_X->setObjectName(QString::fromUtf8("PLOT_VEL_BASE_X"));
        PLOT_VEL_BASE_X->setGeometry(QRect(430, 10, 411, 230));
        PLOT_POS_BASE_X = new QCustomPlot(TAB_LINEAR);
        PLOT_POS_BASE_X->setObjectName(QString::fromUtf8("PLOT_POS_BASE_X"));
        PLOT_POS_BASE_X->setGeometry(QRect(10, 10, 411, 230));
        tabWidget->addTab(TAB_LINEAR, QString());
        TAB_ANGULAR = new QWidget();
        TAB_ANGULAR->setObjectName(QString::fromUtf8("TAB_ANGULAR"));
        PLOT_POS_BASE_YAW = new QCustomPlot(TAB_ANGULAR);
        PLOT_POS_BASE_YAW->setObjectName(QString::fromUtf8("PLOT_POS_BASE_YAW"));
        PLOT_POS_BASE_YAW->setGeometry(QRect(10, 490, 411, 230));
        PLOT_VEL_BASE_YAW = new QCustomPlot(TAB_ANGULAR);
        PLOT_VEL_BASE_YAW->setObjectName(QString::fromUtf8("PLOT_VEL_BASE_YAW"));
        PLOT_VEL_BASE_YAW->setGeometry(QRect(430, 490, 411, 230));
        PLOT_POS_BASE_ROLL = new QCustomPlot(TAB_ANGULAR);
        PLOT_POS_BASE_ROLL->setObjectName(QString::fromUtf8("PLOT_POS_BASE_ROLL"));
        PLOT_POS_BASE_ROLL->setGeometry(QRect(10, 10, 411, 230));
        PLOT_VEL_BASE_ROLL = new QCustomPlot(TAB_ANGULAR);
        PLOT_VEL_BASE_ROLL->setObjectName(QString::fromUtf8("PLOT_VEL_BASE_ROLL"));
        PLOT_VEL_BASE_ROLL->setGeometry(QRect(430, 10, 411, 230));
        PLOT_POS_BASE_PITCH = new QCustomPlot(TAB_ANGULAR);
        PLOT_POS_BASE_PITCH->setObjectName(QString::fromUtf8("PLOT_POS_BASE_PITCH"));
        PLOT_POS_BASE_PITCH->setGeometry(QRect(10, 250, 411, 230));
        PLOT_VEL_BASE_PITCH = new QCustomPlot(TAB_ANGULAR);
        PLOT_VEL_BASE_PITCH->setObjectName(QString::fromUtf8("PLOT_VEL_BASE_PITCH"));
        PLOT_VEL_BASE_PITCH->setGeometry(QRect(430, 250, 411, 230));
        tabWidget->addTab(TAB_ANGULAR, QString());
        TAB_FORWARD = new QWidget();
        TAB_FORWARD->setObjectName(QString::fromUtf8("TAB_FORWARD"));
        PLOT_VEL_LFKP = new QCustomPlot(TAB_FORWARD);
        PLOT_VEL_LFKP->setObjectName(QString::fromUtf8("PLOT_VEL_LFKP"));
        PLOT_VEL_LFKP->setGeometry(QRect(10, 490, 411, 230));
        PLOT_VEL_RFKP = new QCustomPlot(TAB_FORWARD);
        PLOT_VEL_RFKP->setObjectName(QString::fromUtf8("PLOT_VEL_RFKP"));
        PLOT_VEL_RFKP->setGeometry(QRect(430, 490, 411, 230));
        PLOT_VEL_LFHR = new QCustomPlot(TAB_FORWARD);
        PLOT_VEL_LFHR->setObjectName(QString::fromUtf8("PLOT_VEL_LFHR"));
        PLOT_VEL_LFHR->setGeometry(QRect(10, 10, 411, 230));
        PLOT_VEL_RFHR = new QCustomPlot(TAB_FORWARD);
        PLOT_VEL_RFHR->setObjectName(QString::fromUtf8("PLOT_VEL_RFHR"));
        PLOT_VEL_RFHR->setGeometry(QRect(430, 10, 411, 230));
        PLOT_VEL_LFHP = new QCustomPlot(TAB_FORWARD);
        PLOT_VEL_LFHP->setObjectName(QString::fromUtf8("PLOT_VEL_LFHP"));
        PLOT_VEL_LFHP->setGeometry(QRect(10, 250, 411, 230));
        PLOT_VEL_RFHP = new QCustomPlot(TAB_FORWARD);
        PLOT_VEL_RFHP->setObjectName(QString::fromUtf8("PLOT_VEL_RFHP"));
        PLOT_VEL_RFHP->setGeometry(QRect(430, 250, 411, 230));
        tabWidget->addTab(TAB_FORWARD, QString());
        TAB_BACKWARD = new QWidget();
        TAB_BACKWARD->setObjectName(QString::fromUtf8("TAB_BACKWARD"));
        PLOT_VEL_LBHR = new QCustomPlot(TAB_BACKWARD);
        PLOT_VEL_LBHR->setObjectName(QString::fromUtf8("PLOT_VEL_LBHR"));
        PLOT_VEL_LBHR->setGeometry(QRect(10, 10, 411, 230));
        PLOT_VEL_LBHP = new QCustomPlot(TAB_BACKWARD);
        PLOT_VEL_LBHP->setObjectName(QString::fromUtf8("PLOT_VEL_LBHP"));
        PLOT_VEL_LBHP->setGeometry(QRect(10, 250, 411, 230));
        PLOT_VEL_RBHR = new QCustomPlot(TAB_BACKWARD);
        PLOT_VEL_RBHR->setObjectName(QString::fromUtf8("PLOT_VEL_RBHR"));
        PLOT_VEL_RBHR->setGeometry(QRect(430, 10, 411, 230));
        PLOT_VEL_LBKP = new QCustomPlot(TAB_BACKWARD);
        PLOT_VEL_LBKP->setObjectName(QString::fromUtf8("PLOT_VEL_LBKP"));
        PLOT_VEL_LBKP->setGeometry(QRect(10, 490, 411, 230));
        PLOT_VEL_RBHP = new QCustomPlot(TAB_BACKWARD);
        PLOT_VEL_RBHP->setObjectName(QString::fromUtf8("PLOT_VEL_RBHP"));
        PLOT_VEL_RBHP->setGeometry(QRect(430, 250, 411, 230));
        PLOT_VEL_RBKP = new QCustomPlot(TAB_BACKWARD);
        PLOT_VEL_RBKP->setObjectName(QString::fromUtf8("PLOT_VEL_RBKP"));
        PLOT_VEL_RBKP->setGeometry(QRect(430, 490, 411, 230));
        tabWidget->addTab(TAB_BACKWARD, QString());
        TAB_CUSTOM = new QWidget();
        TAB_CUSTOM->setObjectName(QString::fromUtf8("TAB_CUSTOM"));
        QFont font2;
        font2.setFamily(QString::fromUtf8("Ubuntu"));
        font2.setBold(false);
        font2.setWeight(50);
        TAB_CUSTOM->setFont(font2);
        PLOT_CUSTOM_1 = new QCustomPlot(TAB_CUSTOM);
        PLOT_CUSTOM_1->setObjectName(QString::fromUtf8("PLOT_CUSTOM_1"));
        PLOT_CUSTOM_1->setGeometry(QRect(10, 10, 411, 230));
        PLOT_CUSTOM_2 = new QCustomPlot(TAB_CUSTOM);
        PLOT_CUSTOM_2->setObjectName(QString::fromUtf8("PLOT_CUSTOM_2"));
        PLOT_CUSTOM_2->setGeometry(QRect(430, 10, 411, 230));
        PLOT_CUSTOM_3 = new QCustomPlot(TAB_CUSTOM);
        PLOT_CUSTOM_3->setObjectName(QString::fromUtf8("PLOT_CUSTOM_3"));
        PLOT_CUSTOM_3->setGeometry(QRect(10, 250, 411, 230));
        PLOT_CUSTOM_4 = new QCustomPlot(TAB_CUSTOM);
        PLOT_CUSTOM_4->setObjectName(QString::fromUtf8("PLOT_CUSTOM_4"));
        PLOT_CUSTOM_4->setGeometry(QRect(430, 250, 411, 230));
        PLOT_CUSTOM_5 = new QCustomPlot(TAB_CUSTOM);
        PLOT_CUSTOM_5->setObjectName(QString::fromUtf8("PLOT_CUSTOM_5"));
        PLOT_CUSTOM_5->setGeometry(QRect(10, 490, 411, 230));
        PLOT_CUSTOM_6 = new QCustomPlot(TAB_CUSTOM);
        PLOT_CUSTOM_6->setObjectName(QString::fromUtf8("PLOT_CUSTOM_6"));
        PLOT_CUSTOM_6->setGeometry(QRect(430, 490, 411, 230));
        tabWidget->addTab(TAB_CUSTOM, QString());

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1313, 22));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(0);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", nullptr));

        groupBox_Init->setTitle(QApplication::translate("MainWindow", "INIT for real", nullptr));
        BT_CAN_ON->setText(QApplication::translate("MainWindow", "CAN\nON", nullptr));
        BT_VISUAL_ON->setText(QApplication::translate("MainWindow", "Visual\nON", nullptr));
        BT_MOTOR_ON->setText(QApplication::translate("MainWindow", "Motor\nON", nullptr));
        BT_MOTOR_OFF->setText(QApplication::translate("MainWindow", "Motor\nOFF", nullptr));

        groupBox_Simul->setTitle(QApplication::translate("MainWindow", "INIT for simulation", nullptr));
        BT_SIMUL_ON->setText(QApplication::translate("MainWindow", "Simul\nStart", nullptr));
        BT_SIMUL_OFF->setText(QApplication::translate("MainWindow", "Simul\nStop", nullptr));

        groupBox_Gait->setTitle(QApplication::translate("MainWindow", "GAIT", nullptr));
        BT_STAND->setText(QApplication::translate("MainWindow", "Stand", nullptr));
        BT_TROT_SLOW->setText(QApplication::translate("MainWindow", "Trot Slow", nullptr));
        BT_TROT_FAST->setText(QApplication::translate("MainWindow", "Trot Fast", nullptr));
        BT_OVERLAP_TROT_FAST->setText(QApplication::translate("MainWindow", "Overlap trot fast", nullptr));

        groupBox_control->setTitle(QApplication::translate("MainWindow", "CONTROL", nullptr));
        BT_HOME->setText(QApplication::translate("MainWindow", "Home\nPosition", nullptr));
        BT_READY->setText(QApplication::translate("MainWindow", "Ready\nPosition", nullptr));
        BT_CUSTOM1->setText(QApplication::translate("MainWindow", "Custom1", nullptr));
        BT_CUSTOM2->setText(QApplication::translate("MainWindow", "Custom2", nullptr));

        tabWidget->setTabText(tabWidget->indexOf(TAB_LINEAR), QApplication::translate("MainWindow", "Linear", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(TAB_ANGULAR), QApplication::translate("MainWindow", "Angular", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(TAB_FORWARD), QApplication::translate("MainWindow", "Forward", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(TAB_BACKWARD), QApplication::translate("MainWindow", "Backward", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(TAB_CUSTOM), QApplication::translate("MainWindow", "Custom", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
