/********************************************************************************
** Form generated from reading UI file 'main_window.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAIN_WINDOW_H
#define UI_MAIN_WINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "slam2d_viewer.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionLoad;
    QAction *actionSave;
    QAction *actionQuit;
    QAction *actionShowRobotPath;
    QAction *actionShowFrontiers;
    QAction *actionShowObstacles;
    QWidget *centralwidget;
    QHBoxLayout *horizontalLayout;
    QFrame *controlFrame;
    QVBoxLayout *verticalLayout;
    QGroupBox *explorationGroupBox;
    QVBoxLayout *explorationLayout;
    QPushButton *startExplorationButton;
    QPushButton *stopExplorationButton;
    QPushButton *pauseExplorationButton;
    QLabel *explorationStatus;
    QGroupBox *pathPlanningGroupBox;
    QVBoxLayout *pathPlanningLayout;
    QPushButton *calculatePathButton;
    QPushButton *navigateToGoalButton;
    QPushButton *cancelGoalButton;
    QLabel *pathStatus;
    QGroupBox *mapGroupBox;
    QVBoxLayout *mapLayout;
    QPushButton *saveMapButton;
    QPushButton *loadMapButton;
    QPushButton *resetMapButton;
    QLabel *mapInfo;
    QGroupBox *slamSettingsGroupBox;
    QVBoxLayout *slamSettingsLayout;
    QLabel *label;
    QSpinBox *spIterations;
    QCheckBox *cbCovariances;
    QGroupBox *optimizationMethodGroup;
    QVBoxLayout *verticalLayout_2;
    QRadioButton *rbGauss;
    QRadioButton *rbLevenberg;
    QPushButton *btnOptimize;
    QSpacerItem *verticalSpacer;
    g2o::Slam2DViewer *viewer;
    QMenuBar *menubar;
    QMenu *menuFile;
    QMenu *menuView;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1200, 800);
        actionLoad = new QAction(MainWindow);
        actionLoad->setObjectName(QString::fromUtf8("actionLoad"));
        actionSave = new QAction(MainWindow);
        actionSave->setObjectName(QString::fromUtf8("actionSave"));
        actionQuit = new QAction(MainWindow);
        actionQuit->setObjectName(QString::fromUtf8("actionQuit"));
        actionShowRobotPath = new QAction(MainWindow);
        actionShowRobotPath->setObjectName(QString::fromUtf8("actionShowRobotPath"));
        actionShowRobotPath->setCheckable(true);
        actionShowFrontiers = new QAction(MainWindow);
        actionShowFrontiers->setObjectName(QString::fromUtf8("actionShowFrontiers"));
        actionShowFrontiers->setCheckable(true);
        actionShowObstacles = new QAction(MainWindow);
        actionShowObstacles->setObjectName(QString::fromUtf8("actionShowObstacles"));
        actionShowObstacles->setCheckable(true);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        horizontalLayout = new QHBoxLayout(centralwidget);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        controlFrame = new QFrame(centralwidget);
        controlFrame->setObjectName(QString::fromUtf8("controlFrame"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(controlFrame->sizePolicy().hasHeightForWidth());
        controlFrame->setSizePolicy(sizePolicy);
        controlFrame->setMinimumSize(QSize(250, 0));
        controlFrame->setFrameShape(QFrame::StyledPanel);
        controlFrame->setFrameShadow(QFrame::Raised);
        verticalLayout = new QVBoxLayout(controlFrame);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        explorationGroupBox = new QGroupBox(controlFrame);
        explorationGroupBox->setObjectName(QString::fromUtf8("explorationGroupBox"));
        explorationLayout = new QVBoxLayout(explorationGroupBox);
        explorationLayout->setObjectName(QString::fromUtf8("explorationLayout"));
        startExplorationButton = new QPushButton(explorationGroupBox);
        startExplorationButton->setObjectName(QString::fromUtf8("startExplorationButton"));

        explorationLayout->addWidget(startExplorationButton);

        stopExplorationButton = new QPushButton(explorationGroupBox);
        stopExplorationButton->setObjectName(QString::fromUtf8("stopExplorationButton"));

        explorationLayout->addWidget(stopExplorationButton);

        pauseExplorationButton = new QPushButton(explorationGroupBox);
        pauseExplorationButton->setObjectName(QString::fromUtf8("pauseExplorationButton"));

        explorationLayout->addWidget(pauseExplorationButton);

        explorationStatus = new QLabel(explorationGroupBox);
        explorationStatus->setObjectName(QString::fromUtf8("explorationStatus"));

        explorationLayout->addWidget(explorationStatus);


        verticalLayout->addWidget(explorationGroupBox);

        pathPlanningGroupBox = new QGroupBox(controlFrame);
        pathPlanningGroupBox->setObjectName(QString::fromUtf8("pathPlanningGroupBox"));
        pathPlanningLayout = new QVBoxLayout(pathPlanningGroupBox);
        pathPlanningLayout->setObjectName(QString::fromUtf8("pathPlanningLayout"));
        calculatePathButton = new QPushButton(pathPlanningGroupBox);
        calculatePathButton->setObjectName(QString::fromUtf8("calculatePathButton"));

        pathPlanningLayout->addWidget(calculatePathButton);

        navigateToGoalButton = new QPushButton(pathPlanningGroupBox);
        navigateToGoalButton->setObjectName(QString::fromUtf8("navigateToGoalButton"));

        pathPlanningLayout->addWidget(navigateToGoalButton);

        cancelGoalButton = new QPushButton(pathPlanningGroupBox);
        cancelGoalButton->setObjectName(QString::fromUtf8("cancelGoalButton"));

        pathPlanningLayout->addWidget(cancelGoalButton);

        pathStatus = new QLabel(pathPlanningGroupBox);
        pathStatus->setObjectName(QString::fromUtf8("pathStatus"));

        pathPlanningLayout->addWidget(pathStatus);


        verticalLayout->addWidget(pathPlanningGroupBox);

        mapGroupBox = new QGroupBox(controlFrame);
        mapGroupBox->setObjectName(QString::fromUtf8("mapGroupBox"));
        mapLayout = new QVBoxLayout(mapGroupBox);
        mapLayout->setObjectName(QString::fromUtf8("mapLayout"));
        saveMapButton = new QPushButton(mapGroupBox);
        saveMapButton->setObjectName(QString::fromUtf8("saveMapButton"));

        mapLayout->addWidget(saveMapButton);

        loadMapButton = new QPushButton(mapGroupBox);
        loadMapButton->setObjectName(QString::fromUtf8("loadMapButton"));

        mapLayout->addWidget(loadMapButton);

        resetMapButton = new QPushButton(mapGroupBox);
        resetMapButton->setObjectName(QString::fromUtf8("resetMapButton"));

        mapLayout->addWidget(resetMapButton);

        mapInfo = new QLabel(mapGroupBox);
        mapInfo->setObjectName(QString::fromUtf8("mapInfo"));

        mapLayout->addWidget(mapInfo);


        verticalLayout->addWidget(mapGroupBox);

        slamSettingsGroupBox = new QGroupBox(controlFrame);
        slamSettingsGroupBox->setObjectName(QString::fromUtf8("slamSettingsGroupBox"));
        slamSettingsLayout = new QVBoxLayout(slamSettingsGroupBox);
        slamSettingsLayout->setObjectName(QString::fromUtf8("slamSettingsLayout"));
        label = new QLabel(slamSettingsGroupBox);
        label->setObjectName(QString::fromUtf8("label"));

        slamSettingsLayout->addWidget(label);

        spIterations = new QSpinBox(slamSettingsGroupBox);
        spIterations->setObjectName(QString::fromUtf8("spIterations"));
        spIterations->setMinimum(1);
        spIterations->setMaximum(100);
        spIterations->setValue(10);

        slamSettingsLayout->addWidget(spIterations);

        cbCovariances = new QCheckBox(slamSettingsGroupBox);
        cbCovariances->setObjectName(QString::fromUtf8("cbCovariances"));

        slamSettingsLayout->addWidget(cbCovariances);

        optimizationMethodGroup = new QGroupBox(slamSettingsGroupBox);
        optimizationMethodGroup->setObjectName(QString::fromUtf8("optimizationMethodGroup"));
        verticalLayout_2 = new QVBoxLayout(optimizationMethodGroup);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        rbGauss = new QRadioButton(optimizationMethodGroup);
        rbGauss->setObjectName(QString::fromUtf8("rbGauss"));
        rbGauss->setChecked(true);

        verticalLayout_2->addWidget(rbGauss);

        rbLevenberg = new QRadioButton(optimizationMethodGroup);
        rbLevenberg->setObjectName(QString::fromUtf8("rbLevenberg"));

        verticalLayout_2->addWidget(rbLevenberg);


        slamSettingsLayout->addWidget(optimizationMethodGroup);

        btnOptimize = new QPushButton(slamSettingsGroupBox);
        btnOptimize->setObjectName(QString::fromUtf8("btnOptimize"));

        slamSettingsLayout->addWidget(btnOptimize);


        verticalLayout->addWidget(slamSettingsGroupBox);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);


        horizontalLayout->addWidget(controlFrame);

        viewer = new g2o::Slam2DViewer(centralwidget);
        viewer->setObjectName(QString::fromUtf8("viewer"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(1);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(viewer->sizePolicy().hasHeightForWidth());
        viewer->setSizePolicy(sizePolicy1);

        horizontalLayout->addWidget(viewer);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1200, 25));
        menuFile = new QMenu(menubar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        menuView = new QMenu(menubar);
        menuView->setObjectName(QString::fromUtf8("menuView"));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        menubar->addAction(menuFile->menuAction());
        menubar->addAction(menuView->menuAction());
        menuFile->addAction(actionLoad);
        menuFile->addAction(actionSave);
        menuFile->addSeparator();
        menuFile->addAction(actionQuit);
        menuView->addAction(actionShowRobotPath);
        menuView->addAction(actionShowFrontiers);
        menuView->addAction(actionShowObstacles);

        retranslateUi(MainWindow);
        QObject::connect(startExplorationButton, SIGNAL(clicked()), MainWindow, SLOT(startExploration()));
        QObject::connect(stopExplorationButton, SIGNAL(clicked()), MainWindow, SLOT(stopExploration()));
        QObject::connect(saveMapButton, SIGNAL(clicked()), MainWindow, SLOT(saveMap()));
        QObject::connect(loadMapButton, SIGNAL(clicked()), MainWindow, SLOT(loadMap()));
        QObject::connect(actionQuit, SIGNAL(triggered()), MainWindow, SLOT(close()));

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "GraphSLAM \345\217\257\350\247\206\345\214\226\344\270\216\350\267\257\345\276\204\350\247\204\345\210\222", nullptr));
        actionLoad->setText(QCoreApplication::translate("MainWindow", "\345\212\240\350\275\275...", nullptr));
        actionSave->setText(QCoreApplication::translate("MainWindow", "\344\277\235\345\255\230...", nullptr));
        actionQuit->setText(QCoreApplication::translate("MainWindow", "\351\200\200\345\207\272", nullptr));
        actionShowRobotPath->setText(QCoreApplication::translate("MainWindow", "\346\230\276\347\244\272\346\234\272\345\231\250\344\272\272\350\267\257\345\276\204", nullptr));
        actionShowFrontiers->setText(QCoreApplication::translate("MainWindow", "\346\230\276\347\244\272\350\276\271\347\225\214\347\202\271", nullptr));
        actionShowObstacles->setText(QCoreApplication::translate("MainWindow", "\346\230\276\347\244\272\351\232\234\347\242\215\347\211\251", nullptr));
        explorationGroupBox->setTitle(QCoreApplication::translate("MainWindow", "\346\216\242\347\264\242\346\216\247\345\210\266", nullptr));
        startExplorationButton->setText(QCoreApplication::translate("MainWindow", "\345\274\200\345\247\213\350\207\252\345\212\250\346\216\242\347\264\242", nullptr));
        stopExplorationButton->setText(QCoreApplication::translate("MainWindow", "\345\201\234\346\255\242\346\216\242\347\264\242", nullptr));
        pauseExplorationButton->setText(QCoreApplication::translate("MainWindow", "\346\232\202\345\201\234/\347\273\247\347\273\255", nullptr));
        explorationStatus->setText(QCoreApplication::translate("MainWindow", "\347\212\266\346\200\201: \346\234\252\345\274\200\345\247\213", nullptr));
        pathPlanningGroupBox->setTitle(QCoreApplication::translate("MainWindow", "\350\267\257\345\276\204\350\247\204\345\210\222", nullptr));
        calculatePathButton->setText(QCoreApplication::translate("MainWindow", "\350\256\241\347\256\227\350\267\257\345\276\204", nullptr));
        navigateToGoalButton->setText(QCoreApplication::translate("MainWindow", "\345\257\274\350\210\252\345\210\260\347\233\256\346\240\207", nullptr));
        cancelGoalButton->setText(QCoreApplication::translate("MainWindow", "\345\217\226\346\266\210\347\233\256\346\240\207", nullptr));
        pathStatus->setText(QCoreApplication::translate("MainWindow", "\350\267\257\345\276\204\347\212\266\346\200\201: \346\227\240", nullptr));
        mapGroupBox->setTitle(QCoreApplication::translate("MainWindow", "\345\234\260\345\233\276\347\256\241\347\220\206", nullptr));
        saveMapButton->setText(QCoreApplication::translate("MainWindow", "\344\277\235\345\255\230\345\234\260\345\233\276", nullptr));
        loadMapButton->setText(QCoreApplication::translate("MainWindow", "\345\212\240\350\275\275\345\234\260\345\233\276", nullptr));
        resetMapButton->setText(QCoreApplication::translate("MainWindow", "\351\207\215\347\275\256\345\234\260\345\233\276", nullptr));
        mapInfo->setText(QCoreApplication::translate("MainWindow", "\345\234\260\345\233\276\344\277\241\346\201\257: \346\234\252\345\212\240\350\275\275", nullptr));
        slamSettingsGroupBox->setTitle(QCoreApplication::translate("MainWindow", "SLAM \350\256\276\347\275\256", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "\344\274\230\345\214\226\350\277\255\344\273\243\346\254\241\346\225\260", nullptr));
        cbCovariances->setText(QCoreApplication::translate("MainWindow", "\345\215\217\346\226\271\345\267\256", nullptr));
        optimizationMethodGroup->setTitle(QCoreApplication::translate("MainWindow", "\344\274\230\345\214\226\346\226\271\346\263\225", nullptr));
        rbGauss->setText(QCoreApplication::translate("MainWindow", "\351\253\230\346\226\257-\347\211\233\351\241\277", nullptr));
        rbLevenberg->setText(QCoreApplication::translate("MainWindow", "Levenberg-Marquardt", nullptr));
        btnOptimize->setText(QCoreApplication::translate("MainWindow", "\344\274\230\345\214\226\345\234\260\345\233\276", nullptr));
        menuFile->setTitle(QCoreApplication::translate("MainWindow", "\346\226\207\344\273\266", nullptr));
        menuView->setTitle(QCoreApplication::translate("MainWindow", "\350\247\206\345\233\276", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAIN_WINDOW_H
