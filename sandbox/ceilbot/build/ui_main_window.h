/********************************************************************************
** Form generated from reading UI file 'main_window.ui'
**
** Created: Sat Apr 26 18:39:51 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAIN_WINDOW_H
#define UI_MAIN_WINDOW_H

#include <QtCore/QLocale>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLCDNumber>
#include <QtGui/QLabel>
#include <QtGui/QListView>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QScrollArea>
#include <QtGui/QSlider>
#include <QtGui/QStatusBar>
#include <QtGui/QTabWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindowDesign
{
public:
    QAction *action_Quit;
    QAction *action_Preferences;
    QAction *actionAbout;
    QAction *actionAbout_Qt;
    QWidget *centralwidget;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QGroupBox *groupBox_2;
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout_2;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_6;
    QLabel *label_7;
    QLabel *label_8;
    QWidget *layoutWidget1;
    QVBoxLayout *verticalLayout;
    QSlider *sliderHmin;
    QSlider *sliderHmax;
    QSlider *sliderSmin;
    QSlider *sliderSmax;
    QSlider *sliderVmin;
    QSlider *sliderVmax;
    QWidget *layoutWidget2;
    QVBoxLayout *verticalLayout_3;
    QLabel *labelHminVal;
    QLabel *labelHmaxVal;
    QLabel *labelSminVal;
    QLabel *labelSmaxVal;
    QLabel *labelVminVal;
    QLabel *labelVmaxVal;
    QTabWidget *tab_manager;
    QWidget *tab_status;
    QPushButton *quit_button;
    QPushButton *start_button;
    QListView *view_logging;
    QLCDNumber *lcdNumberCurrentAngle;
    QMenuBar *menubar;
    QMenu *menu_File;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindowDesign)
    {
        if (MainWindowDesign->objectName().isEmpty())
            MainWindowDesign->setObjectName(QString::fromUtf8("MainWindowDesign"));
        MainWindowDesign->resize(732, 500);
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MainWindowDesign->sizePolicy().hasHeightForWidth());
        MainWindowDesign->setSizePolicy(sizePolicy);
        MainWindowDesign->setMinimumSize(QSize(73, 82));
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/images/icon.png"), QSize(), QIcon::Normal, QIcon::Off);
        MainWindowDesign->setWindowIcon(icon);
        MainWindowDesign->setWindowOpacity(8);
        MainWindowDesign->setLocale(QLocale(QLocale::English, QLocale::Australia));
        MainWindowDesign->setDockNestingEnabled(true);
        action_Quit = new QAction(MainWindowDesign);
        action_Quit->setObjectName(QString::fromUtf8("action_Quit"));
        action_Quit->setShortcutContext(Qt::ApplicationShortcut);
        action_Preferences = new QAction(MainWindowDesign);
        action_Preferences->setObjectName(QString::fromUtf8("action_Preferences"));
        actionAbout = new QAction(MainWindowDesign);
        actionAbout->setObjectName(QString::fromUtf8("actionAbout"));
        actionAbout_Qt = new QAction(MainWindowDesign);
        actionAbout_Qt->setObjectName(QString::fromUtf8("actionAbout_Qt"));
        centralwidget = new QWidget(MainWindowDesign);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        scrollArea = new QScrollArea(centralwidget);
        scrollArea->setObjectName(QString::fromUtf8("scrollArea"));
        scrollArea->setGeometry(QRect(-20, 0, 721, 451));
        QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(scrollArea->sizePolicy().hasHeightForWidth());
        scrollArea->setSizePolicy(sizePolicy1);
        scrollArea->setMinimumSize(QSize(120, 120));
        scrollArea->setWidgetResizable(true);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QString::fromUtf8("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 719, 449));
        groupBox_2 = new QGroupBox(scrollAreaWidgetContents);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setGeometry(QRect(360, 10, 361, 211));
        layoutWidget = new QWidget(groupBox_2);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(10, 40, 38, 161));
        verticalLayout_2 = new QVBoxLayout(layoutWidget);
        verticalLayout_2->setSpacing(12);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        label = new QLabel(layoutWidget);
        label->setObjectName(QString::fromUtf8("label"));

        verticalLayout_2->addWidget(label);

        label_2 = new QLabel(layoutWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        verticalLayout_2->addWidget(label_2);

        label_3 = new QLabel(layoutWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        verticalLayout_2->addWidget(label_3);

        label_6 = new QLabel(layoutWidget);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        verticalLayout_2->addWidget(label_6);

        label_7 = new QLabel(layoutWidget);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        verticalLayout_2->addWidget(label_7);

        label_8 = new QLabel(layoutWidget);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        verticalLayout_2->addWidget(label_8);

        layoutWidget1 = new QWidget(groupBox_2);
        layoutWidget1->setObjectName(QString::fromUtf8("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(70, 41, 241, 157));
        verticalLayout = new QVBoxLayout(layoutWidget1);
        verticalLayout->setSpacing(13);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setSizeConstraint(QLayout::SetDefaultConstraint);
        verticalLayout->setContentsMargins(3, 0, 3, 0);
        sliderHmin = new QSlider(layoutWidget1);
        sliderHmin->setObjectName(QString::fromUtf8("sliderHmin"));
        sliderHmin->setMaximum(256);
        sliderHmin->setOrientation(Qt::Horizontal);

        verticalLayout->addWidget(sliderHmin);

        sliderHmax = new QSlider(layoutWidget1);
        sliderHmax->setObjectName(QString::fromUtf8("sliderHmax"));
        sliderHmax->setMaximum(256);
        sliderHmax->setOrientation(Qt::Horizontal);

        verticalLayout->addWidget(sliderHmax);

        sliderSmin = new QSlider(layoutWidget1);
        sliderSmin->setObjectName(QString::fromUtf8("sliderSmin"));
        sliderSmin->setMaximum(256);
        sliderSmin->setOrientation(Qt::Horizontal);

        verticalLayout->addWidget(sliderSmin);

        sliderSmax = new QSlider(layoutWidget1);
        sliderSmax->setObjectName(QString::fromUtf8("sliderSmax"));
        sliderSmax->setMaximum(256);
        sliderSmax->setOrientation(Qt::Horizontal);

        verticalLayout->addWidget(sliderSmax);

        sliderVmin = new QSlider(layoutWidget1);
        sliderVmin->setObjectName(QString::fromUtf8("sliderVmin"));
        sliderVmin->setMaximum(256);
        sliderVmin->setOrientation(Qt::Horizontal);

        verticalLayout->addWidget(sliderVmin);

        sliderVmax = new QSlider(layoutWidget1);
        sliderVmax->setObjectName(QString::fromUtf8("sliderVmax"));
        sliderVmax->setMaximum(256);
        sliderVmax->setOrientation(Qt::Horizontal);

        verticalLayout->addWidget(sliderVmax);

        layoutWidget2 = new QWidget(groupBox_2);
        layoutWidget2->setObjectName(QString::fromUtf8("layoutWidget2"));
        layoutWidget2->setGeometry(QRect(310, 40, 41, 161));
        verticalLayout_3 = new QVBoxLayout(layoutWidget2);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalLayout_3->setContentsMargins(0, 0, 0, 0);
        labelHminVal = new QLabel(layoutWidget2);
        labelHminVal->setObjectName(QString::fromUtf8("labelHminVal"));

        verticalLayout_3->addWidget(labelHminVal);

        labelHmaxVal = new QLabel(layoutWidget2);
        labelHmaxVal->setObjectName(QString::fromUtf8("labelHmaxVal"));

        verticalLayout_3->addWidget(labelHmaxVal);

        labelSminVal = new QLabel(layoutWidget2);
        labelSminVal->setObjectName(QString::fromUtf8("labelSminVal"));

        verticalLayout_3->addWidget(labelSminVal);

        labelSmaxVal = new QLabel(layoutWidget2);
        labelSmaxVal->setObjectName(QString::fromUtf8("labelSmaxVal"));

        verticalLayout_3->addWidget(labelSmaxVal);

        labelVminVal = new QLabel(layoutWidget2);
        labelVminVal->setObjectName(QString::fromUtf8("labelVminVal"));

        verticalLayout_3->addWidget(labelVminVal);

        labelVmaxVal = new QLabel(layoutWidget2);
        labelVmaxVal->setObjectName(QString::fromUtf8("labelVmaxVal"));

        verticalLayout_3->addWidget(labelVmaxVal);

        tab_manager = new QTabWidget(scrollAreaWidgetContents);
        tab_manager->setObjectName(QString::fromUtf8("tab_manager"));
        tab_manager->setGeometry(QRect(30, 0, 331, 441));
        QSizePolicy sizePolicy2(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(tab_manager->sizePolicy().hasHeightForWidth());
        tab_manager->setSizePolicy(sizePolicy2);
        tab_manager->setMinimumSize(QSize(93, 0));
        tab_manager->setLayoutDirection(Qt::LeftToRight);
        tab_manager->setLocale(QLocale(QLocale::English, QLocale::Australia));
        tab_manager->setElideMode(Qt::ElideLeft);
        tab_manager->setMovable(true);
        tab_status = new QWidget();
        tab_status->setObjectName(QString::fromUtf8("tab_status"));
        quit_button = new QPushButton(tab_status);
        quit_button->setObjectName(QString::fromUtf8("quit_button"));
        quit_button->setGeometry(QRect(20, 380, 271, 23));
        QSizePolicy sizePolicy3(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(quit_button->sizePolicy().hasHeightForWidth());
        quit_button->setSizePolicy(sizePolicy3);
        start_button = new QPushButton(tab_status);
        start_button->setObjectName(QString::fromUtf8("start_button"));
        start_button->setEnabled(true);
        start_button->setGeometry(QRect(20, 350, 271, 23));
        QSizePolicy sizePolicy4(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
        sizePolicy4.setHorizontalStretch(0);
        sizePolicy4.setVerticalStretch(0);
        sizePolicy4.setHeightForWidth(start_button->sizePolicy().hasHeightForWidth());
        start_button->setSizePolicy(sizePolicy4);
        view_logging = new QListView(tab_status);
        view_logging->setObjectName(QString::fromUtf8("view_logging"));
        view_logging->setGeometry(QRect(10, 10, 301, 321));
        tab_manager->addTab(tab_status, QString());
        lcdNumberCurrentAngle = new QLCDNumber(scrollAreaWidgetContents);
        lcdNumberCurrentAngle->setObjectName(QString::fromUtf8("lcdNumberCurrentAngle"));
        lcdNumberCurrentAngle->setGeometry(QRect(510, 250, 141, 23));
        lcdNumberCurrentAngle->setLayoutDirection(Qt::RightToLeft);
        scrollArea->setWidget(scrollAreaWidgetContents);
        MainWindowDesign->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindowDesign);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 732, 20));
        menu_File = new QMenu(menubar);
        menu_File->setObjectName(QString::fromUtf8("menu_File"));
        MainWindowDesign->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindowDesign);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindowDesign->setStatusBar(statusbar);

        menubar->addAction(menu_File->menuAction());
        menu_File->addAction(action_Preferences);
        menu_File->addSeparator();
        menu_File->addAction(actionAbout);
        menu_File->addAction(actionAbout_Qt);
        menu_File->addSeparator();
        menu_File->addAction(action_Quit);

        retranslateUi(MainWindowDesign);
        QObject::connect(action_Quit, SIGNAL(triggered()), MainWindowDesign, SLOT(close()));
        QObject::connect(quit_button, SIGNAL(clicked()), MainWindowDesign, SLOT(close()));

        tab_manager->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindowDesign);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindowDesign)
    {
        MainWindowDesign->setWindowTitle(QApplication::translate("MainWindowDesign", "QRosApp", 0, QApplication::UnicodeUTF8));
        action_Quit->setText(QApplication::translate("MainWindowDesign", "&Quit", 0, QApplication::UnicodeUTF8));
        action_Quit->setShortcut(QApplication::translate("MainWindowDesign", "Ctrl+Q", 0, QApplication::UnicodeUTF8));
        action_Preferences->setText(QApplication::translate("MainWindowDesign", "&Preferences", 0, QApplication::UnicodeUTF8));
        actionAbout->setText(QApplication::translate("MainWindowDesign", "&About", 0, QApplication::UnicodeUTF8));
        actionAbout_Qt->setText(QApplication::translate("MainWindowDesign", "About &Qt", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("MainWindowDesign", "             HSV -(Hue, Saturation, Value) Adjustment", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MainWindowDesign", "Hmin", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindowDesign", "Hmax", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("MainWindowDesign", "Smin", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("MainWindowDesign", "Smax", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("MainWindowDesign", "Vmin", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("MainWindowDesign", "Vmax", 0, QApplication::UnicodeUTF8));
        labelHminVal->setText(QApplication::translate("MainWindowDesign", "0", 0, QApplication::UnicodeUTF8));
        labelHmaxVal->setText(QApplication::translate("MainWindowDesign", "256", 0, QApplication::UnicodeUTF8));
        labelSminVal->setText(QApplication::translate("MainWindowDesign", "0", 0, QApplication::UnicodeUTF8));
        labelSmaxVal->setText(QApplication::translate("MainWindowDesign", "256", 0, QApplication::UnicodeUTF8));
        labelVminVal->setText(QApplication::translate("MainWindowDesign", "0", 0, QApplication::UnicodeUTF8));
        labelVmaxVal->setText(QApplication::translate("MainWindowDesign", "256", 0, QApplication::UnicodeUTF8));
        quit_button->setText(QApplication::translate("MainWindowDesign", "Quit", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        start_button->setToolTip(QApplication::translate("MainWindowDesign", "Set the target to the current joint trajectory state.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_STATUSTIP
        start_button->setStatusTip(QApplication::translate("MainWindowDesign", "Clear all waypoints and set the target to the current joint trajectory state.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        start_button->setText(QApplication::translate("MainWindowDesign", "Start", 0, QApplication::UnicodeUTF8));
        tab_manager->setTabText(tab_manager->indexOf(tab_status), QApplication::translate("MainWindowDesign", "Ros Communications", 0, QApplication::UnicodeUTF8));
        menu_File->setTitle(QApplication::translate("MainWindowDesign", "&App", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindowDesign: public Ui_MainWindowDesign {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAIN_WINDOW_H
