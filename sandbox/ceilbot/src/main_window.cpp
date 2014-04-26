/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <QPixmap>
#include <QScrollArea>
#include <QLCDNumber>
#include <iostream>
#include "../include/ceilbot/main_window.hpp"
#include "../include/ceilbot/qnode.hpp"
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ceilbot {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent): QMainWindow(parent), qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
    setWindowIcon(QIcon(":/images/icon.png"));
    ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).

    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    /*********************
    ** Logging
    **********************/
    ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    ui.lcdNumberCurrentAngle->display(qnode.currentTiltAngle);

            //Trackbar for HSV adjustment
    QObject::connect(ui.sliderHmin,SIGNAL(valueChanged(int)),this, SLOT(on_sliderHmin_valueChanged()));
    QObject::connect(ui.sliderHmax,SIGNAL(valueChanged(int)),this, SLOT(on_sliderHmax_valueChanged()));
    QObject::connect(ui.sliderSmin,SIGNAL(valueChanged(int)),this, SLOT(on_sliderSmin_valueChanged()));
    QObject::connect(ui.sliderSmax,SIGNAL(valueChanged(int)),this, SLOT(on_sliderSmax_valueChanged()));
    QObject::connect(ui.sliderVmin,SIGNAL(valueChanged(int)),this, SLOT(on_sliderVmin_valueChanged()));
    QObject::connect(ui.sliderVmax,SIGNAL(valueChanged(int)),this, SLOT(on_sliderVmax_valueChanged()));

    QObject::connect(ui.quit_button,SIGNAL(clicked()),this,SLOT(on_quit_button_clicked()));
    /*********************
    ** Auto Start
    **********************/
    // if ( ui.checkbox_remember_settings->isChecked() )
    //  {
    //      on_button_connect_clicked(true);
    //  }
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/
void MainWindow::showNoMasterMessage() {
    QMessageBox msgBox;
    msgBox.setText("Couldn't find the ros master.");
    msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_start_button_clicked()
{
    // if ( ui.checkbox_use_environment->isChecked() )
    {
        if ( !qnode.init() )
        {
            showNoMasterMessage();
        }
        else
        {
            //qnode.begin=true;
            ui.start_button->setEnabled(true);
        }
    }
}
void MainWindow::on_quit_button_clicked()
{
    ros::shutdown();
   // destroyWindow("Thresholded Image");
   // destroyWindow("Original Image");

}
/* else
    {
        if ( !qnode.init() )
        {
            showNoMasterMessage();
        }
        else
        {
            //ros::start();
            ui.button_connect->setEnabled(false);
            ui.line_edit_master->setReadOnly(true);
            ui.line_edit_host->setReadOnly(true);
            ui.line_edit_topic->setReadOnly(true);
        }
    }
}

void MainWindow::on_checkbox_use_environment_stateChanged(int state)
{
    bool enabled;
    if ( state == 0 )
    {
        enabled = true;
    }
    else
    {
        enabled = false;
    }
    ui.line_edit_master->setEnabled(enabled);
    ui.line_edit_host->setEnabled(enabled);
    //ui.line_edit_topic->setEnabled(enabled);
}
*/
////////////////
void MainWindow::on_sliderHmin_valueChanged()
{
    qnode.Hmin=ui.sliderHmin->value();
    ui.labelHminVal->setText(QString::number(qnode.Hmin));
}
void MainWindow::on_sliderHmax_valueChanged()
{
    qnode.Hmax=ui.sliderHmax->value();
    ui.labelHmaxVal->setText(QString::number(qnode.Hmax));
}
void MainWindow::on_sliderSmin_valueChanged()
{
    qnode.Smin=ui.sliderSmin->value();
    ui.labelSminVal->setText(QString::number(qnode.Smin));
}
void MainWindow::on_sliderSmax_valueChanged()
{
    qnode.Smax=ui.sliderSmax->value();
    ui.labelSmaxVal->setText(QString::number(qnode.Smax));
}
void MainWindow::on_sliderVmin_valueChanged()
{
    qnode.Vmin=ui.sliderVmin->value();
    ui.labelVminVal->setText(QString::number(qnode.Vmin));
}
void MainWindow::on_sliderVmax_valueChanged()
{
    qnode.Vmax=ui.sliderVmax->value();
    ui.labelVmaxVal->setText(QString::number(qnode.Vmax));
}
/////////////////////////////////

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView()
{
    ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered()
{
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}


/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "ceilbot");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    // QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    // QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    // ui.line_edit_master->setText(master_url);
    //ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    // bool remember = settings.value("remember_settings", false).toBool();
    // ui.checkbox_remember_settings->setChecked(remember);
    //  bool checked = settings.value("use_environment_variables", false).toBool();
    // ui.checkbox_use_environment->setChecked(checked);
    // if ( checked )
    // {
    //     ui.line_edit_master->setEnabled(false);
    //     ui.line_edit_host->setEnabled(false);
    //ui.line_edit_topic->setEnabled(false);
    // }
    qnode.Hmin= settings.value("Hmin").toInt();
    qnode.Hmax= settings.value("Hmax").toInt();
    qnode.Smin=settings.value("Smin").toInt();
    qnode.Smax=settings.value("Smax").toInt();
    qnode.Vmin=settings.value("Vmin").toInt();
    qnode.Vmax=settings.value("Vmax").toInt();

    ui.sliderHmin->setValue(qnode.Hmin);
    ui.sliderHmax->setValue(qnode.Hmax);
    ui.sliderSmin->setValue(qnode.Smin);
    ui.sliderSmax->setValue(qnode.Smax);
    ui.sliderVmin->setValue(qnode.Vmin);
    ui.sliderVmax->setValue(qnode.Vmax);
    ui.labelHminVal->setText(QString::number(qnode.Hmin));
    ui.labelHmaxVal->setText(QString::number(qnode.Hmax));
    ui.labelSminVal->setText(QString::number(qnode.Smin));
    ui.labelSmaxVal->setText(QString::number(qnode.Smax));
    ui.labelVminVal->setText(QString::number(qnode.Vmin));
    ui.labelVmaxVal->setText(QString::number(qnode.Vmax));


}

void MainWindow::WriteSettings()
{
    QSettings settings("Qt-Ros Package", "ceilbot");
    //settings.setValue("master_url",ui.line_edit_master->text());
    // settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    //settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    //settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));
    settings.setValue("Hmin",ui.sliderHmin->value());
    settings.setValue("Hmax",ui.sliderHmax->value());
    settings.setValue("Smin",ui.sliderSmin->value());
    settings.setValue("Smax",ui.sliderSmax->value());
    settings.setValue("Vmin",ui.sliderVmin->value());
    settings.setValue("vmax",ui.sliderVmax->value());

}

void MainWindow::closeEvent(QCloseEvent *event)
{
    WriteSettings();
    QMainWindow::closeEvent(event);
}

}  // namespace ceilbot
