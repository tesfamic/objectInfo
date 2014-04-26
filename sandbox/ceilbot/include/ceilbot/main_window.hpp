/**
 * @file /include/ceilbot/main_window.hpp
 *
 * @brief Qt based gui for ceilbot.
 *
 * @date November 2010
 **/
#ifndef ceilbot_MAIN_WINDOW_H
#define ceilbot_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QSlider>
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace ceilbot {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();

    void ReadSettings(); // Load up qt program settings at startup
    void WriteSettings(); // Save qt program settings when closing

    void closeEvent(QCloseEvent *event); // Overloaded function
    void showNoMasterMessage();

public slots:
    /******************************************
    ** Auto-connections (connectSlotsByName())
    *******************************************/
    void on_actionAbout_triggered();
    void on_start_button_clicked();
    void on_quit_button_clicked();
   // void on_checkbox_use_environment_stateChanged(int state);

    void on_sliderHmin_valueChanged();
    void on_sliderHmax_valueChanged();
    void on_sliderSmin_valueChanged();
    void on_sliderSmax_valueChanged();
    void on_sliderVmin_valueChanged();
    void on_sliderVmax_valueChanged();

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically


private:
    Ui::MainWindowDesign ui;
    QNode qnode;
};

}  // namespace ceilbot

#endif // ceilbot_MAIN_WINDOW_H
