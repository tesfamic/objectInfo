/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
#include "../include/ceilbot/main_window.hpp"
#include "../include/ceilbot/qnode.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) 
{

    /*********************
    ** Qt
    **********************/
    ros::init(argc,argv,"ceilbot");
    QApplication app(argc, argv);
    ceilbot::MainWindow w(argc,argv);
    ceilbot::QNode tma(argc,argv);
    w.show();
    ros::spin();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));


    int result = app.exec();

    return result;
}






