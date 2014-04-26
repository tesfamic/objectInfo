/**
 * @file /include/ceilbot/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ceilbot_QNODE_HPP_
#define ceilbot_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include <fstream>
#include <sstream>
#include <string>
#include <QThread>
#include <QStringListModel>

#define fname "/home/spacemaster/fuerte_workspace/sandbox/ceilbot/hsv.txt"
#define sizeHSV 6
#define PI 3.14159265
using namespace std;
using namespace cv;

//static const char WINDOW[] = "Image window";


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ceilbot {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT

private:
    int init_argc;
    char** init_argv;

    QStringListModel logging_model;

    ros::NodeHandle n;
    image_transport::ImageTransport it_;

    //ros::Publisher chatter_publisher;
    //image_transport::Subscriber subRgbImage;

    //ros::Subscriber subPclImage;
    ros::Subscriber subTiltAngle;
    //ros::Subscriber subAnswer;

    //ros::Publisher pubTiltAngleAdjust;
    //ros::Publisher pubDistanceofObject;


    //std_msgs::Float64 adjustedAngle;
    //std_msgs::Float32 zVal;
    //int xCoordinate,yCoordinate;
    float tiltAngleAdjust;
    //bool ready;
    //pcl::PointCloud<pcl::PointXYZ> input_;
    //float zPcl,zNow,zPrev;


public:
    QNode(int argc, char** argv );
    virtual ~QNode();
    bool init();
    //bool init(const std::string &master_url, const std::string &host_url);
    void run();
    ////////////////////////////////////////
    //void replyCb(const std_msgs::BoolConstPtr& replyMsg);
    void tiltAngleCb(const std_msgs::Float64ConstPtr& tilt);
    //void pcloudCb (const sensor_msgs::PointCloud2ConstPtr& cloud);
    //void rgbImageCb(const sensor_msgs::ImageConstPtr& msg);

    void createTrackbars();

    void drawObject(int x, int y, cv::Mat &frame);
    void trackFilteredObject(int &x, int &y, cv::Mat threshold, cv::Mat &cameraFeed);
    ///////////////////////////////////////
    int Hmin,Hmax,Smin,Smax,Vmin,Vmax;
    //Mat rgbImage;
    //Mat finalImage;
   // QPixmap myimg;
    bool begin;
    bool rdy;
    float currentTiltAngle;
    /*********************
    ** Logging
    **********************/
    enum LogLevel { Debug, Info, Warn, Error, Fatal };

    QStringListModel* loggingModel() { return &logging_model; }
    void log( const LogLevel &level, const std::string &msg);

signals:
    void loggingUpdated();
    void rosShutdown();


};

}  // namespace ceilbot

#endif /* ceilbot_QNODE_HPP_ */
