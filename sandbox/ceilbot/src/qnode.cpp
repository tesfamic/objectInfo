/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/ceilbot/qnode.hpp"
#include "../include/ceilbot/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace ceilbot
{

/*****************************************************************************
** Implementation
*****************************************************************************/
QNode::QNode(int argc, char** argv ) : init_argc(argc),	init_argv(argv),it_(n)
{
    subTiltAngle = n.subscribe("/cur_tilt_angle",1,&QNode::tiltAngleCb,this);
}
QNode::~QNode()
{
    if(ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}
bool QNode::init()
{
    if ( ! ros::master::check() )
    {
        return false;
    }
    else
    {
        ros::start(); // explicitly needed since our nodehandle is going out of scope.
        // Add your ros communications here.
        //rdy=true;
        start();
        return true;
    }
}
void QNode::run()
{
    // ros::Rate loop_rate(10);
    //int count = 0;
    while ( ros::ok() )
    {
        // if(begin)  rdy=true;
        // else       rdy=false;
    }

    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    emit rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}
void QNode::tiltAngleCb(const std_msgs::Float64ConstPtr& tilt)
{
    currentTiltAngle=tilt->data;
}

void QNode::log( const LogLevel &level, const std::string &msg)
{
    logging_model.insertRows(logging_model.rowCount(),1);
    std::stringstream logging_model_msg;
    switch ( level ) {
    case(Debug) : {
        ROS_DEBUG_STREAM(msg);
        logging_model_msg << "[DEBUG] "<<msg;//[" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Info) : {
        ROS_INFO_STREAM(msg);
        logging_model_msg << "[INFO] "<<msg;//[" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Warn) : {
        ROS_WARN_STREAM(msg);
        logging_model_msg << "[INFO] "<<msg;//[" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Error) : {
        ROS_ERROR_STREAM(msg);
        logging_model_msg << "[ERROR] "<<msg;//[" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Fatal) : {
        ROS_FATAL_STREAM(msg);
        logging_model_msg << "[FATAL] "<<msg;//[" << ros::Time::now() << "]: " << msg;
        break;
    }
    }
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    emit loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace ceilbot

/*bool QNode::init(const std::string &master_url, const std::string &host_url) {
    std::map<std::string,std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;
    ros::init(remappings,"ceilbot");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.
    chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    start();
    return true;
}

void QNode::replyCb(const std_msgs::BoolConstPtr& replyMsg)
{
    ready=replyMsg->data;
}
void QNode::rgbImageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    bool useMorphOps = true;
    Mat HSV, threshold; //binary threshold image
    int x = 0, y = 0; //center point of the detected object

    Mat cameraFeed = cv_ptr->image; //convert ros msg to opencv format Mat
    //Mat blurredPic;
    GaussianBlur(cameraFeed, cameraFeed, cvSize(5,5), 0, 0);//, int borderType=BORDER_DEFAULT )
    cvtColor(cameraFeed, HSV, COLOR_BGR2HSV); //convert RGB to HSV
    // filter HSV image between min & max values and store filtered image to threshold matrix
    inRange(HSV, Scalar(Hmin, Smin, Vmin), Scalar(Hmax, Smax, Vmax), threshold);
    //perform morphological operations on thresholded image to eliminate noise
    GaussianBlur(threshold, threshold, cvSize(3,3), 0, 0);
    if (useMorphOps)  // morphOps(threshold);
    {
        //create structure element for morphological operation
        Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
        //dilate with larger element so make sure object is well visible
        Mat dilateElement = getStructuringElement(MORPH_RECT, Size(5, 5));
        erode(threshold, threshold, erodeElement);
        dilate(threshold, threshold, dilateElement);
        //dilate(thresh, thresh, dilateElement);
    }
    if (trackObjects)
        trackFilteredObject(x, y, threshold, cameraFeed);

    finalImage=cameraFeed;
    imshow(windowName2, threshold);
    imshow(windowName, cameraFeed);
    //QPixmap myimg=QPixmap::fromImage(QImage((unsigned char*) cameraFeed.data, cameraFeed.cols, cameraFeed.rows, QImage::Format_RGB888));
    //imshow(windowName1, HSV);
    waitKey(3); //delay for 10ms screen refresh.
}
void QNode::trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed)
{
    cv::Mat temp;
    threshold.copyTo(temp);
    //these two vectors needed for output of findContours
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    Canny(temp, temp, 100, 127*2, 3 );
    //find contours of filtered image using openCV findContours function
    findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    //use moments method to find our filtered object
    if (hierarchy.size() > 0)
    {
        int numObjects = hierarchy.size();
        // cout<<"\n number of objects: "<<numObjects;
        double _area[numObjects];
        double _momX[numObjects];
        double _momY[numObjects];
        double maxArea=0.0;
        int indV=0;
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if (numObjects<MAX_NUM_OBJECTS)
        {
            for (int index = 0; index >= 0; index = hierarchy[index][0])
            {
                Moments moment = moments((cv::Mat)contours[index]);
                _area[index] = moment.m00;
                _momX[index] = moment.m10;
                _momY[index] = moment.m01;
            }
            for(int j=0;j<numObjects;j++)
            {
                if(_area[j]>maxArea)
                {
                    maxArea=_area[j]; indV=j;
                }
            }
            //cout<<"\n Max area: "<<maxArea<<"  index: "<<indV;
            if (maxArea>2*MIN_OBJECT_AREA && maxArea<MAX_OBJECT_AREA)// && area>refArea)
            {
                x = _momX[indV]/_area[indV]; // x = moment.m10 / _area[0];
                y = _momY[indV]/_area[indV]; // y = moment.m01 / _area[0];

                if(y<50)        tiltAngleAdjust=28;
                else if(y>460)   tiltAngleAdjust=-28;
                else             tiltAngleAdjust=0;
                adjustedAngle.data=tiltAngleAdjust;
                // pubTiltAngleAdjust.publish(adjustedAngle);

                xCoordinate=x;
                yCoordinate=y;
                objectFound=true;
                putText(cameraFeed, "Tracking Object", Point(0, 30), 2, 1, Scalar(0, 255, 0), 2);

                drawObject(x, y, cameraFeed);  //draw object location on screen
            }
            else
            {
                objectFound=false;   //xCoordinate=0; //yCoordinate=0;
            }
        }
        else
        {
            objectFound=false;  //xCoordinate=0; //yCoordinate=0;
            putText(cameraFeed,"Too much noise. Adjust filter", Point(0, 50), 2, 1, Scalar(0, 0, 255), 2);
        }
    }
}
void QNode::drawObject(int x, int y, Mat &frame)
{
    //draw crosshairs on tracked object
    rectangle(frame,Point(x-16,y+16),Point(x+16,y-16), Scalar(255,0,0),1);
    line(frame, Point(x-18,y),Point(x+18,y), Scalar(0,255,0),1);
    line(frame, Point(x,y-18),Point(x,y+18), Scalar(0,255,0),1);

    stringstream sx,sy; //converting x,y values to string
    sx<<x;
    sy<<y;
    putText(frame, sx.str() + "," + sy.str(), Point(x+2, y + 30), 1, 1, Scalar(0, 255, 0), 2);
}
void QNode::pcloudCb (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    pcl::fromROSMsg(*cloud, input_);   //converting from pcl to ros format
    if(xCoordinate>20 && yCoordinate>20 && xCoordinate<630 && yCoordinate<470)
    {
        zPcl=input_.points[abs(yCoordinate-1)*640+xCoordinate].z;
        //xPcl=input_.points[abs(yCoordinate-1)*640+xCoordinate].x;
        //yPcl=input_.points[abs(yCoordinate-1)*640+xCoordinate].y;
    }
    else
        zPcl=0.0;
    //std::cout<<"\n PCL output, z: "<<zPcl;
    if(ready)
    {
        zVal.data=zPcl;
        zPrev=zPcl;
        //pubDistanceofObject.publish(zVal);
        //std::cout<<"\n Dist    : "<<zVal.data;
        //log(Info,std::string("Distance: ")+ QString::number(zVal.data));
    }
    else
    {
        zVal.data=zPrev;
    }
    if(rdy)
    {
        std_msgs::String dd;
        stringstream ss;
        ss<<zPcl;
        dd.data=ss.str();
        log(Info,std::string("PCL data :  ")+ dd.data);
        cout<<zPcl;
    }
}


*/
