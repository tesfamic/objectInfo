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

#include <iostream>
#include <sstream>
#include <string>
#include <fstream>

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>

#define fname "/home/tesfamic/fuerte_workspace/sandbox/nodes/hsv.txt"
#define sizeHSV 6
#define PI 3.14159265
using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

int Hmin = 0;//37;
int Hmax = 255;//224;
int Smin = 0;//221;
int Smax = 255;//256;
int Vmin = 0;//94;
int Vmax = 255;//256;

const int MAX_NUM_OBJECTS = 100;//max number of objects to be detected in frame
const int MIN_OBJECT_AREA = 10 * 10;
const int MAX_OBJECT_AREA = 640*320 *2/3;
const string windowName = "Original Image";
const string windowName1 = "HSV Image";
const string windowName2 = "Thresholded Image";
const string windowName3 = "After Morphological Operations";
const string trackbarWindowName = "Trackbars";
static const char WINDOW[] = "Image window";
bool objectFound=false;
bool starting=true;
bool trackObjects = true;

class ObjectDetect
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber subRgbImage;
    image_transport::Subscriber subDepthImage;
    ros::Subscriber subPclImage;
    ros::Subscriber subTiltAngle;
    ros::Publisher pubTiltAngleAdjust;
    ros::Publisher pubDistanceofObject;
    ros::Subscriber subAnswer;

    int xCoordinate,yCoordinate;
    std_msgs::Float64 adjustedAngle,zVal;
    float currentTiltAngle,tiltAngleAdjust;
    bool ready;
    Mat depthImage;
    pcl::PointCloud<pcl::PointXYZ> input_;
    float zPcl,zNow,zPrev;
public:
    ObjectDetect() : it_(nh_)
    {
        subRgbImage   = it_.subscribe("/camera/rgb/image_color", 1, &ObjectDetect::rgbImageCb, this);
        //subDepthImage = it_.subscribe("/camera/depth_registered/image", 1, &ObjectDetect::depthImageCb, this);
        subPclImage   = nh_.subscribe("/camera/depth_registered/points", 1, &ObjectDetect::pcloudCb, this);
        subTiltAngle  = nh_.subscribe("/cur_tilt_angle",1,&ObjectDetect::tiltAngleCb,this);
        subAnswer    = nh_.subscribe("/reply",1, &ObjectDetect::replyCb,this);

        pubTiltAngleAdjust  = nh_.advertise<std_msgs::Float64>("/tilt_angle",1);
        pubDistanceofObject = nh_.advertise<std_msgs::Float64>("instruction",1);
        //cv::namedWindow(WINDOW);
    }
    ~ObjectDetect()
    {
        cv::destroyWindow(WINDOW);
    }
    void replyCb(const std_msgs::BoolConstPtr& replyMsg)
    {
        ready=replyMsg->data;
    }
    void tiltAngleCb(const std_msgs::Float64ConstPtr& tilt)
    {
        currentTiltAngle=tilt->data;
    }
    void pcloudCb (const sensor_msgs::PointCloud2ConstPtr& cloud)
    {
        pcl::fromROSMsg(*cloud, input_);   //converting from pcl to ros format
        if(xCoordinate>20 && yCoordinate>20 && xCoordinate<630 & yCoordinate<470)
        {
            zPcl=input_.points[abs(yCoordinate-1)*640+xCoordinate].z;
            //xPcl=input_.points[abs(yCoordinate-1)*640+xCoordinate].x;
            //yPcl=input_.points[abs(yCoordinate-1)*640+xCoordinate].y;
        }
        else
            zPcl=0.0;
        cout<<"\n PCL output, z: "<<zPcl;
        if(ready)
        {
            zVal.data=zPcl;
            zPrev=zPcl;
            pubDistanceofObject.publish(zVal);
            cout<<"\n     : "<<zVal.data;
            //cout<<endl;
        }
        else
        {
            zVal.data=zPrev;
        }
        //pubDistanceofObject.publish(zVal);
    }
    void rgbImageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        ifstream fileInput;
        ofstream fileOutput;
        string names[sizeHSV],lines[sizeHSV],hsvval[sizeHSV], _sname,str;
        stringstream nameString, valString;
        int hsv[sizeHSV],n=0;
        if(starting)
        {
            fileInput.open(fname,ios::in);
            if(!fileInput.is_open())
                cout<<"\n Error opening input file.";
            else
            {
                while(getline(fileInput,str,'\n'))
                {
                    lines[n]=str;
                    n++;
                }
            }
            fileInput.close();
            for(int i=0;i<n;i++)
            {
                int len=lines[i].length();
                int pos=lines[i].find(" ");
                if(!(pos<0 || pos>len))
                {
                    names[i]=lines[i].substr(0,pos);
                    hsvval[i]=lines[i].substr(pos+1,len-1);
                }
            }
            for(int i=0;i<n;i++)
            {
                nameString<<names[i];
                valString<<hsvval[i];
                nameString>>_sname;
                if(_sname.compare("hmin")==0)  valString>>hsv[i];
                if(_sname.compare("hmax")==0)  valString>>hsv[i];
                if(_sname.compare("smin")==0)  valString>>hsv[i];
                if(_sname.compare("smax")==0)  valString>>hsv[i];
                if(_sname.compare("vmin")==0)  valString>>hsv[i];
                if(_sname.compare("vmax")==0)  valString>>hsv[i];
                valString.clear();
            }
            Hmin=hsv[0];  Smin=hsv[2];  Vmin=hsv[4];
            Hmax=hsv[1];  Smax=hsv[3];  Vmax=hsv[5];
            starting=false;
        }

        bool useMorphOps = true;
        Mat HSV, threshold; //binary threshold image
        int x = 0, y = 0; //center point of the detected object

        createTrackbars(); //creates slider bars to filter the requierd object color

        hsv[0]=Hmin;  hsv[2]=Smin;  hsv[4]=Vmin;
        hsv[1]=Hmax;  hsv[3]=Smax;  hsv[5]=Vmax;

        fileOutput.open(fname,ios::out);
        if(!fileOutput.is_open())  cout<<"\n Error opening output file.";
        else
        {
            fileOutput<<"hmin"<<" "<<hsv[0]<<endl;
            fileOutput<<"hmax"<<" "<<hsv[1]<<endl;
            fileOutput<<"smin"<<" "<<hsv[2]<<endl;
            fileOutput<<"smax"<<" "<<hsv[3]<<endl;
            fileOutput<<"vmin"<<" "<<hsv[4]<<endl;
            fileOutput<<"vmax"<<" "<<hsv[5]<<endl;
        }
        fileOutput.close();

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

        imshow(windowName2, threshold);
        imshow(windowName, cameraFeed);
        //imshow(windowName1, HSV);
        waitKey(10); //delay for 10ms screen refresh.
    }
    void createTrackbars()
    {
        namedWindow(trackbarWindowName, 0);
        //create trackbars and insert them into window
        createTrackbar("Hmin", trackbarWindowName, &Hmin, 256, 0, 0);//on_trackbar);
        createTrackbar("Hmax", trackbarWindowName, &Hmax, 256, 0, 0);//on_trackbar);
        createTrackbar("Smin", trackbarWindowName, &Smin, 256, 0, 0);//on_trackbar);
        createTrackbar("Smax", trackbarWindowName, &Smax, 256, 0, 0);//on_trackbar);
        createTrackbar("Vmin", trackbarWindowName, &Vmin, 256, 0, 0);//on_trackbar);
        createTrackbar("Vmax", trackbarWindowName, &Vmax, 256, 0, 0);//on_trackbar);
    }
    void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed)
    {
        Mat temp;
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
                    pubTiltAngleAdjust.publish(adjustedAngle);

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
    void drawObject(int x, int y, Mat &frame)
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
};
int main(int argc, char** argv)
{
    ros::init(argc, argv, "ObjectDetector");
    ObjectDetect ic;
    ros::spin();
    return 0;
}

 /* void depthImageCb(const sensor_msgs::ImageConstPtr& depthMsg)
    {
        cv_bridge::CvImagePtr cvPtr;
        try
        {
            cvPtr = cv_bridge::toCvCopy(depthMsg,sensor_msgs::image_encodings::TYPE_32FC1);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        depthImage=cvPtr->image.clone();
                    //float zNow=0.0,zPrev=0.0,zt=0.0;
                    if(x>20 && y>20 && x<630 & y<470)
                    {
                        //zt=depthImage.at<float>(y,x);
                        //if(!isnan(zt))
                        //{
                        //    zNow= zt;
                         //   zPrev=zt;
                        //}
                       // else
                        //    zNow=zPrev;

                        //circle(depthImage, cv::Point(640,480), 5, CV_RGB(0,255,255));//320,240
                        //if (depthImage.rows > 60 && depthImage.cols > 60)
                        //    circle(depthImage, cv::Point(x,y), 5, CV_RGB(255,255,0));
                        //zval.data=zNow*cos(32*PI/180);

                        //pubDistanceofObject.publish(zval);
                    }
                    //imshow("depth", depthImage);//cvPtr->image);
                   // waitKey(3);
                    //cout<<"\n depth output:  "<<zNow;//<<"   @32 deg: "<<zval.data;
    } */

















