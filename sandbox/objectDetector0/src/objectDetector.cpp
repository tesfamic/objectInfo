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
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
 
// custom messages
#include <objectDetector0/objectPos.h>
#include <objectDetector0/objectPosArray.h>
using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;
#define fname "/home/tesfamic/fuerte_workspace/sandbox/objectDetector0/hsv.txt"
#define sizeHSV 6
int Hmin = 0;
int Hmax = 255;
int Smin = 0;
int Smax = 256;
int Vmin = 0;
int Vmax = 256;

//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 100;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 10 * 10;
const int MAX_OBJECT_AREA = 640*320 *2/3;
//titles for each window
const string windowName = "Original Image";
const string windowName1 = "HSV Image";
const string windowName2 = "Thresholded Image";
const string windowName3 = "After Morphological Operations";
const string trackbarWindowName = "Trackbars";
static const char WINDOW[] = "Image window";
bool found=false;
bool starting=true;
//bool trackObjects = true;
class ObjectDetect
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
   
    ros::Publisher imagePix_pub;
    ros::Subscriber response_sub;
    ros::Subscriber subTiltAngle;
    ros::Publisher pubDetection;
    ros::Publisher pubTiltAngleAdjust;
    objectDetector0::objectPosArray PosCurrent,PosPrev;
    objectDetector0::objectPos xyPos;
     
    int xCoordinate;
    int yCoordinate;
    bool resp;
    std_msgs::Bool detection;
    std_msgs::Float64 adjustedAngle;
    float currentTiltAngle;
    float tiltAngleAdjust;
   public:
      ObjectDetect() : it_(nh_)
      {
         image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &ObjectDetect::imageCb, this);
         subTiltAngle=nh_.subscribe("/cur_tilt_angle",1,&ObjectDetect::tiltAngleCb,this);
         imagePix_pub= nh_.advertise<objectDetector0::objectPosArray>("objectCoordinatePixel",1);
         pubDetection= nh_.advertise<std_msgs::Bool>("detected",1);
         pubTiltAngleAdjust=nh_.advertise<std_msgs::Float64>("/tilt_angle",1);
         response_sub= nh_.subscribe("reply",1,&ObjectDetect::respCb,this);
         //cv::namedWindow(WINDOW);
      }
     ~ObjectDetect()
     {
        cv::destroyWindow(WINDOW);
     }
     void tiltAngleCb(const std_msgs::Float64ConstPtr& tilt)
     {
        currentTiltAngle=tilt->data;
        //cout<<"\ntilt angle: "<<tiltAngle;
     }
     void respCb(const std_msgs::BoolConstPtr& response)
     {
        resp=response->data;
     }
     void imageCb(const sensor_msgs::ImageConstPtr& msg)
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
         
      // file input output:  reades HSV values from file 
      ifstream fileInput;
      ofstream fileOutput;
      string names[sizeHSV];
      string lines[sizeHSV];
      string hsvval[sizeHSV];
      string _sname;
      stringstream nameString,valString;
      int hsv[sizeHSV];      
      string str;
      int n=0;
    if(starting) //do this only at the beginning
    {
        fileInput.open(fname,ios::in);
        if(!fileInput.is_open())  cout<<"\n Error opening input file.";
        else
        {
          while(getline(fileInput,str,'\n')) //read file line by line
          {
            lines[n]=str;   //store each line  
            n++;
          }
        }
        fileInput.close();
        for(int i=0;i<n;i++)
        {
          int len=lines[i].length();  //length of characters in the line
          int pos=lines[i].find(" ");  //find the location of the first white space
          if(!(pos<0 || pos>len))
          {
           names[i]=lines[i].substr(0,pos); //the first string is name
           hsvval[i]=lines[i].substr(pos+1,len-1); //the second string is value
          }
        }
        for(int i=0;i<n;i++)     // convert string to integer
        {
           nameString<<names[i];    //store string to string stream
           valString<<hsvval[i];
           nameString>>_sname;
           if(_sname.compare("hmin")==0) { valString>>hsv[i];}
           if(_sname.compare("hmax")==0) { valString>>hsv[i];}
           if(_sname.compare("smin")==0) { valString>>hsv[i];}
           if(_sname.compare("smax")==0) { valString>>hsv[i];}
           if(_sname.compare("vmin")==0) { valString>>hsv[i];}
           if(_sname.compare("vmax")==0) { valString>>hsv[i];}
           valString.clear();     
        } 
        Hmin=hsv[0]; Smin=hsv[2]; Vmin=hsv[4]; //load the hsv min max values
        Hmax=hsv[1]; Smax=hsv[3]; Vmax=hsv[5];
        starting=false;    //exit this loop after loading once
    }
                     
    bool useMorphOps = true;
    bool trackObjects = true;
    Mat HSV;
    Mat threshold; //binary threshold image
    int x = 0, y = 0; //center point of the detected object    
    createTrackbars(); //creates slider bars to filter the requierd object color  
    
    
      hsv[0]=Hmin; hsv[2]=Smin;  hsv[4]=Vmin; //store the changes of the slider
	hsv[1]=Hmax; hsv[3]=Smax;  hsv[5]=Vmax;
	
	fileOutput.open(fname,ios::out);  //open file for storage
	if(!fileOutput.is_open())
	  cout<<"\n Failed to open output file.";
	else
	{
	   fileOutput<<"hmin"<<" "<<hsv[0]<<endl;  //store name first and then value
	   fileOutput<<"hmax"<<" "<<hsv[1]<<endl;  //separated by single white space
	   fileOutput<<"smin"<<" "<<hsv[2]<<endl;
	   fileOutput<<"smax"<<" "<<hsv[3]<<endl;
	   fileOutput<<"vmin"<<" "<<hsv[4]<<endl;
	   fileOutput<<"vmax"<<" "<<hsv[5]<<endl;
	}
	fileOutput.close(); 
     
      
    Mat cameraFeed = cv_ptr->image; //convert ros msg to opencv format Mat
    Mat blurredPic;
     
    GaussianBlur(cameraFeed, cameraFeed, cvSize(5,5), 0, 0);//, int borderType=BORDER_DEFAULT )
    cvtColor(cameraFeed, HSV, COLOR_BGR2HSV); //convert RGB to HSV
     
    
     // filter HSV image between min & max values and store filtered image to threshold matrix
    inRange(HSV, Scalar(Hmin, Smin, Vmin), Scalar(Hmax, Smax, Vmax), threshold);
    //perform morphological operations on thresholded image to eliminate noise
    GaussianBlur(threshold, threshold, cvSize(3,3), 0, 0);
    if (useMorphOps)
       morphOps(threshold);
     
    if (trackObjects)
       trackFilteredObject(x, y, threshold, cameraFeed);
 
    imshow(windowName2, threshold);
    imshow(windowName, cameraFeed);
    //imshow(windowName1, HSV);    
    waitKey(10); //delay for 10ms screen refresh.
    //int delY=abs(240-yCoordinate);
    tiltAngleAdjust=currentTiltAngle;
     
    //if(yCoordinate<60)        tiltAngleAdjust=26.5;
     if(yCoordinate<120)  tiltAngleAdjust=18.5;
    //else if(yCoordinate<180)  tiltAngleAdjust=11.5;
    else if(yCoordinate<240)  tiltAngleAdjust=11.5;//3.0;
   // else if(yCoordinate<300)  tiltAngleAdjust=-3.0;
    else if(yCoordinate<360)  tiltAngleAdjust=-11.5;
    //else if(yCoordinate<420)  tiltAngleAdjust=-18.5;
    else                      tiltAngleAdjust=-18.5;//26.5;
         
    adjustedAngle.data=tiltAngleAdjust;
    pubTiltAngleAdjust.publish(adjustedAngle);
     
    xyPos.width=xCoordinate;   //converting from int to custom message
    xyPos.height=yCoordinate;
     
    PosCurrent.position.push_back(xyPos);  //store it into the new message called Pos
    int delta=5;
    int xprev,yprev;
    if(abs(xCoordinate-xprev)>delta || abs(yCoordinate-yprev)>delta) //!resp)
    {
       imagePix_pub.publish(PosCurrent);  //publish both x,y positions in an array
       
       xprev=xCoordinate;
       yprev=yCoordinate;
       PosPrev=PosCurrent;
    }
    else
    {
       imagePix_pub.publish(PosPrev);
    }
        pubDetection.publish(detection);
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
      
     void morphOps(Mat &thresh)
     {
       //create structure element for morphological operation
       Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
      //dilate with larger element so make sure object is well visible
       Mat dilateElement = getStructuringElement(MORPH_RECT, Size(3, 3));
 
       erode(thresh, thresh, erodeElement);
     
       dilate(thresh, thresh, dilateElement);
       dilate(thresh, thresh, dilateElement);
     }
     void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed)
     {
    Mat temp;
    threshold.copyTo(temp);
    //imshow("image input to tracker", temp);
    //these two vectors needed for output of findContours
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    Canny(temp, temp, 100, 127*2, 3 );
    //find contours of filtered image using openCV findContours function
    findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    //use moments method to find our filtered object   
    //bool objectFound = false;
    //imshow("contour", temp);
    double refArea=0;
    if (hierarchy.size() > 0)
    {
          int numObjects = hierarchy.size();
         // cout<<"\n number of objects: "<<numObjects;
          double _area[numObjects];
          double _momX[numObjects];
          double _momY[numObjects];
          double maxArea=0.0;//_area[0];
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
         //  _area[index]=moment.m00;
         for(int j=0;j<numObjects;j++)
         {
            if(_area[j]>maxArea)  
            {
               maxArea=_area[j]; indV=j;
            }  
          }
          //cout<<"\n Max area: "<<maxArea<<" index: "<<indV;
          if (maxArea>2*MIN_OBJECT_AREA && maxArea<MAX_OBJECT_AREA)// && area>refArea)
           {
		     x = _momX[indV]/_area[indV]; // x = moment.m10 / _area[0];
		     y = _momY[indV]/_area[indV]; // y = moment.m01 / _area[0];          
		      
		     //refArea=area;   
		     detection.data=true;
		     xCoordinate=x;
                 yCoordinate=y;
                
               putText(cameraFeed, "Tracking Object", Point(0, 30), 2, 1, Scalar(0, 255, 0), 2);
               //draw object location on screen
               drawObject(x, y, cameraFeed);
           }
           else
           { 
          		detection.data=false;   //xCoordinate=0; //yCoordinate=0; 
           } 
           
          }
          else
		{
		  detection.data=false;  //xCoordinate=0; //yCoordinate=0;
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






















