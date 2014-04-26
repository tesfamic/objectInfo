#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
 
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
 
#include <iostream>
//#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
//custom messages
#include <objectLocation0/coordinateContent.h>
#include <objectLocation0/coordinateArray.h>
#include <objectLocation0/objectPosArray.h>
#include <objectLocation0/objectPos.h>
using namespace std;
using namespace cv;
 
#define PI 3.14159265
static const std::string OPENCV_WINDOW = "Image window";
 
class CoordinateCalc
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  //image_transport::Publisher image_pub_;
  ros::Subscriber objectcoord_sub;
  ros::Subscriber pcl_sub;
  ros::Publisher coordinate_pub;
  ros::Publisher dist_pub;
  ros::Publisher reply_pub;
  ros::Subscriber subDetected;
  int _height,_width,_widtht,_heightt;
   
  //int widthAr[4],heightAr[4];
  std_msgs::Float32 zval;
  std_msgs::Bool reply;
  //std_msgs::Bool
  bool detectionConf;
public:
  CoordinateCalc(): it_(nh_)
  {
     image_sub_ = it_.subscribe("/camera/depth_registered/image_rect", 1, &CoordinateCalc::imageCb, this);
     objectcoord_sub = nh_.subscribe("/objectCoordinatePixel", 1, &CoordinateCalc::ObjectCoordCb, this);
     subDetected=nh_.subscribe("/detected",1,&CoordinateCalc::objDetectedCb,this);
     // subscribe to input point cloud
     pcl_sub = nh_.subscribe ("/camera/depth_registered/points", 1, &CoordinateCalc::cloud_cb, this);    
     coordinate_pub = nh_.advertise<objectLocation0::coordinateArray> ("xyzCoordinate", 1);
     dist_pub = nh_.advertise<std_msgs::Float32>("instruction",1);
     reply_pub =nh_.advertise<std_msgs::Bool>("reply",1);
     //cv::namedWindow(OPENCV_WINDOW);
  }
  ~CoordinateCalc()
  {
     cv::destroyWindow(OPENCV_WINDOW);
  }
  void objDetectedCb(const std_msgs::BoolConstPtr& detected)
  {
     detectionConf=detected->data;
  }
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
     cv_bridge::CvImagePtr cv_ptr,cvPtr;
     try
     {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cvPtr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_32FC1);
     }
     catch (cv_bridge::Exception& e)
     {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
     }   
     Mat depthImage=cvPtr->image.clone();
     float zNow,zPrev; 
     float zt;
     if(_width>20 && _height>20 && _width<630 & _height<470)
     { 
       if(detectionConf)
       {
          zt=depthImage.at<float>(_height,_width);
          if(!isnan(zt))
          {
            zNow= zt;
            zPrev=zNow;
          }
          else
            zNow=zPrev;          
       }
       else
          zNow=zPrev;
     }
      
     circle(cv_ptr->image, cv::Point(320,240), 5, CV_RGB(0,255,255));
     if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
       circle(cv_ptr->image, cv::Point(_width, _height), 5, CV_RGB(255,0,0));     
     // Update GUI Window
     imshow(OPENCV_WINDOW, cv_ptr->image);
     waitKey(3); 
     zval.data=zNow*cos(32*PI/180);
       cout<<"\n depth:"<<zNow<<" @32 deg: "<<zval.data;
     dist_pub.publish(zval);  
  }
  void ObjectCoordCb (const objectLocation0::objectPosArray::ConstPtr& dim)
  {
      for(int j=0;j<dim->position.size();++j)
      {
         objectLocation0::objectPos xyPositions=dim->position[j];
         _widtht = xyPositions.width+20; //to adjust the 2.5cm diff. between camera cent
         _heightt = xyPositions.height;
        // int delta=3;
         int prevWid,prevHei;
         if(_widtht>20 && _heightt>20 && _widtht<630 & _heightt<470)
         {          
             _width=_widtht;
             _height=_heightt;
              prevWid=_width;
              prevHei=_height;
              reply.data= true;          
         }
         else
         {
            _width=25; _height=25;
            reply.data = false;
         }
      }
      reply_pub.publish(reply);
  }
  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
  {
     sensor_msgs::PointCloud2 cloud_filtered;
 
     pcl::PointCloud<pcl::PointXYZ> input_;
     pcl::fromROSMsg(*cloud, input_);   //converting from pcl to ros format
   
     objectLocation0::coordinateContent pubCoor; //declaring x,y,and z
     objectLocation0::coordinateArray xyz;  //declaring array containing x,y,z
 
     int start=_height*640+_width-2;  //calculating the pixel point
     //int finish=start+4;
     //for(int i=start;i<=finish;i++) 
    //{   //if(input_.points[i].x>=0 || input_.points[i].x<=0)
        // cout<<"\npt: "<<i<<" "<<input_.points[i]<<endl;
     //}
     pubCoor.x=input_.points[start+2].x;
     pubCoor.y=input_.points[start+2].y;
     pubCoor.z=input_.points[start+2].z;
      
     xyz.coordinates.push_back(pubCoor);  //put the x,y,z values in an array  
     //cout<<"\n size of xyz: "<<xyz.coordinates.size();
     //for(int i=0;i<xyz.coordinates.size();++i){
      // objectLocation::coordinateContent extract= xyz.coordinates[i];
       //cout<<"\n x val: "<<extract.x<<" y val: "<<extract.y<<" z val: "<<extract.z; }  
      
     coordinate_pub.publish (xyz);  // Publish the data
  } 
};
 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "CoordinateCalc");
  CoordinateCalc ic;  //class ic
  ros::spin();
  return 0;
}























