#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"
#include <iostream>
#include <fstream>
//#define fname "/home/ceilbot/fuerte_workspace/sandbox/motorcontrol/prevLocation.txt"
#define fname "/home/ceilbot/fuerte_workspace/sandbox/motorcontrol/log.txt"
using namespace std;
//bool starting=true;
//double distanceValue=0.0;
double totalDist=0.0;
double xPrev=0.0;
//long int countValue=0;
string mode="calibrate";
class Node
{
  protected:
       ros::NodeHandle nh_;
       ros::Subscriber subInstruct;
       ros::Subscriber subDistance;
       ros::Subscriber subCounter;
       ros::Subscriber subModeInstruction;
       ros::Publisher pubCmd;
       ros::Publisher pubMode;
       ros::Subscriber subOthercmd;
       ros::Subscriber subSwitch;
       ros::Subscriber subSwitchState;
       std_msgs::Int16 command;
       long int msg;
       long int cmdNow,cmdPrev;
       float valNow,valPrev;
       float xNow;
       float delta,deltaMax;
       float pwm,pwm0,error;
       std_msgs::String cmdMode;
       string sw;
       string on_or_off;
  public:
        Node(): nh_()//(ros::NodeHandle& nh): nh_(nh)
        {
      pubCmd=nh_.advertise<std_msgs::Int16>("command",3);
      pubMode=nh_.advertise<std_msgs::String>("opMode",3);
      subModeInstruction=nh_.subscribe("modeInstruction",10,&Node::modeInstructionCb,this);
      subInstruct=nh_.subscribe("instruction",10,&Node::instructionCb,this);
      subDistance=nh_.subscribe("distance",10,&Node::distanceCb,this);
      subCounter=nh_.subscribe("counter",10,&Node::counterCb,this);
      //subOthercmd=nh_.subscribe("othercmd",10,&Node::othersCb,this);
      subSwitch=nh_.subscribe("switchVal",10,&Node::switchCb,this);
      subSwitchState=nh_.subscribe("switchState",10,&Node::switchStateCb,this);
    }
    ~Node()
    {}
    void counterCb(const std_msgs::Int32::ConstPtr& countVal)
    {
       msg=countVal->data; //cout<<"\ncount received:   "<<msg;
    }
    void instructionCb(const std_msgs::Float32::ConstPtr& instructVal)
    {
       valNow=instructVal->data;//cout<<"\ninstruction received: "<<valNow;
    }
    void distanceCb(const std_msgs::Float32::ConstPtr& distanceX)
    {
       xNow=distanceX->data;  //cout<<"\ndistance traveled:   "<<xNow;
       execute();
    }
    void switchCb(const std_msgs::String::ConstPtr& swVal)
    {
       sw=swVal->data;   //cout<<"\n Switch status: "<<sw;
    }
    void switchStateCb(const std_msgs::String::ConstPtr& switchSt)
    {
       on_or_off=switchSt->data;
    }
    void modeInstructionCb(const std_msgs::String::ConstPtr& modeInstruction)
    {
       mode=modeInstruction->data;
    }
    void execute()
    {

       ofstream writeCurrentData;// log file

      if(mode=="calibrate")
      {
         valNow=0.0;
         delta=0.0; error=0.0;
         deltaMax=0.0;
         pwm=0.0;
         pwm0=0.157;
         //cmdNow=0;
         xPrev=0.0;
         totalDist=0.0;
         cout<<"\n CALIBRATION MODE.";
         cout<<"\n       DISTANCE: "<<xNow;
         cout<<"\n          COUNT: "<<msg;
        // cout<<"\n    SWITCH STAT: "<<sw;
         cout<<endl;
         cmdMode.data="calibrate";
         pubMode.publish(cmdMode);

         writeCurrentData.open(fname,ios::out);
         if(!writeCurrentData.is_open())
          cout<<"\n Failed to open output (log) file.";
          writeCurrentData<<"";
         writeCurrentData.close();
      }
      if(mode=="normal")
      {
        //cmdNow=0;
        cmdMode.data="normal";
        pubMode.publish(cmdMode);
        //////////////////////////////////
        writeCurrentData.open(fname,ios::app);
        if(!writeCurrentData.is_open())
          cout<<"\n Failed to open output (log) file.";

        //////
       // valNow=valNow+xNow;
        if(valNow<0.0)            cmdNow=0;
        else if(abs(valNow)>2.99)  valNow=valPrev;
        else
        {
            //valNow=valNow+xNow;
            delta=valNow-xNow;
            error=delta;
            if(abs(delta)>abs(deltaMax))
               deltaMax=abs(delta);
            //cout<<"\n Which part of code: "<<sw;
            cout<<"\n instructed distan.: "<<valNow;
            cout<<"\n distance travelled: "<<xNow;
            cout<<"\n delta val:          "<<delta;
            cout<<"\n deltaMax:           "<<deltaMax;
            if(valNow>xNow && !(abs(delta)<0.003))
            {
              if(deltaMax>0.4)//if(abs(delta)>0.8)
              {
                if(abs(error)>(deltaMax-0.2))
                   pwm=int(255*(pwm0+5*(1-pwm0)*(xNow-valNow+deltaMax)));
                else if(abs(error)<0.2)
                   pwm=int(255*(pwm0-5*(1-pwm0)*(xNow-valNow)));
                else
                   pwm=254;
              }
              else           pwm=60;

              cmdNow=pwm;
              writeCurrentData<<"\nvalNow: "<<valNow<<" xNow: "<<xNow<<" delta: "<<abs(delta)<<"  COMMAND: "<<cmdNow;
            }
            else if(valNow<xNow && !(abs(delta)<0.003))
            {

              if(deltaMax>0.4)//if(abs(delta)>0.8)
              {
                if(abs(error)>(deltaMax-0.2))
                   pwm=int(-255*(pwm0-5*(1-pwm0)*(xNow-valNow-deltaMax)));
                else if(abs(error)<0.2)
                   pwm=int(-255*(pwm0+5*(1-pwm0)*(xNow-valNow)));
                else
                   pwm=-254;
              }
              else             pwm=-60;//        cmdNow=-60;

             cmdNow=pwm;
             writeCurrentData<<"\nvalNow: "<<valNow<<" xNow: "<<xNow<<" delta: "<<abs(delta)<<"  COMMAND: "<<cmdNow;

            }
            else
            {
               cmdNow=0;
               if(xNow!=xPrev)
               {
                  totalDist += abs(xNow-xPrev);
                  xPrev=xNow;
                  if(totalDist>60.0)
                     mode="calibrate";
                  deltaMax=abs(delta);
                  pwm=0.0;
               }
               cout<<"\n TOTAL DISTANCE TRAVELLED: "<<totalDist<<endl;
               writeCurrentData<<"\nvalNow: "<<valNow<<" xNow: "<<xNow<<" delta: "<<abs(delta)<<" COMMAND: "<<cmdNow;
            }
            valPrev=valNow;
            writeCurrentData.close();
        }
       }
       command.data=cmdNow;
       pubCmd.publish(command);
        }

}; //End of class

int main(int argc, char **argv)
{
  ros::init(argc, argv,"motorControl");
  //ros::NodeHandle nh;

  //Node *nodeN= new Node(nh);
  Node motorC;
  //nodeN->spin();
  ros::spin();
  return 0;
}
