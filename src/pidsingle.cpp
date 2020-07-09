#include "ros/ros.h"
#include "iostream"
#include "std_msgs/Float32.h"


using namespace std;

class PIDRos
{
    float W,wl,s_wl,mario;
    double r, l, kp, ki, kd, err_wl, err_wl_prev,  I_wl;
    double   U_wl, D_wl, des_wl;
    ros::NodeHandle node;
    ros::Subscriber des_vel_W, sens_vel_wl;
    ros::Publisher rot_vel_wl;
    ros::Time current_time, last_time;
public:
    PIDRos();
    void Run();
    void Wcallback(const std_msgs::Float32::ConstPtr& msg);
    void WlSenscallback(const std_msgs::Float32::ConstPtr& msg);
};

PIDRos::PIDRos(){


	//Advertise velocity control messages to arduino wr and wl (Values -1 to 1) 
    rot_vel_wl=node.advertise<std_msgs::Float32>("Vel_ctl/wl",10);
	//Subscribe to the set points of V and W of the robot and the Wr and Wl of the sensors from Arduino
    des_vel_W=node.subscribe("Vel_ctl/SP_W",10,&PIDRos::Wcallback,this);

    sens_vel_wl=node.subscribe("Vel_sens/wl",10,&PIDRos::WlSenscallback,this);

	//Initialize variables

    W=0;

    wl=0;
    s_wl=0;
    r=0.0508;
    l=0.365;

    err_wl=0;
    I_wl=0;
    err_wl_prev=0;
    kp=0.02;
    ki=0.3;
    kd=0;

    current_time = ros::Time::now();
    last_time = ros::Time::now();

}


void PIDRos::Wcallback(const std_msgs::Float32::ConstPtr& msg)
{
        W = msg->data;
}

void PIDRos::WlSenscallback(const std_msgs::Float32::ConstPtr& msg)
{
        s_wl = msg->data;
}


void PIDRos::Run(){
    double dt;
    current_time = ros::Time::now();
    dt = (current_time - last_time).toSec();
    last_time = current_time;

    des_wl=W;
    //err_wr =V-s_wr;
    //err_wl =W+(s_wl);
    err_wl=des_wl-s_wl;

    I_wl+=err_wl*dt;
    D_wl=(err_wl-err_wl_prev)/dt;
    U_wl=kp*err_wl+ki*I_wl+kd*D_wl;
    err_wl_prev=err_wl;

    std_msgs::Float32 w_l;
    w_l.data=U_wl;
    rot_vel_wl.publish(w_l);
}


int main(int argc, char **argv)
{

  cout<<"Initialising PID_Ros connection"<<endl;

  ros::init(argc, argv, "pid_ctl");

  cout<<"PID_Ros connected"<<endl;

  PIDRos pid_ros;

  ros::Rate loop_rate(50);

  while(ros::ok())
  {
    ros::spinOnce();
    
    pid_ros.Run();

    loop_rate.sleep();
  }

  return 0;
}
