#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "iostream"

using namespace std;

class MotorSensor
{
	float s_wl,dt;
	double R,k1,k2,J,L,b,m,u_l,u;
	double i_now,i_prev,w_now,w_prev;
	double rads_l;
	double motor_current_l;
        double ctr_w=1;
	ros::NodeHandle node;
        ros::Subscriber sens_vel_wl,subcrt,subvtg;
        ros::Publisher rot_vel_wl,Vel_i_wl;
        ros::Time current_time, last_time;
public:
	MotorSensor();
	void Run();
	void WlSenscallback(const std_msgs::Float32& msg);
	void crt_lSenscallback(const std_msgs::Float32& msg);
	void vtg_lSenscallback(const std_msgs::Float32& msg);
	
};

MotorSensor::MotorSensor(){
	rot_vel_wl=node.advertise<std_msgs::Float32>("Vel_ctl/wl",1000);
	Vel_i_wl = node.advertise<std_msgs::Float32>("Vel_i_l",1000);
	
	sens_vel_wl=node.subscribe("Vel_sens/wl",1000,&MotorSensor::WlSenscallback,this);

	subcrt = node.subscribe("current",1000,&MotorSensor::crt_lSenscallback,this);
	subvtg = node.subscribe("voltage",1000,&MotorSensor::vtg_lSenscallback,this);
	
    	R=8.7;
    	k1=0.03334;
    	k2=0.03334;
    	J=0.01*(1e-4);
    	L=0.003;
    	b=3.5*(1e-6);
    	m=0;
	u_l=0;
    	s_wl=0;
    	i_now=0; 
    	i_prev=0;
    	w_now=0; 
    	w_prev=0;

    	current_time = ros::Time::now();
    	last_time = ros::Time::now();
    	dt=0.0001;	
}



void MotorSensor::WlSenscallback(const std_msgs::Float32& msg)
{
    
    ROS_INFO("W_l = %g", msg.data);
}

void MotorSensor::crt_lSenscallback(const std_msgs::Float32& msg)
{
  ROS_INFO("current_l = %g", msg.data);
  //i_prev = msg.data;  
  u_l = ctr_w*0.45;
  //u = 0.45;
  i_now=dt*(-R*i_prev-k1*w_prev+u_l)/L+i_prev;
  
  w_now=dt*(k2*i_prev-b*w_prev-m)/J+w_prev;

  i_prev=i_now;
  w_prev=w_now;

  ROS_INFO("i_now = %g", i_prev);
  ROS_INFO("dt = %g", dt);
  ROS_INFO("w_now = %g", w_now);   
}

void MotorSensor::vtg_lSenscallback(const std_msgs::Float32& msg)
{
  ROS_INFO("vtg_l = %g",msg.data);
  
}

void MotorSensor::Run(){
    double dt;
    current_time = ros::Time::now();
//    dt = ((current_time - last_time).toSec())/1000;
    last_time = current_time;

//left encoder reading
    std_msgs::Float32 w_l;
    w_l.data = 1;
    ctr_w = w_l.data;
    rot_vel_wl.publish(w_l);
//left current sensor calculated speed
    std_msgs::Float32 i_wl;
    i_wl.data = w_now;
    Vel_i_wl.publish(i_wl);

}


int main(int argc, char **argv)
{

  cout<<"Initialising MotorSensor connection"<<endl;

  ros::init(argc, argv, "final");

  cout<<"MotorSensor connected"<<endl;

  MotorSensor motorsensor;

  ros::Rate loop_rate(1000);

  while(ros::ok())
  {
    ros::spinOnce();
    
    motorsensor.Run();

    loop_rate.sleep();
  }

  return 0;
}





