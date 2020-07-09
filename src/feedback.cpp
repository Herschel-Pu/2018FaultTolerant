#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "iostream"
#include "math.h"

using namespace std;

class MotorSensor
{
	double s_wl,dt,current;
	double R,k1,k2,J,L,b,m,u_l,u;
	double i_now,i_prev,w_now,w_prev;
	double rads_l;
	double motor_current_l;
        double ctr_w;
	ros::NodeHandle node;
        ros::Subscriber sens_vel_wl,subcrt,subvtg,rot_vel_wl;
	ros::Subscriber u_wl;
        ros::Publisher i_cal_wl,Vel_cal_wl,curet;
        ros::Time current_time, last_time;
public:
	MotorSensor();
	void Run();
	void WlSenscallback(const std_msgs::Float32& msg);
	void crt_lSenscallback(const std_msgs::Float32::ConstPtr& msg);
	void vtg_lSenscallback(const std_msgs::Float32& msg);
	void controlvl(const std_msgs::Float32::ConstPtr& msg);
//	void ucallback(const std_msgs::Float32& msg);
	
};

MotorSensor::MotorSensor(){
	
	Vel_cal_wl = node.advertise<std_msgs::Float32>("Vel_cal/wl",100);
	i_cal_wl   = node.advertise<std_msgs::Float32>("i_cal/wl",100);
  //      curet = node.advertise<std_msgs::Float32>("i_sens/wl",10);
	
	sens_vel_wl=node.subscribe("Vel_sens/wl",100,&MotorSensor::WlSenscallback,this);
	subcrt = node.subscribe("current",100,&MotorSensor::crt_lSenscallback,this);
	subvtg = node.subscribe("voltage",100,&MotorSensor::vtg_lSenscallback,this);
	rot_vel_wl=node.subscribe("Vel_ctl/wl",100,&MotorSensor::controlvl,this);
//	u_wl=node.subscribe("u/wl",1000,&MotorSensor::ucallback,this);
	
    	R=8.7;
    	k1=0.03334;
    	k2=0.03334;
	//k2=0.005;
	//k2=0.03334;
	//k1=0.032;
    	//k2=0.032;
	//J=1.8*(1e-4);
	J=1.8*(1e-6);
    	//J=1.8*(exp(-6));
    	L=0.003;
	//b=3.5*(1e-6);
    	//b=2.5*(exp(-10));
	b=3.1*(exp(-7));
    	m=0.000;
	u_l=0;
    	s_wl=0;
    	i_now=0; 
    	i_prev=0;
    	w_now=0; 
    	w_prev=0;
	ctr_w=0;
	current=0;
    	//dt=0.001;
current_time = ros::Time::now();
    last_time = ros::Time::now();	
}

//void MotorSensor::ucallback(const std_msgs::Float32& msg)
//{
//   u_l = msg.data;
//}
void MotorSensor::controlvl(const std_msgs::Float32::ConstPtr& msg)
{ 
    ctr_w = msg->data;

//    if(i_now<0)
//	i_now=0;

}

void MotorSensor::WlSenscallback(const std_msgs::Float32& msg)
{
    
    ROS_INFO("W_l = %g", msg.data);



}

void MotorSensor::crt_lSenscallback(const std_msgs::Float32::ConstPtr& msg)
{
  //ROS_INFO("current_l = %g", msg.data);
  //i_prev = msg.data;  
  current = msg->data;
  

  ROS_INFO("i_now = %g", i_prev);

 // ROS_INFO("w_now = %g", w_now);   
}

void MotorSensor::vtg_lSenscallback(const std_msgs::Float32& msg)
{
  ROS_INFO("vtg_l = %g",msg.data);
  
}

void MotorSensor::Run(){
    double dt;
    current_time = ros::Time::now();
    dt = (current_time - last_time).toSec();
    last_time = current_time;

 //   u_l = ctr_w*0.645 - 0.186;
  //  u_l=11.1; 
   // u_l=0; 
    if(ctr_w>1)
	{
	ctr_w=1;
	} 
    u_l=11.1*ctr_w-3;
    if(u_l<0)
	{
	u_l=0;
	}
    //u_l = ctr_w * 0.66 - 0.215;
 //   i_now=dt*(-R*i_prev-k1*w_prev+u_l)/L+i_prev; 
 //   i_now=i_now*0.2+0.8*i_prev;
 //   i_prev = i_now;
 //   i_now = current*0.3 + i_prev*0.7;
 //   i_now = (i_now-0.03)/20;
 //   if(i_now<0){
//	i_now=0;}
  //  i_now=current;
 //   w_now = dt*(k2*i_prev-b*w_prev-m)/J+w_prev;
 //   u_l = ctr_w * 0.66 - 0.215;
    i_now=dt*(-R*i_prev-k1*w_prev+u_l)/L+i_prev; 
    w_now=dt*(k2*i_prev-b*w_prev-m)/J+w_prev;
    i_prev=i_now;
    w_prev=w_now;
    
   // w_prev=w_prev+(w_now-400);
    if(w_now<0){
       w_now = 0;}
    ROS_INFO("w_now = %g", w_now);
    ROS_INFO("current = %g", current);	
    //ROS_INFO("w_now = %g", (1e-2));
   // if(w_now>50){
//	w_now=0;}

//left calculated current
    std_msgs::Float32 i_wl;
    i_wl.data = i_now;
    i_cal_wl.publish(i_wl);	

    
//left calculated speed
    std_msgs::Float32 w_wl;
    w_wl.data = w_now;
    Vel_cal_wl.publish(w_wl);
//current 
 //   std_msgs::Float32 cs_wl;
 //   cs_wl.data=current;
  //  curet.publish(cs_wl);


}


int main(int argc, char **argv)
{

  cout<<"Initialising MotorSensor connection"<<endl;

  ros::init(argc, argv, "feedback");

  cout<<"MotorSensor connected"<<endl;

  MotorSensor motorsensor;

  ros::Rate loop_rate(10000);

  while(ros::ok())
  {
    ros::spinOnce();
    
    motorsensor.Run();

    loop_rate.sleep();
  }

  return 0;
}
 
