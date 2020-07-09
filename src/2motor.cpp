#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "iostream"
#include "math.h"

using namespace std;

class ModelRos
{
    float s_wr,s_wl,dt,current_l,current_r;
    double R,k1,k2,J,L,b,m,u_l,u_r;
    double i_l_now,i_l_prev,w_l_now,w_l_prev,i_r_now,i_r_prev,w_r_now,w_r_prev;
    double rads_l,rads_r;
    double ctr_w_l, ctr_w_r;
    ros::NodeHandle node;
    ros::Subscriber r_vel_wl, r_vel_wr, sens_vel_wr,sens_vel_wl;
    ros::Publisher Vel_i_wl, Vel_i_wr ;
    ros::Time current_time, last_time;
public:
    ModelRos();
    void Run();
 //  void crt_lSenscallback(const std_msgs::Float32& msg);
 //  void crt_rSenscallback(const std_msgs::Float32& msg);
    void WlctlCallback(const std_msgs::Float32::ConstPtr& msg);
    void WrctlCallback(const std_msgs::Float32::ConstPtr& msg);
	
};

ModelRos::ModelRos(){

    r_vel_wl=node.subscribe("Vel_ctl/wl",100,&ModelRos::WlctlCallback,this);
    r_vel_wr=node.subscribe("Vel_ctl/wr",100,&ModelRos::WrctlCallback,this);

    Vel_i_wl = node.advertise<std_msgs::Float32>("Vel_cal/wl",10);
    Vel_i_wr = node.advertise<std_msgs::Float32>("Vel_cal/wr",10);
	
 //   subcrt_l = node.subscribe("current",100,&MotorSensor::crt_lSenscallback,this);
 //   subcrt_r = node.subscribe("current",100,&MotorSensor::crt_rSenscallback,this);

	//Initialize variables
    R=8.7;
    k1=0.03334;
    k2=0.03334;
    J=1.8*(1e-6);
    L=0.003;
    b=3.5*(exp(-7));
    m=0;
    s_wr=0;
    s_wl=0;

    i_l_now=0; 
    i_l_prev=0.0;
    w_l_now=0; 
    w_l_prev=0.0;

    i_r_now=0; 
    i_r_prev=0.0;
    w_r_now=0; 
    w_r_prev=0.0;
    current_l=0;
    current_r=0;


    current_time = ros::Time::now();
    last_time = ros::Time::now();
    //dt=0.001;
}


	//Callback functions


//void FeedbackRos::crt_lSenscallback(const std_msgs::Float32& msg)
//{
//       current = msg->data;
//}
//void FeedbackRos::crt_rSenscallback(const std_msgs::Float32& msg)
//{
//       current = msg->data;
//}

void ModelRos::WlctlCallback(const std_msgs::Float32::ConstPtr& msg)
{
	 ctr_w_l = msg->data;
}

void ModelRos::WrctlCallback(const std_msgs::Float32::ConstPtr& msg)
{
	 ctr_w_r = msg->data;
}


void ModelRos::Run(){
    double dt;
    current_time = ros::Time::now();
    dt = (current_time - last_time).toSec();
    last_time = current_time;

//left speed estimated
    u_l=12.48*ctr_w_l-3.5;
    i_l_now=dt*(-R*i_l_prev-k1*w_l_prev+u_l)/L+i_l_prev; 
    w_l_now=dt*(k2*i_l_prev-b*w_l_prev-m)/J+w_l_prev;
    i_l_prev=i_l_now;
    w_l_prev=w_l_now;
    if(w_l_now<0){
       w_l_now = 0;}

//right speed estimated
    u_r=12.48*ctr_w_r-3.5;
    i_r_now=dt*(-R*i_r_prev-k1*w_r_prev+u_r)/L+i_r_prev; 
    w_r_now=dt*(k2*i_r_prev-b*w_r_prev-m)/J+w_r_prev;
    i_r_prev=i_r_now;
    w_r_prev=w_r_now;
    if(w_r_now<0){
       w_r_now = 0;}
   
//left current sensor calculated speed
    std_msgs::Float32 i_wl;
    i_wl.data = w_l_now;
    Vel_i_wl.publish(i_wl);
//right current sensor calculated speed
    std_msgs::Float32 i_wr;
    i_wr.data = w_r_now;
    Vel_i_wr.publish(i_wr);
}

int main(int argc, char **argv)
{

  cout<<"Initialising FeedbackRos connection"<<endl;

  ros::init(argc, argv, "2motor");

  cout<<"FeedbackRos connected"<<endl;

  ModelRos modelros;

  ros::Rate loop_rate(10000);

  while(ros::ok())
  {
    ros::spinOnce();
    
    modelros.Run();

    loop_rate.sleep();
  }

  return 0;
}


