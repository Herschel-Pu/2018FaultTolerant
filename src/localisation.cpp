#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "iostream"
#include "fstream"
#include "math.h"

using namespace std;
double trajectory[6][53];
int number_of_lines;
int row, column,ROW;
class LocalRos
{
	float vc, wc, v, w, vr, wr,radius,w_r,w_l;
	int k;
	float pose_mean[3];
	int flag_r,flag_l;
	double w_sens_wl,w_sens_wr,w_cal_wl,w_cal_wr;
	float x_all[], y_all[], xd_all[], yd_all[], vr_all[], wr_all[], xe_all[], ye_all[], theta_e_all[], s1_all[], s2_all[];
	double x_d, y_d, theta_d, v_d, w_d, v_d_dot, w_d_dot;
	double x, y, theta, xe, ye, theta_e;	
	double x_dot,y_dot,theta_dot,xe_dot,ye_dot,theta_e_dot,x_d_dot,y_d_dot,theta_d_dot;
	double vc_dot, wc_dot;
	float k1, P1, Q1, k2, k0, P2, Q2, s1, s2, s1_dot, s2_dot;  
		
	ros::NodeHandle node;
	ros::Subscriber sens_vel_wr,sens_vel_wl,cal_vel_wr, cal_vel_wl, flag_vel_wl, flag_vel_wr;
	ros::Publisher  ref_vel_V,ref_vel_W;
	ros::Time current_time, last_time;
	
public:
	LocalRos();
	void Run();
	void WrSensCallback(const std_msgs::Float32::ConstPtr& msg);
	void WlSensCallback(const std_msgs::Float32::ConstPtr& msg);
	void WrcalCallback(const std_msgs::Float32::ConstPtr& msg);
	void WlcalCallback(const std_msgs::Float32::ConstPtr& msg);
	void WrflagCallback(const std_msgs::Float32::ConstPtr& msg);
	void WlflagCallback(const std_msgs::Float32::ConstPtr& msg);
};

LocalRos::LocalRos()
{
	//publisher;
	ref_vel_V = node.advertise<std_msgs::Float32>("Vel_ctl/SP_V",10);
	ref_vel_W = node.advertise<std_msgs::Float32>("Vel_ctl/SP_W",10);
	//subscriber;
	sens_vel_wr=node.subscribe("Vel_sens/wr",10,&LocalRos::WrSensCallback,this);
	sens_vel_wl=node.subscribe("Vel_sens/wl",10,&LocalRos::WlSensCallback,this);
	
	cal_vel_wr =node.subscribe("Vel_cal/wr",10,&LocalRos::WrcalCallback,this);
	cal_vel_wl =node.subscribe("Vel_cal/wl",10,&LocalRos::WlcalCallback,this);
	
	flag_vel_wl = node.subscribe("Vel_warn/wl",10,&LocalRos::WlflagCallback,this);
	flag_vel_wr = node.subscribe("Vel_warn/wr",10,&LocalRos::WrflagCallback,this);
	
	
	//parameters initialization;	
    k=0;
    pose_mean[0] = 0;
    pose_mean[1] = 0;
    pose_mean[2] = 0;
    vc = 0;  wc = 0;vr = 0; wr = 0; radius = 0.05; v=0;	w=0;
    number_of_lines=0; flag_r=0 ;flag_l=0;
    x_all[0] = 0;
    y_all[0] = 0;
    xd_all[0] = 0;
    yd_all[0] = 0;   
    vr_all[0] = 0;
    wr_all[0] = 0;
    xe_all[0] = 0;
    ye_all[0] = 0;
    theta_e_all[0] = 0;
    s1_all[0] = 0;
    s2_all[0] = 0;
	trajectory[0][0]=0;
	w_sens_wl=0; w_sens_wr=0; w_cal_wl=0; w_cal_wr=0;w_l=0;w_r=0;
	x_d=0; y_d=0; theta_d=0; v_d=0; w_d=0; v_d_dot=0; w_d_dot=0;
	x=0; y=0; theta=0; xe=0; ye=0; theta_e=0;
	x_dot=0; y_dot=0; theta_dot=0; xe_dot=0; ye_dot=0; theta_e_dot=0; x_d_dot=0; x_d_dot=0; theta_d_dot=0;
	vc_dot=0; wc_dot=0;	
	s1=0; s2=0; s1_dot=0; s2_dot=0; 
	k1=0.4; 
	P1=0.01; 
	Q1=1;    
	k2=0.75; 	
	k0=1; 	
	P2=0.02; 	
	Q2=0.5;  
	current_time = ros::Time::now();
	last_time = ros::Time::now();
		
}
//callback functions;
void LocalRos::WrSensCallback(const std_msgs::Float32::ConstPtr& msg)
{
		w_sens_wr = msg->data;
}
void LocalRos::WlSensCallback(const std_msgs::Float32::ConstPtr& msg)
{
		w_sens_wl = msg->data;
}
void LocalRos::WrcalCallback(const std_msgs::Float32::ConstPtr& msg)
{
		w_cal_wr = msg->data;
}
void LocalRos::WlcalCallback(const std_msgs::Float32::ConstPtr& msg)
{
		w_cal_wl = msg->data;
}
void LocalRos::WrflagCallback(const std_msgs::Float32::ConstPtr& msg)
{
		flag_r = msg->data;
}
void LocalRos::WlflagCallback(const std_msgs::Float32::ConstPtr& msg)
{
		flag_l = msg->data;
}
void LocalRos::Run()
{
	double dt;
	int column, row;
	int ROW;

	
	//calculate dt;
	current_time = ros::Time::now();
	dt = (current_time-last_time).toSec();
	last_time = current_time;
	// read real robot speed;
	if(flag_r==1)
	{	
		w_r = w_cal_wr;
	}
	else
	{
		w_r = w_sens_wr;
	}
	if(flag_l==1)
	{
		w_l = w_cal_wl;
	}
	else
	{
		w_l = w_sens_wl;
	}
	wr = (w_r+w_l)/2;
	vr = wr * radius;
	
	//separate data from matrix;
	while (k < ROW)
	{
		x_d = trajectory[k][0];
		y_d = trajectory[k][1];
		theta_d = trajectory[k][2];
		v_d = trajectory[k][3];
		w_d = trajectory[k][4];
		v_d_dot = trajectory[k][5];
		w_d_dot = trajectory[k][6];
	
		x=pose_mean[0];
		y=pose_mean[1];
		theta=pose_mean[2];
		xe=cos(theta_d)*(x-x_d)+sin(theta_d)*(y-y_d);
		ye=-sin(theta_d)*(x-x_d)+cos(theta_d)*(y-y_d);
		theta_e=theta-theta_d;
    
		x_dot=vr*cos(theta);
		y_dot=vr*sin(theta);
		theta_dot=wr;
    
		x_d_dot=v_d*cos(theta_d);
		y_d_dot=v_d*sin(theta_d);
		theta_d_dot=w_d;
    
  
		xe_dot=-v_d+vr*cos(theta_e)+ye*w_d;
		ye_dot=vr*sin(theta_e)-xe*w_d;
		theta_e_dot=theta_dot-theta_d_dot;

    
		s1=xe_dot+k1*xe;
		s2=theta_e_dot+k2*theta_e+k0*ye;
    
		s1_dot=-Q1*s1-P1*copysign(1,s1);
		s2_dot=-Q2*s2-P2*copysign(1,s2);
    
		vc_dot=(s1_dot-k1*xe_dot+v_d_dot+vr*sin(theta_e)*theta_e_dot-ye_dot*w_d-w_d_dot*ye)/cos(theta_e);
		wc_dot=s2_dot-k0*ye_dot+w_d_dot-k2*theta_e_dot;

		pose_mean[0]=x+dt*vr*cos((pose_mean[2]+dt*wr+theta)/2);
		pose_mean[1]=y+dt*vr*sin((pose_mean[2]+dt*wr+theta)/2);
		pose_mean[2]=dt*wr+theta; 
		vc=vc+dt*vc_dot;
		wc=wc+dt*wc_dot;
	
		x_all[k] = x;
		y_all[k] = y;
		xd_all[k] = x_d;
		yd_all[k] = y_d;
		vr_all[k] =  vr;
		wr_all[k] =  wr;
		xe_all[k]=xe;
		ye_all[k]=ye;
		theta_e_all[k]=theta_e;
		s1_all[k] = s1;
		s2_all[k] = s2;

		k = k + 1; 
	
		v = vc;
		w = wc;
		
		//publish
		std_msgs::Float32 v_c;
		v_c.data = v;
		ref_vel_V.publish(v_c);
		
		std_msgs::Float32 w_c;
		w_c.data = w;
		ref_vel_W.publish(w_c);
	
	
	}

}
  
int main(int argc, char **argv)
{
	cout<<"Initialising Trajectory Setting" << endl;
	ros::init(argc,argv,"localisation");
	LocalRos local_ros;
	
	//read trajectory file
	ifstream in("TrajectoryCircle.txt");
	//count rows of txt file;	
	if(in.is_open())
	{
		while(!in.eof())
		{
			number_of_lines++;
		}
	}
	ROW = number_of_lines;	
	//read every row of the data respectively;
	if(!in)
	{
		cout <<"cannot open file.\n";
	}
	for(row=0;row<ROW;row++)
	{	
		for(column=0;column<7;column++)
		{
			in >> trajectory[row][column];							
		}

	}
    	in.close();
	ros::Rate loop_rate(20);
	while(ros::ok())
	{
		ros::spinOnce();
		local_ros.Run();
		loop_rate.sleep();
	}
	return 0;
}
