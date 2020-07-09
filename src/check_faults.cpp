#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"

using namespace std;

class CheckRos
{
	double w_sens_wl,w_sens_wr,w_cal_wl,w_cal_wr;
	double threshold;
	int flag_l, flag_r;
	ros::NodeHandle node;
	ros::Publisher vel_flag_wl, vel_flag_wr;
	ros::Subscriber sens_vel_wl, sens_vel_wr, cal_vel_wl, cal_vel_wr;
	
	
public:
	CheckRos();
	void Run();
	void WrSensCallback(const std_msgs::Float32::ConstPtr& msg);
	void WlSensCallback(const std_msgs::Float32::ConstPtr& msg);
	void WrcalCallback(const std_msgs::Float32::ConstPtr& msg);
	void WlcalCallback(const std_msgs::Float32::ConstPtr& msg);
		
};

CheckRos::CheckRos()
{
	//publisher
	vel_flag_wl = node.advertise<std_msgs::Float32>("Vel_warn/wl",10);
	vel_flag_wr = node.advertise<std_msgs::Float32>("Vel_warn/wr",10);
	
	//subscriber
	sens_vel_wr=node.subscribe("Vel_sens/wr",10,&CheckRos::WrSensCallback,this);
	sens_vel_wl=node.subscribe("Vel_sens/wl",10,&CheckRos::WlSensCallback,this);
	
	cal_vel_wr =node.subscribe("Vel_cal/wr",10,&CheckRos::WrcalCallback,this);
	cal_vel_wl =node.subscribe("Vel_cal/wl",10,&CheckRos::WlcalCallback,this);	
	
	
	threshold = 10;
	flag_l=0;
	flag_r=0;
}


void CheckRos::WrSensCallback(const std_msgs::Float32::ConstPtr& msg)
{
		w_sens_wr = msg->data;
}
void CheckRos::WlSensCallback(const std_msgs::Float32::ConstPtr& msg)
{
		w_sens_wl = msg->data;
}
void CheckRos::WrcalCallback(const std_msgs::Float32::ConstPtr& msg)
{
		w_cal_wr = msg->data;
}
void CheckRos::WlcalCallback(const std_msgs::Float32::ConstPtr& msg)
{
		w_cal_wl = msg->data;
}

void CheckRos::Run()
{
	if(!((w_sens_wl <(w_cal_wl+10))&&((w_cal_wl-10)<w_sens_wl)))
		flag_l=1;
	else 
		flag_l=0;
	if(!((w_sens_wr <(w_cal_wr+10))&&((w_cal_wr-10)<w_sens_wr)))
		flag_r=1;
	else
		flag_r=0;
	
	std_msgs::Float32 flagl;
	flagl.data = flag_l;
	vel_flag_wl.publish(flagl);
	
	std_msgs::Float32 flagr;
	flagr.data = flag_r;
	vel_flag_wr.publish(flagr);

}

int main(int argc, char **argv)
{
	cout<<"checking faults"<<endl;
	ros::init(argc, argv, "check");
	cout<<"Check_Ros conncected"<<endl;
	CheckRos check_ros;
	ros::Rate loop_rate(20);
	while(ros::ok())
	{
		ros::spinOnce();
		check_ros.Run();
		loop_rate.sleep();
	}
	return 0;
	
	
	
}


















