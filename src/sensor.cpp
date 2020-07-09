#include "ros/ros.h"
#include "std_msgs/Float32.h"


void crt(const std_msgs::Float32& cmd2)
{
  ROS_INFO("current = %g", cmd2.data);
 
}
void vtg(const std_msgs::Float32& cmd1)
{
  ROS_INFO("voltage = %g",cmd1.data);
  
}



int main(int argc, char **argv)
{

   ros::init(argc, argv, "sensor");
   ros::NodeHandle n;
 
   ros::Subscriber sub=n.subscribe("current", 1000, crt);
   ros::Subscriber sub2=n.subscribe("voltage", 1000, vtg);

 

   ros::spin();
 

   return 0;
}
