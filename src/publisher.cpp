#include "ros/ros.h"
#include "std_msgs/Float32.h"


int main(int argc, char **argv)
{

   ros::init(argc, argv, "testt");
   ros::NodeHandle n;
 
   ros::Publisher pub=n.advertise<std_msgs::Float32>("test", 1000);
   ros::Rate loop_rate(10);

 
   int count = 0;
   while (ros::ok())
{

   std_msgs::Float32 msg;


   msg.data = 5;

   pub.publish(msg);

   
   ros::spinOnce();
   
 
   loop_rate.sleep();
   ++count;
}
 

   ros::spin();
 

   return 0;
}
