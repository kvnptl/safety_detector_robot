#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

// Initialize the node
ros::init(argc, argv, "safety_detector_node");
ros::NodeHandle n;

ros::Rate r(1);
ros::Publisher drive_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

void Callback(const sensor_msgs::LaserScan msg)
{
  ROS_INFO("I heard: [%s]", msg);
//   drive_pub.publish(marker);
}


int main(int argc, char** argv){

  ros::Subscriber sub = n.subscribe("scan", 1000, Callback);
  
  ros::spin();

  return 0;
}
