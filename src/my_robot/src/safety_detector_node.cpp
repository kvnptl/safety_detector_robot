#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

int height {1};
int width {2};

ros::Publisher drive_pub;

void Callback(const sensor_msgs::LaserScan msg)
{
  ROS_INFO("Front object distance [%f] \n", msg.ranges[359]);
  ROS_INFO("Right object distance [%f] \n", msg.ranges[0]);
  ROS_INFO("Left object distance [%f] \n", msg.ranges[719]);
  ROS_INFO("\n");
  if ((msg.ranges[359] < height) || (msg.ranges[0] < (width/2)) || (msg.ranges[719] < (width/2))){

      geometry_msgs::Twist data1;

      data1.linear.x = 0;
      data1.linear.y = 0;
      data1.linear.z = 0;

      data1.angular.x = 0;
      data1.angular.y = 0;
      data1.angular.z = 0;

      drive_pub.publish(data1);
  }
  else{

      geometry_msgs::Twist data2;
      
      data2.linear.x = 0.05;
      data2.linear.y = 0;
      data2.linear.z = 0;

      data2.angular.x = 0;
      data2.angular.y = 0;
      data2.angular.z = 0;

      drive_pub.publish(data2);
  }

}


int main(int argc, char** argv){
    // Initialize the node
    ros::init(argc, argv, "safety_detector_node");
    ros::NodeHandle n;
    ros::Rate r(1);

    drive_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Subscriber laser_data = n.subscribe("scan", 1000, Callback);

    ros::spin();

    return 0;
}
