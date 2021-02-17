#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

bool send_stop = false;

void Callback(const sensor_msgs::LaserScan msg)
{
  ROS_INFO("I heard");
  send_stop = true;

}


int main(int argc, char** argv){
    // Initialize the node
    ros::init(argc, argv, "safety_detector_node");
    ros::NodeHandle n;
    ros::Rate r(1);

    ros::Publisher drive_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Subscriber laser_data = n.subscribe("scan", 1000, Callback);
    
    if (send_stop == true){
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

        data2.linear.x = 0.15;
        data2.linear.y = 0;
        data2.linear.z = 0;

        data2.angular.x = 0;
        data2.angular.y = 0;
        data2.angular.z = 0;

        drive_pub.publish(data2);
    }

    ros::spin();

    return 0;
}
