#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

// safety shied dimentions around the robot
int height{1}; //from the center of the robot
int width{2};  //half on right and half on left side of the robot

//line equation variables
double x1 = width / 2;
double y2 = height;
double m = (y2 / x1);

//robot drive command publisher
ros::Publisher drive_pub;

//Move function: publish drive command on /cmd_vel topic
// input: boolean command
// output: none
void Move(bool command)
{
    geometry_msgs::Twist data;

    if (!command)
    {
        data.linear.x = 0.05; //drive robot at constant 0.1 m/s speed
    }
    else
    {
        data.linear.x = 0.0; //stop robot
        ROS_INFO("OBSTACLE DETECTED, STOP!!!");
        // ros::Duration(5).sleep(); // sleep for 5 seconds
    }

    //publish data on cmd_vel topic
    drive_pub.publish(data);
}

//Safety_check function: check if obstacle within a certain range
// input: Laser scan data
// output: none
void Safety_check(const sensor_msgs::LaserScan msg)
{
    bool is_obstacle = false;
    double len = sizeof(msg.ranges) / sizeof(msg.ranges[0]);
    len = 720.0;
    int half_len = len / 2;
    // ROS_INFO("Max ranges: [%f]\n", len);
    //laser scan sensor sampling resolution
    double vertical_resolution = height / half_len;

    double x;

    // double side = width / 2;
    // ROS_INFO("Sample resolution: [%f]\n", y1);
    // ros::Duration(5).sleep(); // sleep for 5 seconds

    //laser scan sensor collects data from 0 to 180 degree
    for (int i = 0; i < half_len; i++)
    {
        //line equation x = (y - c) / m;
        x = (y2 - (vertical_resolution * (half_len - 1 - i))) / (m);

        //from 0 to 90 degree
        //consider -ve slope line
        if (msg.ranges[half_len - 1 - i] < x)
        {
            is_obstacle = true;
            ROS_INFO("RIGHT SIDE OBSTACLE");
            break;
        }
        //from 90 to 180 degree
        //consider +ve slope line
        if (msg.ranges[half_len + i] < x)
        {
            is_obstacle = true;
            ROS_INFO("LEFT SIDE OBSTACLE");
            break;
        }

        if (msg.ranges[half_len] < height)
        {
            is_obstacle = true;
            break;
        }

        else
        {
            is_obstacle = false;
        }
    }
    Move(is_obstacle);
}



int main(int argc, char **argv)
{
    // Initialize the node
    ros::init(argc, argv, "safety_detector_node");
    ros::NodeHandle n;
    ros::Rate r(1);

    //publisher: publish drive data on cmd_vel topic
    //buffer/queue size 100
    drive_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);

    //subscriber: subscribe to hokuyu laser scan data
    //buffer/queue size 100
    ros::Subscriber laser_data = n.subscribe("scan", 1, Safety_check);

    ros::spin();

    return 0;
}
