#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

// safety shied dimentions around the robot
double height{1}; //from the center of the robot towards front side
double width{2};  //half on right and half on left side of the robot

//line equation variables; y = mx + c
double x1 = width / 2;
double y2 = height;
double m = (y2 / x1); //slope

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
        data.linear.x = 0.2; //drive robot at constant 0.2 m/s speed
    }
    else
    {
        data.linear.x = 0.0; //stop robot
        ROS_INFO("OBSTACLE AHEAD, STOP!!!");

        //change path
    }

    //publish data on cmd_vel topic
    drive_pub.publish(data);
}

//Safety_check function: check if obstacle within a certain range
// input: Laser scan data
// output: none
void Safety_check(const sensor_msgs::LaserScan msg)
{
    //check if obstacle present or not
    bool is_obstacle = false;
    
    //laser scan data points size (default values are start: -1.57 to stop: 1.57 with resolution of 0.00436111, total 720 points)
    double len = msg.ranges.size();
    int half_len = len / 2;
    
    //Debug
    // ROS_INFO("Max ranges: [%f]\n", len);
    // ros::Duration(8).sleep(); // sleep for 8 seconds
    
    //laser scan sensor sampling resolution
    double vertical_resolution = height / half_len;
     // ROS_INFO("Sample resolution: [%f]\n", vertical_resolution);

    //x-axis as width of the robot shield
    double x; 

    //laser scan sensor collects data from 0 to 180 degree
    for (int i = 0; i < half_len; i++)
    {
        //line equation x = (y - c) / m;
        x = (y2 - (vertical_resolution * (half_len - 1 - i))) / (m); //making this equation +ve all the time (absolute)


        //front side obstacle
        if (msg.ranges[half_len-1] < height){
            is_obstacle = true;
            ROS_INFO("OBSTACLE ON FRONT SIDE");
            break;
        }
                
        //from 0 to 90 degree
        //consider -ve slope line
        if (msg.ranges[half_len - 1 - i] < x)
        {
            is_obstacle = true;
            ROS_INFO("OBSTACLE ON RIGHT SIDE");
            break;
        }
        //from 90 to 180 degree
        //consider +ve slope line
        if (msg.ranges[half_len + i] < x)
        {
            is_obstacle = true;
            ROS_INFO("OBSTACLE ON LEFT SIDE");
            break;
        }

        //no obstacle detected on laser data
        else
        {
            is_obstacle = false;
        }
    }

    //send drive command to robot
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
    ros::Subscriber laser_data = n.subscribe("scan", 100, Safety_check);

    ros::spin();

    return 0;
}
