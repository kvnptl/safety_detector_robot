#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"


// safety shied dimentions around the robot
int height {1}; //from the center of the robot 
int width {2}; //half on right and half on left side of the robot

//robot drive command publisher
ros::Publisher drive_pub;

//Move function: publish drive command on /cmd_vel topic
// input: boolean command
// output: none
void Move(bool command){    
    geometry_msgs::Twist data;
    
    if (command){
        data.linear.x = 0.1; //drive robot at constant 0.1 m/s speed
    }
    else{
        data.linear.x = 0.0; //stop robot
        ROS_INFO("OBSTACLE DETECTED, STOP!!!");
    }
    
    //publish data on cmd_vel topic
    drive_pub.publish(data);
}

//Safety_check function: check if obstacle within a certain range
// input: Laser scan data
// output: none
void Safety_check(const sensor_msgs::LaserScan msg)
{
  ROS_INFO("Front object distance [%f] \n", msg.ranges[359]);
  ROS_INFO("Right object distance [%f] \n", msg.ranges[0]);
  ROS_INFO("Left object distance [%f] \n", msg.ranges[719]);
  ROS_INFO("\n");

  //laser scan sensor sampling resolution
  double y1 = (width/2)/(msg.ranges.size()/2);
  
  //laser scan sensor collects data from 0 to 180 degree
  for (int i=0; i<msg.ranges.size(); i++){
      
      //from 0 to 90 degree
      //consider -ve slope line
      if (i < (msg.ranges.size()/2)){
          if (msg.ranges[i] < (1 - y1*i)){
              Move(false);
          }
          else{
              Move(true);
          } 
      }

      //from 90 to 180 degree
      //consider +ve slope line
      else{
          if (msg.ranges[i] < (y1*i)){
              Move(false);
          }
          else{
              Move(true);
          }
      }
  }
}


int main(int argc, char** argv){
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
