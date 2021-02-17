#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"


// safety shied dimentions around the robot
int height {1}; //from the center of the robot 
int width {2}; //half on right and half on left side of the robot

int cnt=0;


//robot drive command publisher
ros::Publisher drive_pub;

//Move function: publish drive command on /cmd_vel topic
// input: boolean command
// output: none
void Move(bool command){    
    geometry_msgs::Twist data;
    
    if (!command){
        data.linear.x = 0.05; //drive robot at constant 0.1 m/s speed
    }
    else{
        data.linear.x = 0.0; //stop robot
        ROS_INFO("OBSTACLE DETECTED, STOP!!!");
        ros::Duration(5).sleep(); // sleep for 5 seconds
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
  ROS_INFO("Counter: [%d]\n", cnt);
  cnt+=1;
  bool is_obstacle = false;
  
  //laser scan sensor sampling resolution
  double y1 = (width/2)/(msg.ranges.size()/2);
  
  //laser scan sensor collects data from 0 to 180 degree
  for (int i=0; i<msg.ranges.size(); i++){
      
      //from 0 to 90 degree
      //consider -ve slope line
      if (i < (msg.ranges.size()/2)){
          if (msg.ranges[i] < (1 - y1*i)){
              ROS_INFO("scan data value [%f] \n", msg.ranges[i]);
              ROS_INFO("line eq value [%f] \n", (1 - y1*i));
            //   ros::Duration(3).sleep(); // sleep for 5 seconds
            //   Move(false);
              ROS_INFO("IN LOOPPPPPPPPPPPPP  --> OBSTACLE DETECTED RIGHT SIDE, STOP!!!");
            //   ros::Duration(3).sleep(); // sleep for 5 seconds
              is_obstacle = true;
          }
          else{
              ROS_INFO("IN LOOPPPPPPPPPPPPP 0 to 90 degree");
              ROS_INFO("scan data value [%f] \n", msg.ranges[i]);
              ROS_INFO("line eq value [%f] \n", (1 - y1*i));
            //   ros::Duration(1).sleep(); // sleep for 5 seconds
            //   Move(true);
              is_obstacle = false;
          } 
      }

      //from 90 to 180 degree
      //consider +ve slope line
      else{
          if (msg.ranges[i] < (y1*i)){
              ROS_INFO("scan data value [%f] \n", msg.ranges[i]);
              ROS_INFO("line eq value [%f] \n", (y1*i));
            //   Move(false);
              ROS_INFO("IN LOOPPPPPPPPPPPPP 90 to 180 degree --> OBSTACLE DETECTED LEFT SIDE, STOP!!!");
            //   ros::Duration(3).sleep(); // sleep for 5 seconds
              is_obstacle = true;
          }
          else{
              ROS_INFO("IN LOOPPPPPPPPPPPPP 90 to 180 degree");
              ROS_INFO("scan data value [%f] \n", msg.ranges[i]);
              ROS_INFO("line eq value [%f] \n", (y1*i));
            //   ros::Duration(1).sleep(); // sleep for 5 seconds
            //   Move(true);
              is_obstacle = false;
          }
      }
  }

  Move(is_obstacle);
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
