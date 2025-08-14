#include <geometry_msgs/Twist.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

// Rename main to avoid duplicate symbol when including node source
#define main dont_use_main
#include "../src/safety_detector_node.cpp"
#undef main

geometry_msgs::Twist last_cmd;
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
  last_cmd = *msg;
}

class MoveCommandTest : public ::testing::Test {
protected:
  ros::NodeHandle nh;
  ros::Subscriber sub;

  void SetUp() override {
    last_cmd = geometry_msgs::Twist();
    drive_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
    sub = nh.subscribe("cmd_vel", 1, cmdVelCallback);
    while (drive_pub.getNumSubscribers() == 0 || sub.getNumPublishers() == 0) {
      ros::Duration(0.01).sleep();
    }
  }
};

TEST_F(MoveCommandTest, PublishesForwardWhenNoObstacle) {
  Move(false);
  ros::spinOnce();
  EXPECT_DOUBLE_EQ(0.2, last_cmd.linear.x);
}

TEST_F(MoveCommandTest, PublishesStopWhenObstacle) {
  Move(true);
  ros::spinOnce();
  EXPECT_DOUBLE_EQ(0.0, last_cmd.linear.x);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "safety_detector_node_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
