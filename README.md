# Safety_detector_robot

Here, I am using linear equations to make a shield around the robot for obstacle detection.

I am considering +ve slope slope line and -ve slope line for making it as a triangle. 

Overview: A subscriber function receives laser scan data (Format: 1-D array of float values) from hokuyu sensor and checking whether data range values fall whithin robot shield. If obstacle detected whithin the limit, then the robot will immediately stop by publishing 0 value on cmd_vel topic (subscribed by gazebo). If no obstacle detects,then robot will constinue its jounrney forward untill it hits the obstacle.

