# Safety detector robot

## Output:

<img src="https://github.com/kvnptl/safety_detector_robot/blob/main/results/obstacle_detector_demo.gif" width="1080" height="600"/>

Here, I am using linear equations to make a shield around the robot for obstacle detection.

I am considering +ve slope slope line and -ve slope line for making it as a triangle.

Overview: A subscriber function receives laser scan data (Format: 1-D array of float values) from hokuyu sensor and checking whether data range values fall whithin robot shield. If obstacle detected whithin the limit, then the robot will immediately stop by publishing 0 value on cmd_vel topic (subscribed by gazebo). If no obstacle detects,then robot will constinue its jounrney forward untill it hits the obstacle.

## Logic:
Here, I am using linear equations to make a shield around the robot for obstacle detection.

I am considering the +ve slope line and -ve slope line for making it a triangle.

Code: Using For loop to check data from peak to the bottom (as shown by the array in the figure)

![algo](https://github.com/kvnptl/safety_detector_robot/blob/main/results/logic_overview.png)

## How to run:

1. Terminal 1: `roslaunch my_robot world.launch`
2. Terminal 2: `roslaunch my_robot safety_detector.launch`
