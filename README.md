# OpenCV_Project
Project involving OpenCV for a Mobile Robotics Lab (C++)

The Mobile Robotics Lab research explores various localization techniques for purposes such as search and rescue and obstacle avoidance. Since the research is in its the starting stages, I was asked to explore the option of using an overhead camera to localize turtlebots within a ROS network. It is expected that my understanding of ROS and OpenCV from this small project would contribute towards localization via alternative methods-- this makes the goal of this project to be to better understand OpenCV and ROS. More specifically, the goal is to understand how to incorporate data from a separate device (here the overhead camera) into the ROS network for localization purposes.

OpenCV is the machine learning and imaging library that is used to gather information from the overhead camera. Simple color-based image detection is applied with location information sent to the ROS network.

The filter calibration program consists of a small C++ script to quickly calibrate color settings to filter a specific color. The filter information is saved to a JSON file to be later read by a ROS publisher. The publisher will use the saved information to quickly lock onto the calibrated colors and will publish the positional information to the ROS network.

A seprate publisher/subscriber hybrid will subscribe to the previously menitoned publisher and itself publish the required angular and velocity information to control the robot.

...
