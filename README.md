# Duckie-Bot-Localization
ROS nodes for a localization system based on Range Finder and Kalman filter.

## Introduction

This work has as its objective the implementation of a localization system for a differential robot, attributable to the duckiebot platform, in the ROS environment. The work is composed of a ROS package, which if executed in a simulated environment, allows to display the result of the localization.
The localization problem that is taken into consideration is a problem with a known map, in this work a tool used to generate a map of the environment in which the robot will have to be located is also shown, in a representation coherent with the localization process. The measuring instrument taken into consideration in this work is a range finder, specifically a laser scanner; a tool that can detect obstacles in the environment, returning the result as a point cloud.

For the determination of the position the Kalman Filter is used, a recursive matrix instrument, able to determine the state of a dynamic system starting from measurements subject to noise. The state is modeled as a Gaussian random variable, as are all the quantities considered in the system; in the localization process the Extended Kalman Filter is actually used, a tool able to use the same previous paradigm even in the presence of non-linear models. In the simulated system two environments are used:
- Gazebo: simulation and robotic prototyping environment
- RViz: 3D visualization tool for ROS


