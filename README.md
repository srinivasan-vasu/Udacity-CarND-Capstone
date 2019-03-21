# Capstone Project
_Udacity "Self-Driving Car Engineer" Course_<br>
_Term #3 \ Project #3_
<br><br>
_-- Chang, Kenneth, Mahmoud, Srinivasan, Joydeep --_
<br><br>
_**Keywords**: Autonomous driving, Robot Operating System, path planning, machine learning, image classification, drive by wire._
<br><br>
_Video Output_:<br>
https://vimeo.com/...<br>

## Introduction

The final project of the Udacity "Self-Driving Car Engineer" course is a team code completion exercise in which the objective is to program a real car to drive autonomously around a simple test track. This is achieved by writing Robot Operating System (ROS) nodes implementing path planning and control by drive by wire (DBW) and by implementing a traffic light classifier by training a neural network. 

A simulator was used for testing during development. Please see the installation instructions provided in 'SETUP.md' to run the program with the simulator.

<p align="center">
<img src="imgs/unity.png" width="400"><br>
<i>The car driving in the simulator.</i>
</p>


## System Architecture

This section provides a general overview of autonomous vehicle architecture and presents the architecture implemented in this project.

### Overview of the Software Architecture of Autonomous Vehicles

This section provides a brief overview of the principal software subsystems typically found in an autonomous vehicle.

| Subsystem | Description |
| --------- | ----------- |
| Sensors | Lidar, radar, cameras, GPS, etc. |
| Perception | Processing of data obtained from sensors. |
| Planning | Behaviour and path planning. |
| Control | Send control commands to the vehicle. |

In the sensor fusion step, results of individual measurements from separate sensors may be combined using a Kalman filter. Sensor fusion data is passed to the perception module, which is responsible for localizing the vehicle's position and obtaining an understanding of the vehicle's environment. Localization is achieved by analysing sensor and map data to determine the vehicle's location.  Common techniques include Markov localization and particle filters. An understanding of the vehicle's environment can be obtained by applying image processing, machine learning and other means to accomplish tasks such as lane detection, traffic sign recognition, object detection and tracking and free space detection.

The output of the perception module is passed to the planning module. The planning module is responsible for route planning, predicting the trajectories of nearby vehicles, behaviour planning and trajectory generation. Path planning may make use of search algorithms such as the A* search algorithm or a dynamic programming approach to find the optimum trajectory given the available information. Predicting the behaviour of other vehicles can be achieved by employing a model based approach using a process model and/or by a data driven approach e.g. using a naive Bayes classifier.

Effecting the trajectory specified by the planning module is the responsibility of the control module, which makes use of PID, MPC or other sorts of controller to assert control over the acceleration, braking and steering of the vehicle through a drive by wire (DBW) interface. 

### Project Architecture

This project implements a reduced subset of the functionality described above, sufficient only to drive a car around a simple test track. The project is implemented using Robot Operating System (ROS), which is an open source robotics framework providing libraries and tools for working with hardware and passing messages between components. A program implemented in ROS consists of nodes which pass messages to each other through "topics" or "services".

A diagram illustrating the ROS nodes implemented in this project and how they relate to the subsystems typically found in an autonomous vehicle as described above is shown below. Note that obstacle detection was not implemented in this project.

<p align="center">
<img src="imgs/architecture.png" width="900"><br>
<i>Image source: Udacity.</i>
</p>

## Waypoint Updater Node

todo

<p align="center">
<img src="imgs/waypoint-updater-ros.png" width="600"><br>
<i>Image source: Udacity.</i>
</p>

### Path Planning

### Target Velocity

## Drive by Wire Node

todo

<p align="center">
<img src="imgs/dbw-node-ros.png" width="600"><br>
<i>Image source: Udacity.</i>
</p>

## Traffic Light Detection

todo

<p align="center">
<img src="imgs/tl-detector-ros.png" width="600"><br>
<i>Image source: Udacity.</i>
</p>

### Colour Detection

### Waypoint Publishing

## Discussion

todo
