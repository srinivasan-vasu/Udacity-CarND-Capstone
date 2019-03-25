# A. Team Members 
_Team: AutoDreamCar_

|  Name                                   |    Email Address     			  |
|:---------------------------------------:|:---------------------------------:|
| Joydeep Ball		                      |   joydeep.ball@gmail.com    	  |
| Chang K Hong	                          |   changki.hong@gmail.com          |
| Kenneth Strouts                         |   kds3@protonmail.com             |
| Mahmoud Moataz                          |   mahmoud.moataz.zayed@gmail.com  |
| Srinivasan Vasu                         |   srinivasan.vasu@outlook.com     |
<br>

# B. Capstone Project Write-Up
_Udacity "Self-Driving Car Engineer" Course_<br>
_Term #3 \ Project #3_<br>

_**Keywords**: Autonomous driving, Robot Operating System, path planning, machine learning, image classification, drive by wire._
<br><br>
_Video Output_: [AutoDreamCar](https://www.youtube.com/watch?v=cIdQeTc6HjU&t=24s)<br>

## Introduction

The final project of the Udacity "Self-Driving Car Engineer" course is a team code completion exercise in which the objective is to program a real car to drive autonomously around a simple test track. This is achieved by writing Robot Operating System (ROS) nodes implementing path planning and control by drive by wire (DBW) and by implementing a traffic light classifier by training a neural network. 

A simulator was used for testing during development. Please see the installation instructions appended to the end of this report in order to run the program with the simulator.

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

## Traffic Light Detection Node

The traffic light detection node is implemented in 'tl_detector.py'. Its purpose is to detect traffic light colours {RED, GREEN, YELLOW} and publish {RED} light waypoints. The node subscribes to the following topics.

| Topics | Description |
| - | - |
| /base_waypoints | Contains the co-ordinates of map waypoints which the vehicle is to follow. |
| /current_pose | Contains the current position of the vehicle. |
| /image_color | Contains camera frame. |

The traffic light detection node publishes RED traffic light waypoints via the '/traffic_waypoint' topic.
<p align="center">
<img src="imgs/tl-detector-ros.png" width="600"><br>
<i>Image source: Udacity.</i>
</p>

This node uses a TensorFlow trained model for classification of the traffic light colour. Classification is implemented in '/tl_detector/light_classification_model/tl_classfier.py'. See the repository [Traffic Light Classification](https://github.com/linuxairhead/light_classification) for further details about the model classifier.

## Waypoint Updater Node

The waypoint updater node is implemented in 'waypoint_updater.py'. Its purpose is to set the target velocity of each waypoint based on traffic light data.  The node subscribes to the following topics.

| Topics | Description |
| - | - |
| /base_waypoints | Contains the co-ordinates of map waypoints which the vehicle is to follow. |
| /current_pose | Contains the current position of the vehicle. |
| /traffic_waypoint | Contains traffic light data. |
| /obstacle_waypoints | Not implemented. |

The waypoint updater node publishes a list of waypoints ahead of the vehicle and their target velocities to the topic '/final_waypoints'.
<p align="center">
<img src="imgs/waypoint-updater-ros.png" width="600"><br>
<i>Image source: Udacity.</i>
</p>

### Target Velocity

The vehicle's velocity is controlled in such a way so as to guarantee that the jerk and acceleration are not too high. This prevents discomfort for the passenger. Limits of 10 m/s^2 for acceleration and 10 m/s^3 for jerk apply. 

Two scenarios exist for changing the vehicle speed.
1. Matching speed limits.
2. Stopping for a traffic light.

A limit for the acceleration is implemented in the PID controller by limiting the control action based on the error. The car is set to travel at 95% of the speed limit so as to avoid violation of the road speed limit caused by the PID controller's steady state error and/or overshoot. A velocity updater is implemented to adjust the car to the new limit, while remaining flexible to road speed limit changes. The velocity updater updates the entire base waypoints because we update upon the change in the speed limit.
  
It is necessary to stop at the traffic light when it turns red. The aim is to obtain a smooth deceleration so as to avoid high jerk. The module implements linear deceleration proportional to the distance between the vehicle and the traffic light. Velocity is calculated based on a univariable equation which depends on distance. 
V = K * Distance, where:<br>
K is configurable parameter to tune the deceleration.<br>
a = K, maximum value for acceleration.

## Drive by Wire Node

The vehicle is controlled via a drive by wire interface which programs the throttle, brake and steering. When manual control is disabled (i.e. dbw_enabled is enabled), control values for steering, throttle and brake are calculated using subscribed topics and published to the topics '/vehicle/throttle_cmd', '/vehicle/steering_cmd' and '/vehicle/brake_cmd'. 
<p align="center">
<img src="imgs/dbw-node-ros.png" width="600"><br>
<i>Image source: Udacity.</i>
</p>

### Inputs

| Topics | Description |
| - | - |
| /vehicle/dbw_enabled | Specifies whether the DBW node is enabled or disabled. |
| /current_velocity | Contains the current velocity of the vehicle. |
| /twist_cmd | Contains waypoint follower data with necessary information for the controller. |

### Outputs

| Topics | Description |
| - | - |
| /vehicle/throttle_cmd | Contains throttle control information. |
| /vehicle/steering_cmd | Contains steering control information. |
| /vehicle/brake_cmd | Contains brake control information. |

The controller algorithm is implemented in 'twist_controller.py'. It uses a PID controller for control of the throttle, yaw control for control of the steering and finally filtered using low pass filter. The PID and yaw controller algorithms are implemented in 'pid.py' and 'yaw_controller.py' respectively.

## Result and Discussion

As can be observed in the attached [video](https://www.youtube.com/watch?v=cIdQeTc6HjU&t=24s), the vehicle is able to follow the path specified by the map waypoints while respecting the speed limit and obeying acceleration and jerk constraints.  The vehicle passes through the green traffic lights as intended and slows down to a stop for red traffic lights.
<br><br>
# C. Initial README (Setup Instructions)

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
