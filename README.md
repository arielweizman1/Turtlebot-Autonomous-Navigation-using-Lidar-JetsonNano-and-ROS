
<h1 align="center">
  <br>
Turtlebot Autonomous Navigation:

  <br>
 </h1>
 </h1>
  <p align="center">
    <a • href="https://www.linkedin.com/in/ariel-weizman/">Ariel Weizman</a> 
    <a • href="https://www.linkedin.com/in/shir-erdreich/">Shir erdreich</a>
  </p>
  
Make your [Turtlebot2](https://www.turtlebot.com/turtlebot2/) run on ROS Melodic with [Nvidia Jetson nano](https://developer.nvidia.com/embedded/jetson-nano-developer-kit)  (Ubuntu 18.04) and [rplidar A1](https://www.slamtec.com/en/Lidar/A1).

![](https://www.turtlebot.com/assets/images/turtlebot_2_lg.png)

- [Turtlebot Autonomous Navigation:](#turtlebot-autonomous-navigation:)
  * [Step 0 - Setup Nvidia Jetson Nano on SD card](#step-0---setup-nvidia-jetson-nano-on-sd-card)
  * [Step 1 - Install ROS Melodic](#step-1---install-ros-melodic)
  * [Step 2 - Install the Turtlebot Package](step-2---install-the-turtlebot-package)
  * [Step 3 - Install rplidar Package](#step-3---install-rplidar-package)
  * [Step 4 - SLAM Algorithm - Install hector slam Package](#step-4---slam-algorithm---install-hector-slam-package)
  * [Step 5 - Control the jetson remotly](#Step-5---control-the-jetson-remotly)
  * Step 6 - Install turtlebot navigation package
  * Step 7 - How to save and load a map
  * Step 8 - Step 8 - Autonomous Navigation
  * [References](#references)

## Step 0 - Setup Nvidia Jetson Nano on SD card

You can learn about the parts of your jetson nano kit [here](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit#intro).

After you know your kit, you'll want to follow the [setup instructions](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit#write) for setting up the Ubuntu SD card image. At time of writing image is Ubuntu 18.04.

## Step 1 - Install ROS Melodic

You'll want to install ROS Melodic so follow the [instructions here](http://wiki.ros.org/melodic/Installation/Ubuntu).

### Step 1.1 - Learn about ROS
If you don't have a lot of experience with ROS check out the [start guide](http://wiki.ros.org/ROS/StartGuide) and follow the links there for an [intro](http://wiki.ros.org/ROS/Introduction).

The [tutorials](http://wiki.ros.org/ROS/Tutorials) are very good, so check them out.
I also highly recommend to follow the tutorials with [this](https://www.youtube.com/watch?v=HMXYXcCMd-Y&list=PL1R5gSylLha3i1nbDdmpkJ6wBVudIJheI&index=2) videos series. 

## Step 2 - Install the Turtlebot Package

The following assumes that you've created a catkin workspace in the default directory as in the [tutorial](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).
We'll have to first clone the package from github into the catkin workspace. 
from: https://github.com/gaunthan/Turtlebot2-On-Melodic
Now run the following command (inside the root of catkin workspace) to build up running environment for Turtlebot2:
``` 
curl -sLf https://raw.githubusercontent.com/gaunthan/Turtlebot2-On-Melodic/master/install_basic.sh | bash
catkin_make
```
source your setup file before running any .launch files
```
source ~/catkin_ws/devel/setup.bash
```
### Run Turtlebot
```
roslaunch turtlebot_bringup minimal.launch
```
If nothing wrong, you will hear the Turtlebot2 give out a reminder.

### Test Turtlebot2
If you want to use keyboard to control it, just run the following command
```
source ./devel/setup.bash
roslaunch turtlebot_teleop keyboard_teleop.launch
``` 
## Step 3 - Install rplidar Package

On the turtlebot at our lab, we have an rplidar (version A1). So we need to install the rplidar package by cloning the git file:
```
cd ~/catkin_ws/src
git clone https://github.com/Slamtec/rplidar_ros.git
```
problems occurred using current version thus we used an older version :
```
git chackout 1.7.0
```
After cloning we build the catkin:
```
cd ~/catkin_ws
catkin_make
```

### Run rplidar
After installing the lidar package, to run it we need to first set the read/write authorisations.
First we make sure the turtlebot is turned on.
Second we check where the lidar is connected. 
We run the following command:
```
ls -l /dev |grep ttyUSB
```
The output should be something like:
```console
jetson0@jetson-nano:~/catkin_ws$ ls -l /dev |grep ttyUSB
lrwxrwxrwx  1 root    root           7 Nov  5 11:45 gps0 -> ttyUSB0
lrwxrwxrwx  1 root    root           7 Nov  5 11:45 kobuki -> ttyUSB1
crw-rw-rw-  1 root    dialout 188,   0 Nov  5 13:13 ttyUSB0
crw-rw-rw-  1 root    dialout 188,   1 Nov  5 11:45 ttyUSB1
```
our lidar has the name of `gps0` so we need to change the `ttyUSB0` permissions:
```
sudo chmod 777 /dev/ttyUSB0
```

You'll also want to change the rplidar launch to take the tf in consideration. In `/home/Arielshir22/catkin_ws/src/rplidar_ros/launch/rplidar.launch` add the following line before `</launch>`:
```
  <node pkg="tf" type="static_transform_publisher" name="base_frame_2_laser"
   args="0 0 0.05 0 0 0 0 /base_link /laser 50" />
```
I had a lidar orientation Incompatibility to the robot odometry (turned 180 degrees) thus i used the next tf In `/home/Arielshir22/catkin_ws/src/rplidar_ros/launch/rplidar.launch` before `</launch>`:
```
  <node pkg="tf" type="static_transform_publisher" name="base_frame_2_laser"
   args="0 0 0.05 3.14 0 0 0 /base_link /laser 50" />
```

Now we can execute a launch file:
```
roslaunch rplidar_ros view_rplidar.launch
```

For more info check out the [ros wiki page](http://wiki.ros.org/rplidar)

## Step 4 - SLAM Algorithm - Install hector slam Package
You can use [hector mapping](http://wiki.ros.org/hector_mapping).
To install do the regular drill: 
```
cd ~/catkin_ws/src
git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git
```
I used the `melodic-devel` branch:
```
git checkout melodic-devel
```
After cloning we build the catkin:
```
cd ~/catkin_ws
catkin_make
```
### Adjust tf names in hector mapping launch file:

You'll need to change the tf names in the launch files so that they can work with each other.
In hector slam the odom_frame is called nav. Change these names in any launch file that you'll want to use. for example in `/home/Arielshir22/catkin_ws/src/hector_slam/hector_mapping/launch/mapping_default.launch` you'll want to change:

```
    <param name="odom_frame" value="nav"/>
```

to:
```
    <param name="odom_frame" value="odom"/>
```

Now, in order to work on real robot (not simulation), you'll need to change in `/home/Arielshir22/catkin_ws/src/hector_slam/hector_slam_launch/launch/tutorial.launch` :
from:
```
  <param name="/use_sim_time" value="true"/>
```
to:
```
  <param name="/use_sim_time" value="false"/>
```

## Run Everything together

1. Open a new terminal and launch turtlebot :

    roslaunch turtlebot_bringup minimal.launch
    
2. Open a new terminal and launch the rplidar:

    roslaunch rplidar_ros rplidar.launch
    
3. Open a new terminal and launch hector-slam:

    roslaunch hector_slam_launch tutorial.launch
    
4. Open a new terminal and launch the keyboard_teleop:

    roslaunch turtlebot_teleop keyboard_teleop.launch

Now (by clicking on the teleop terminal) you can control the robot with your keyboard and map your room!

## Step 5 - Control the jetson remotly
You'll want the robot to be mobile and control/monitor it from afar. In order to see the rviz during the navigation, we use vnc.

You'll need to follow the [instructions](https://developer.nvidia.com/embedded/learn/tutorials/vnc-setup).
Make sure that you are using the same wifi in the jetson and in your remote computer.

I also followed the instructions [here](https://medium.com/@bharathsudharsan023/jetson-nano-remote-vnc-access-d1e71c82492b).
You can check that your vnc is working by running:
```
ps -ef|grep vnc
```
Next, download any vnc server to your remote computer. i recommand vnc viewer.

to check what is your ip simply run on terminal `ifconfig`
and than type the ip in your vnc server.

## Step 6 - Install turtlebot navigation package
In order to make an Autonomous Navigation, install the [ROS Navigation Stack](http://wiki.ros.org/navigation)
```
cd ~/catkin_ws/src
git clone https://github.com/ros-planning/navigation.git
```
I used the `melodic-devel` branch:
```
git checkout melodic-devel
```
After cloning we build the catkin:
```
cd ~/catkin_ws
catkin_make
```
## Step 7 - How to save and load a map
### save the map
The way to save a map is to use a package called map_server. This package will save map data to a yaml and pgm formatted file.
You need to install map_server.
```
sudo apt-get install ros-melodic-map-server
```
(you might already installed it with the navigation stack)

Launch the mapping process 
```
    roslaunch hector_slam_launch tutorial.launch
```

After some strolling  around, when you are happy with the map that you see in rviz, you can save the map as map1.yaml and map1.pgm. Open a new terminal.
```
cd ~/catkin_ws/src/turtlebot_apps/turtlebot_navigation/maps
```
than type 'rosrun map_server map_saver -f <your_map_name>'
```
rosrun map_server map_saver -f lab1
```
Maps will save to the turtlebot_navigation/maps directory.

### Load the map
To load the map. In a new terminal window, type:
```
rosrun map_server map_server lab1.yaml
```
Open rviz in another terminal.
```
rviz
```

## Step 8 - Autonomous Navigation
First, please make sure that keyboard_teleop.launch is closed. if not, just ctrl+c to terminate this terminal.
to navigate we use amcl package
Now, add the rplidar_costmap_params.yaml from my files to turtlebot_navigation/param directory.
Next, change the Laser_amcl_demo.launch file to the one in my files.


