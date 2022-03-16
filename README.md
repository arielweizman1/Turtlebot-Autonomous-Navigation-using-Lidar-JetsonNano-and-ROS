
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


<p align="center">
  <img src="https://media.istockphoto.com/vectors/smart-home-kitchen-assistant-flat-illustration-vector-id1145748143?k=20&m=1145748143&s=612x612&w=0&h=59GPU9vcmle89N4sFXrFkKbPC0CAoWeluh5rmm7vdSY=" width="500" >
</p>

# [Turtlebot Autonomous Navigation Guide:](#turtlebot-autonomous-navigation:)
  * [Step 0 - Setup Nvidia Jetson Nano on SD card](#step-0---setup-nvidia-jetson-nano-on-sd-card)
  * [Step 1 - Install ROS Melodic](#step-1---install-ros-melodic)
  * [Step 2 - Install the Turtlebot Package](step-2---install-the-turtlebot-package)
  * [Step 3 - Install rplidar Package](#step-3---install-rplidar-package)
  * [Step 4 - SLAM Algorithm - Install hector slam Package](#step-4---slam-algorithm---install-hector-slam-package)
  * [Step 5 - Control the Jetson remotly](#step-5---control-the-jetson-remotly)
  * [Step 6 - Install turtlebot navigation package](#step-6---install-turtlebot-navigation-package)
  * [Step 7 - How to save and load a map](#step-7---how-to-save-and-load-a-map)
  * [Step 8 - Autonomous Navigation](#step-8---autonomous-navigation)
  * [References](#references)

## Step 0 - Setup Nvidia Jetson Nano on SD card
<p align="center">
  <img src="https://developer.nvidia.com/sites/default/files/akamai/embedded/images/jetsonNano/JetsonNano-DevKit_Front-Top_Right_trimmed.jpg" width="200" >
</p>

You can learn about the parts of your jetson nano kit [here](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit#intro).

After you know your kit, follow the [setup instructions](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit#write) for setting up the Ubuntu SD card image. (At time of writing this, image is Ubuntu 18.04).

## Step 1 - Install ROS Melodic
<p align="center">
  <img src="http://wiki.ros.org/melodic?action=AttachFile&do=get&target=melodic.jpg" width="230" >
</p>

You'll want to install ROS Melodic so follow the [instructions here](http://wiki.ros.org/melodic/Installation/Ubuntu).

### Step 1.1 - Learn about ROS
If you are new with ROS you can start with the [start guide](http://wiki.ros.org/ROS/StartGuide) and follow the links there for an [intro](http://wiki.ros.org/ROS/Introduction).

You can also do the [tutorials](http://wiki.ros.org/ROS/Tutorials). they are very good, so check them out!
I also highly recommend to follow the tutorials with [this](https://www.youtube.com/watch?v=HMXYXcCMd-Y&list=PL1R5gSylLha3i1nbDdmpkJ6wBVudIJheI&index=2) videos series. 

## Step 2 - Install the Turtlebot 2 Package

<p align="center">
  <img src="https://www.turtlebot.com/assets/images/turtlebot_2_lg.png" width="250" >
</p>

The following assumes that you've created a catkin workspace in the default directory as in the [tutorial](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).
We'll have to first clone the package from github into the catkin workspace. 
according to https://github.com/gaunthan/Turtlebot2-On-Melodic , run the following command (inside the root of catkin workspace) to build up running environment for Turtlebot2:
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
And it will output something like this

```
ROS_MASTER_URI=http://localhost:11311

process[turtlebot_teleop_keyboard-1]: started with pid [23757]

Control Your Turtlebot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit

currently:	speed 0.2	turn 1 
```
## Step 3 - Install rplidar Package
On the turtlebot at our lab, we have an rplidar (version A1). 
<p align="center">
  <img src="https://www.slamtec.com/images/a1/summary-section3.jpg" width="700" >
</p>

We need to install the rplidar package by cloning the git file:
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

![image](https://user-images.githubusercontent.com/57818213/158692871-734bf8eb-949c-4a8b-887c-faf5b424d1cd.png)

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
To map your surrounding, Use hector slam algorithm:
You can read about it [here](http://wiki.ros.org/hector_slam). note that the important node is [hector mapping](http://wiki.ros.org/hector_mapping).

To install : 
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

    `roslaunch turtlebot_bringup minimal.launch`
    
2. Open a new terminal and launch the rplidar:

    `roslaunch rplidar_ros rplidar.launch`
    
3. Open a new terminal and launch hector-slam:

    `roslaunch hector_slam_launch tutorial.launch`
    
4. Open a new terminal and launch the keyboard_teleop:

    `roslaunch turtlebot_teleop keyboard_teleop.launch`

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
Next, download any vnc server to your remote computer. I recommand vnc viewer.

to check what is your ip simply run on terminal `ifconfig`
and than type the ip in your vnc server.

## Step 6 - Install turtlebot navigation package
In order to make an Autonomous Navigation, install the [ROS Navigation Stack](http://wiki.ros.org/navigation) who is responsible for any kind of navigation.
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
### Save the map
Our plan is to navigate over a saved map of the environment.
For that purpose, we need a way to save and use a map.
The way to save a map is using a package called map_server. This package will save map data to a yaml and pgm formatted file.
You need to install map_server.
```
sudo apt-get install ros-melodic-map-server
```
(you might already installed it with the navigation stack)

Launch the mapping process 
```
    roslaunch hector_slam_launch tutorial.launch
```

After some strolling  around, when you are happy with the map that you see in rviz, you can save the map as map1.yaml and map1.pgm.

Now, open a new terminal.
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

To navigate, we use [amcl package](http://wiki.ros.org/amcl).
you already installed it within the navigation stack.
I have made some changes in the files to make it work together with the lidar:
1. Add the 'rplidar_costmap_params.yaml' from this repository to turtlebot_navigation/param directory.
2. Next, change the 'Laser_amcl_demo.launch' file to the one from my repository.

Pay attention that in the 'Laser_amcl_demo.launch' we are loading the map so you dont need to do it seperatly.
Remember to change the name according to your saved map_name whenever you pick different location.

To view the navigation in rviz, we need the 'view_navigation.launch' thus, install [turtlebot_apps](https://github.com/turtlebot/turtlebot_apps):
```
cd ~/catkin_ws/src
git clone https://github.com/turtlebot/turtlebot_apps.git
```
build:
```
cd ~/catkin_ws
catkin_make
```

### Run Everything together for navigation!

1. Close all the terminals.
2. Run every line in a new terminal:

```
roscore &
roslaunch turtlebot_bringup minimal.launch
roslaunch rplidar_ros rplidar.launch
roslaunch turtlebot_navigation laser_amcl_demo.launch
roslaunch turtlebot_rviz_launchers view_navigation.launch --screen
```
The rviz is open now. 

Press the 2D pose estimate, and press on the real location of the robot. make sure that you also set the orientation correct.

![image](https://user-images.githubusercontent.com/57818213/158700683-d59d11c8-3ef6-4bd1-b385-ce21cfb4479a.png)

The next step is to mark the navigation goal, by pressing the 2D nav goal

![image](https://user-images.githubusercontent.com/57818213/158701164-c2b77804-fcb6-41a2-ab7c-75c6ece57128.png)

But before you press the 2d nav goal, please make sure that keyboard_teleop.launch is closed. if not, just ctrl+c to terminate this terminal (otherwise the robot will not respond and it won't work)

Now, you can finally choose your destination on the map!!

thats it :)



## References
[1] Guidence and running lidar and hector slam: https://github.com/elishafer/Kobuki

[2] Turtlebot2 github source: https://github.com/gaunthan/Turtlebot2-On-Melodic

[3] 

[4]


