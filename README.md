# **Summerschool ETH notes**

# 1.  Super Mega Bot

![Screenshot](.imgs/smb.jpg)

## Available Sensors:
1. *D435* (front down) 
2. *T265* (fisheye front)
3. *RS-LiDAR-16* (on top) RobotSense, 16 beams, 10 hz
4. IMU synchronized with 1 *Flir Blackfly S* USB3 (RGB)
## Computational Power:
1. Intel NUC 
2. Nvidia Jetson Xavier AGX 
# 2. Networking
(Each robot has its own wifi)
* WiFi Name (SSD): SMB_26x where x is the robot number 
* Password (PSK): SMB_26x_RSS
* NUC has IP of 10.0.x.5
* Jetson has IP of 10.0.x.7

Connect to the robot's PC:
```
$ ssh <username>@10.0.x.5 
```
Login info:
* usr: teamN
* pwd: smb

Don't forget to build and source smb_opc



# 2. Hardware Equipment
T265 stereo fisheye
![Screenshot](.imgs/t265.PNG)


IMU : 400 Hz 
Joint Encoders: 50 hz


# 3. Installing Software
Visit this and follow the tutorial https://ethz-robotx.github.io/SuperMegaBot/

Run the simulation:
```
$ roslaunch smb_gazebo sim.launch launch_gazebo_gui:=true world:=WaA
```

Run the teleop with keyboard:

```
$ roslaunch smb_gazebo sim.launch launch_gazebo_gui:=false launch_rviz:=false  world:=WaA keyboard_teleop:=true

```

# SMB Basics
Run on the robot after ssh:

```
$ roslaunch smb smb.launch
```
In PC for visualization Rviz  you can do:
```
$ roslaunch smb_opc opc.launch
```

# Record Data + convert to costmap:
## Log a bag file.
At src/core/
```
$ ./record_sensors.sh
```
It logs the following topics:
```
/clock
/imu
/rosout
/rosout_agg
/rslidar/points
/tf
/tf_static
```
## Create 3D map with smb_slam
```
$ roslaunch smb_slam build_map_from_bag.launch rosbag_full_path:="ABSOULTE PATH TO BAG FILE"
```

To save the created **.pcd** file.
```
$ rosservice call /mapping_node/save_map
```

## Convert **.pcd** to **costmap**
~/<Your_workspace>/planning/smb_path_planner/smb_navigation/script/

Run: 
```
$ ./pcd_to_gridmap.sh abs_path_input_file abs_path_output_folder [run_rviz]
```
Just kill it and it will save a **.png**

## Experiments Real Robot:
**INIT**
```
$ roslaunch smb smb.launch
```
**VISUALIZE**
```
$ roslaunch smb_opc opc.launch
```

**LOCALIZE** 
```
$ roslaunch smb_slam localization.launch
```
**NAVIGATE (with existing map)**

Change *global_map* param to the path of the costmap 
```
$ roslaunch smb_navigation navigate2d_ompl.launch use_global_map:=true
```


