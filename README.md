# **Cheatsheet**

# Installing Software
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

**LOCALIZATION** 
```
$ roslaunch smb_slam localization.launch
```
**SLAM**
```
$ roslaunch smb_msf_graph smb_msf_graph.launch launch_o3d_slam:=false
```
**NAVIGATE (with existing map)**

Change *global_map* param to the path of the costmap 
```
$ roslaunch smb_navigation navigate2d_ompl.launch global_frame:=world_graph_msf odom_topic:=/graph_msf/est_odometry_odom_imu global_map:= ??
```




