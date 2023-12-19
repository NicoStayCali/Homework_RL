# 2D navigation stack

This repository provides launch files and nodes that use ROS Navigation Stack.

## Dependencies
Before use it install the following dependencies:

1 - Navigation stack
```bash
sudo apt-get install ros-<DISTRO>-navigation
  ```

2 - Teb Local Planner
``` bash
sudo apt-get install ros-<DISTRO>-teb-local-planner
```
3 - Move Base
``` bash
sudo apt-get install ros-<DISTRO>-move-base
```

## Use in simulation
Launch first the launch file to bring-up sensors (lidar), motor drives, tf and Gmapping
```bash
roslaunch fra2mo_2dnav move_base.launch 
```
This launch file is used also to load all the parameters for the costmap (local and global), Teb local planner and Move Base.
To send a simple goal to the rover use:
```bash
rosrun fra2mo_2dnav single_goal
```
