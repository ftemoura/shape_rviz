# shape_rviz
This repository collects all the materials required to build a ROS Visualization Node for Continuum Soft Arm in RViz.

## Download
### Step 1: Setup Git Large File Storage (LFS)
Follow the steps in this [link](https://docs.github.com/en/repositories/working-with-files/managing-large-files/installing-git-large-file-storage).

### Step 2: Clone this Repository (SSH)
```bash
git clone git@github.com:BRAIR-Education/shape_rviz.git
```

## Run ROS Bag and RViz
1) ROS Bag with 100 `/tf` and 100 `sensor_msgs/PointCloud`:
```bash
roslaunch shape_rviz shape_rviz100.launch
```

2) ROS Bag with 7 `/tf` and no `sensor_msgs/PointCloud`:
```bash
roslaunch shape_rviz shape_rviz7.launch
```

## Description of the Project
A **Continuum Soft Arm** can be represented through a 3D oriented curve, called *backbone*. One of the common way to represent it is a set of N reference frames. In this project, the student will develop a ROS Node that represent graphically the backbone in RViz, using the `/tf` messages.

[Flowchar of the Project](/docs/flowchart.svg)

## Hints
- [Rviz Marker](http://wiki.ros.org/rviz/DisplayTypes/Marker);
- [Existing GUI in RViz for Soft Robotics](https://github.com/ContinuumRoboticsLab/CRVisToolkit).