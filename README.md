# ROS Visualization Node for Continuum Soft Arm
This repository collects all the materials required to build a ROS Visualization Node for Continuum Soft Arm in RViz.

![Flowchat of the Project](/docs/flowchart.svg)


## Examples
1) ROS Bag with 100 `/tf` and CDCR visualization:
```bash
roslaunch shape_rviz shape_rviz100_cdcr.launch
```

2) ROS Bag with 100 `/tf` and TDCR visualization:
```bash
roslaunch shape_rviz shape_rviz100_tdcr.launch
```

2) ROS Bag with 7 `/tf` and TDCR visualization:
```bash
roslaunch shape_rviz shape_rviz7_tdcr.launch
```

## Parameters

There are parameters configurable in the ROS params (there are predefined values included in the example launch files)

**TDCR** and **CDCR**:
- `arm_type`: defines the type of arm to be represented (`tdcr` or `cdcr`)
- `sections`: defines the different sections of the arm e.g `[10, 20]` (will be represented by a different color)
- `scale`: scale of the arm
- `total_frames`: amount of backbone frames (not counting the base and tip)

**TDCR** only:
- `density`: density of circular spacers (values between `]0-1]`)



