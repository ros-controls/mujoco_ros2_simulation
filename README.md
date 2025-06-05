# MuJoCo ROS 2 Simulation

This package contains a ROS 2 control system interface for the [MuJoCo Simulator](https://mujoco.readthedocs.io/en/3.3.2/overview.html).

The system interface wraps MuJoCo's [Simulate App](https://github.com/google-deepmind/mujoco/tree/3.3.2/simulate) to provide their included functionality.
Because the app is not bundled as a library, we compile it directly from a local install of MuJoCo.

This library is based on the MoveIt [mujoco_ros2_control](https://github.com/moveit/mujoco_ros2_control) package.

## Installation

This interface has only been tested against ROS 2 humble and MuJoCo `3.3.2`.
We assume all required ROS dependencies have been installed either manually or with `rosdep`.

A local install of MuJoCo is required to build the application.
Refer to their documentation for installation procedures.

Once installed, set the following environment variables:

```bash
MUJOCO_VERSION=3.3.2
MUJOCO_DIR=/opt/mujoco/mujoco-3.3.2
```

From there the library can be compiled with `colcon build ...`, as normal.

## Configuration

TODO
