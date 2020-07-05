PX4 autopilot
=============

Python autopilot using the MAVLink protocol.

## Installation

Install the [PX4 toolchain](https://dev.px4.io/master/en/) and python 3.

## Run the simulation

Launch PX4, using the Gazebo simulation. We use the quadcopter **iris** and the
map **warehouse**.
The maps are stored in the PX4 toolchain in `Firmware/Tools/sitl_gazebo/worlds/`.
The Gazebo models and UAVs used in the simulation are in `Firmware/Tools/sitl_gazebo/models/`.
**Do not forget the double `_` !**

```shell
make px4_sitl_default gazebo_iris__warehouse
```

This compiles the toolchain and launches a Gazebo simulation with a world and a UAV.


## Run the autopilot

```shell
python main.py -p <planning_algorithm>
```

`<planning_algorithm>` can be:
- `a*`: A*
- `rrt`: RRT
- `rrt*`: RRT*
- `dummy`: Dummy planning algorithm, for testing purposes
