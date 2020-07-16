Generate an Octomap from a Gazebo world
=======================================

[Reference](https://github.com/OctoMap/octomap/wiki/Importing-Data-into-OctoMap)

## Installation

Download [binvox](https://www.patrickmin.com/binvox/)
to convert `.obj` files to voxels representation `.binvox`.
By default, we install the `binvox` executable in `$HOME`.
```shell script
wget http://www.patrickmin.com/binvox/linux64/binvox?rnd=1520896952313989 -O ~/binvox
chmod 755 binvox
```

Download [binvox2bt](https://manpages.ubuntu.com/manpages/bionic/man1/binvox2bt.1.html)
to convert `.binvox` files to octomap representation.
```shell script
sudo apt install octomap-tools
```

Download [Blender](https://www.blender.org/) to convert `.dae` to `.obj`.
```shell script
sudo apt install blender
```

## How to use

To convert a Gazebo world to an Octomap usable by the Octomap ROS node,
use the `world2oct.sh` script. For example, using the PX4 toolchain:
```shell script
./world2oct.sh ~/Firmware/Tools/sitl_gazebo/worlds/warehouse.world ~/warehouse.bt
```

## Workflow

Combine all DAE files with a python script

DAE -> OBJ with Blender

OBJ -> binvox with binvox
```shell script
./binvox -e -fit file.obj
```

binvox -> oct tree with binvox2bt
```shell script
binvox2bt --mark-free file.binvox
```

To visualize the final output, with octovis.
```shell script
octovis file.binvox.bt
```
