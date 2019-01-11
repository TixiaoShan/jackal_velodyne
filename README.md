# Jackal and Velodyne in Gazebo


## Dependency

```
sudo apt-get install ros-kinetic-jackal-*
sudo apt-get install ros-kinetic-velodyne-*
```

## Run


```
roslaunch jackal_velodyne run.launch
```

## Sensor Configuration

Change sensor settings at the bottom of `jackal_velodyne.urdf.xacro`

## New Worlds

1. Put new world file to worlds folder
2. Change `run.launch` file to use the new world