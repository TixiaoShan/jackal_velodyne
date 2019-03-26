![](/launch/pic/demo.gif)


# Simulate Jackal and Velodyne in Gazebo


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

1. Put new world file to `worlds` folder
2. Change `run.launch` file to use the new world

## Upgrade Gazebo

```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
sudo apt upgrade
```

## Teleop Jackal

1. Install teleop package: `sudo apt-get install ros-kinetic-teleop-twist-keyboard`
2. Run: `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`
