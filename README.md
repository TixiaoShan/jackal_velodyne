![](/launch/pic/demo.gif)


# Simulate Jackal and Velodyne in Gazebo


## Dependency

```
sudo apt-get install ros-kinetic-jackal-*
sudo apt-get install ros-kinetic-velodyne-*
```

## Compile

You can use the following commands to download and compile the package.

```
cd ~/catkin_ws/src
git clone https://github.com/TixiaoShan/jackal_velodyne.git
cd ..
catkin_make
```

## Run

```
roslaunch jackal_velodyne run.launch
```

## Sensor Configuration

Change sensor settings at the bottom of `jackal_velodyne.urdf.xacro`

```
parent="base_link"
name="velodyne"
topic="/velodyne_points"
hz="10"
samples="1800"
gpu="false"
```

## New Worlds

1. Put new world file to `worlds` folder
2. Change `run.launch` file to use the new world

## Upgrade Gazebo

It's recommended that upgrade your Gazebo to the latest version, which has great performance improvements.

```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
sudo apt upgrade
```

## Teleop Jackal

1. Install teleop package: `sudo apt-get install ros-kinetic-teleop-twist-keyboard`
2. Run: `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

## Gmapping

1. Install: `sudo apt-get install ros-kinetic-slam-gmapping`
2. Run: `roslaunch jackal_velodyne gmapping_demo.launch`
3. Set the `2D Nav Goal` in Rviz
