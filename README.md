# onboard_px4
GRVC repository including the software developed for the operation of px4-based UAVs.

*NOTE: The files in this repository are particulary designed for a raspberri Pi 4, with ubuntu 20.04 and ROS Noetic release. It's not a problem if you have ubuntu 18.04 and ROS Melodic, because only few changes are needed.* 

## Dependencies
This package depends on the following packages:
- [grvc-ual](https://github.com/grvcTeam/grvc-ual): an abstraction layer for many packages such as mavros, px4, dji_ros, unreal engine ... Here relevant ones are mavros and px4.
- [grvc-utils](https://github.com/grvcTeam/grvc-utils): Auxiliary tools and utils for real experiments and simulations. The tool needed is mission_lib.
- [multimaster_fkie](http://fkie.github.io/multimaster_fkie/): Run the following commands:<br>
```
# ROS melodic
sudo apt install ros-melodic-multimaster-fkie
## or only these two:
sudo apt install ros-melodic-master-sync-fkie
sudo apt install ros-melodic-master-discovery-fkie

# ROS noetic
sudo apt install ros-noetic-fkie-multimaster
## or only these two:
sudo apt install ros-noetic-fkie-master-sync
sudo apt install ros-noetic-fkie-master-discovery

```
You should take into account the names of the multimaster packages used in multimaster.launch.

The URLs includes the whole information for installation and running.

## Usage
The purpose is to install this package in a companion computer with pixhawk controllers, the controller will run PX4 in this case. So set-up your companion computer as explained in [Using a Companion Computer with Pixhawk Controllers](https://docs.px4.io/main/en/companion_computer/pixhawk_companion.html), then install the dependencies and finally clone this repository in your workspace and do catkin_make.

