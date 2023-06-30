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

### running the node

You can run the nodes both in simulation and real environment:

#### real
Of course you will have to communicate with the companion computer which is onboard your UAV, in our case we have an ubiquiti based local network and we connect to the onboard computer with ssh protocol:
```
# VTOL Deltaquad
ssh grvc@10.42.0.41 #pswd: grvc1234
```

Once we are into our onboard computer we execute the following nodes:

```
# The mission node
roslaunch aerialcore_onboard_px4 atlas.launch
```

And in another terminal:
```
roslaunch onboard_dji multimaster.launch
```
This last launch will allow us to connect the onboard computer with the GCS computer through the local network

#### simulation
At first you have to install the PX4 firmware inside your GCS computer as is explained in [PX4 official website](https://docs.px4.io/main/en/dev_setup/building_px4.html).

Then you can install QgroundControl if you have not another GUI following the steps they provide in [QGroundControl installation](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html).


When you finish this last two steps, you can run the following commands in different terminals:

```
#Stablishment of take off point (ATLAS Flight Centre in Villacarrillo)
export PX4_HOME_LAT=38.13931349915096
export PX4_HOME_LON=-3.173436419425258 
export PX4_HOME_ALT=445

# Launching the simulation with gazebo in old version
make px4_sitl_default gazebo_standard_vtol

# if you have just downloaded the firmware use this instead
make px4_sitl_default gazebo-classic_standard_vtol 
```
Las process will run the simulator and the Software in the loop, that will allow you to connect the simulated aircraft with the QGroundControl once you launch a mavros node with the proper ports opened.



If your are using our whole system,  to connect the UAV with out GCS we have several additional steps:

```
roslaunch aerialcore_gui connect_uas.launch
```
In order to run our GUI:
```
# we use 18 version of node.js
nvm use 18
# run the gui
npm run server
```

Finally we launch the mavros connection node in order to have ros functionalities in the simulated vehicle:

```
roslaunch aerialcore_onboard_px4 px4-simulation.launch
```
