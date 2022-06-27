# Cobalt

ROS node for a two wheeled Unmanned Ground Vehicle based on a Raspberry Pi Zero W.

Link to [PiSugar website](https://www.pisugar.com/).

Link to the [EasyEDA project](https://u.easyeda.com/account/user/projects/index/detail?project=1f5f350c7c974c4f834d326827976b48&folder=all).

Link to [cobalt_hardware repository]() containing the 3d printed frame and the files for the pcb.

## Prerequisites

ROS, obviously. To install ROS Melodic on the Raspberry Pi give a look [here](docs/raspberry_setup.md).
You may need to install [raspicam_node](https://github.com/UbiquityRobotics/raspicam_node) but this is optional; if you don't want it, edit the [launch file](launch/cobalt.launch) file and remove the sensitive part.

## Build instructions

### Build on ARM for ARM

Go to your catkin_ws cd ~/catkin_ws/src and download the sources:
```
cd ~/catkin_ws/src
git clone https://github.com/ggldnl/cobalt.git
```

Compile with `catkin_make`.

To compile ONLY this package at a later time:
```
roscd ugv
cd ../..
catkin_make -DCATKIN_WHITELIST_PACKAGES="cobalt";
```

or you can automate these steps adding the following to the slave RPi `bashrc`
```
function build_package { 
    previous_dir=$(pwd); 
    roscd $1; 
    cd ../..; 
    catkin_make -DCATKIN_WHITELIST_PACKAGES="$1"; 
    cd $previous_dir;
}
export -f build_package
```
I used this function a lot during the development. Here is the oneliner you need to run to add it:

```
echo -e 'function build_package { previous_dir=$(pwd); roscd $1; cd ../..; catkin_make -DCATKIN_WHITELIST_PACKAGES="$1"; cd $previous_dir;}\nexport -f build_package' >> ~/.bashrc
```

After sourcing it, just: `build_package cobalt`.

I usually keep the master on a separate machine that has a display, so I can get the camera feed with `rqt_image_view`.
My configuration is the following (I don't like to hardcode the variables):

`master bashrc`:
```
export CONNECTION=wlp2s0 #wlan0
export ROS_IP=$(ifconfig "${CONNECTION}"|sed -n "s/^ *inet [^0-9.]*\([0-9.]*\) .*$/\1/p") #strip away the IP
export ROS_MASTER_IP="${ROS_IP}"
export ROS_MASTER_PORT=11311
export ROS_MASTER_URI=http://"${ROS_MASTER_IP}":"${ROS_MASTER_PORT}"
export ROS_HOSTNAME="${ROS_IP}"

```

`slave (RPi) bashrc`:
```
export ROS_IP=$(ifconfig wlan0|sed -n "s/^ *inet [^0-9.]*\([0-9.]*\) .*$/\1/p")
export ROS_MASTER_IP=<master_IP>
export ROS_MASTER_PORT=11311
export ROS_MASTER_URI=http://"${ROS_MASTER_IP}":"${ROS_MASTER_PORT}"
export ROS_HOSTNAME="${ROS_IP}"
```

This way you only need to change the master IP on the slave in case you switch
network. Keep in mind that the `CONNECTION` variable may vary depending on the master machine, read [this](https://www.freedesktop.org/wiki/Software/systemd/PredictableNetworkInterfaceNames/). The same holds for the RPi, but it's unlikely that you will run it with any kind of eth cable, especially since we are dealing with an ugv.

### Build for ARM on another architecture

TODO

## Usage

Once you have the node built, you can run it either using the launch file or with `rosrun`.

### Rosrun

Simply run the node only:
`rosrun cobalt main`

### Launch file

Launch configuration:
`roslaunch cobalt cobalt.launch`

This will launch our node and the raspicam_node with a slightly different config (image flipped along horizontal axis to accomodate the 3d printed mount of the robot).
Use `rqt_image_view` on the master to view the published image.