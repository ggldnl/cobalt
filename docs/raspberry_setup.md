## Install ROS Melodic on the Raspberry Pi Zero W

I followed the [official guide](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Melodic%20on%20the%20Raspberry%20Pi) but some steps didn't work out for me, so I'll post what I did after a bit of searching.

Add the official ROS Debian repo to the OS:
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-melodic.list'

# from the official guide, this yiels 'no keyserver available'
# sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

sudo apt update && sudo apt upgrade # will ask you to confirm
```

Install Bootstrap Dependencies:
```bash
sudo apt install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake libyaml-cpp-dev
```

Initialize rosdep:
```bash
sudo rosdep init
rosdep update
```
Download and build ROS Melodic:
```bash
# create a catkin workspace
mkdir -p ~/catkin_ws
cd ~/catkin_ws

# fetch the ross_comm package, no gui tool -> produces the rosinstall yaml
rosinstall_generator ros_comm --rosdistro melodic --deps --wet-only --tar > melodic-ros_comm-wet.rosinstall

# you can install more than one package:
# rosinstall_generator ros_comm std_msgs std_srvs geometry_msgs eigen_stl_containers sensor_msgs tf nav_msgs --rosdistro kinetic --deps --wet-only --exclude collada_parser collada_urdf --tar > kinetic-ros_comm-wet.rosinstall

# wstool init [TARGET_PATH [SOURCE_PATH]] 
# reads SOURCE_PATH looking for a rosinstall yaml
wstool init -j1 src melodic-ros_comm-wet.rosinstall

# -j -> How many parallel threads to use for installing
#       Had problems using more than 1
```
We need to specify `-j1` to use one thread to build; the RPi is not so powerful and with the default `-j4` we can cause overhead. It failed several times without `-j1`.
If `wstool init` fails or is interrupted, you can resume the download by running `wstool update -j1 -t src`.

Before we can build the catkin workspace, we need to make sure that we have all the required dependencies:
```bash
rosdep install -y --from-paths src --ignore-src --rosdistro melodic -r --os=debian:buster
```

If some package fails to install for some reason, repeat the process. In my case, `python-nose` failded:
```bash
sudo -H apt-get install -y python-nose
```

Build the catkin packages:
```bash
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic -j1
```

Source the `setup.bash` in the `~/.bashrc`, so that ROS environment variables are automatically added to the bash session every time a new shell is launched:
```bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
````

Add the variables:
```bash
# this macro returns the ip addres (requires net-tools, should be alraedy installed)
echo 'export ROS_IP=$(ifconfig wlan0|sed -n "s/^ *inet [^0-9.]*\([0-9.]*\) .*$/\1/p")' >> ~/.bashrc

# master
echo "export ROS_MASTER_IP=<master node IP>" >> ~/.bashrc
echo "export ROS_MASTER_PORT=11311" >> ~/.bashrc

# slave
echo 'export ROS_MASTER_URI=http://"${ROS_MASTER_IP}":"${ROS_MASTER_PORT}"' >> ~/.bashrc
echo 'export ROS_HOSTNAME="${ROS_IP}"' >> ~/.bashrc

```

After building the whole workspace and all the bundled packages, we will only need to build our package if we make changes to it. I added a mini function to my bashrc that allows me to build only the selected package.

```bash
echo 'function build_package { previous_dir=$(pwd); roscd $1; cd ../..; catkin_make -DCATKIN_WHITELIST_PACKAGES="$1"; cd $previous_dir;}' >> ~/.bashrc
echo "export -f build_package" >> ~/.bashrc
```

Keep in mind that doing so we are setting an environment variable `-DCATKIN_WHITELIST_PACKAGES=...` and if for some reason we want to build again the entire workspace we need to address it.

After sourcing the bashrc file you can `build_package <pacakage>` and it will build only this package and go back to the directory where you were at.