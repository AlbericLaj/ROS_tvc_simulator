# Installation
*These steps have been tested on Ubuntu 18.04, making ROS Melodic work on newer versions of ubuntu might be possible but is a little bit trickier*

After installing Ubuntu 18.04:
1. Install ROS Melodic ([Official documentation](wiki.ros.org/melodic/Installation/Ubuntu))
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-melodic-desktop-full
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc 
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo rosdep init
rosdep update
```
2. Install Eigen
Download eigen version >3.4, open the folder in a terminal
```bash
sudo cp -r -t /usr/local/include/ Eigen/ unsupported/
```
3. Install [OSQP](https://osqp.org/)
```bash
git clone --recursive https://github.com/oxfordcontrol/osqp
cd osqp
mkdir build
cd build
cmake -G "Unix Makefiles" ..
sudo cmake --build . --target install
```
4. Install [OSQP-eigen](https://github.com/robotology/osqp-eigen)
```bash
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen
mkdir build && cd build
cmake ../
make
sudo make install
export OsqpEigen_DIR=/path/where/you/installed/
```

5. You may need a cmake version >3.12 for "add_compile_definitions()". (Check with cmake --version). To install the latest version:
   Download the latest version from [here](https://cmake.org/download/) and copy the script in /opt/.
```bash
cd /opt
sudo chmod +x cmake-3.*your_version*.sh
sudo bash cmake-3.*your_version.sh*
sudo ln -s cmake-3.*your_version*/bin/* /usr/local/bin
```

6. Install python dependencies:
```bash
sudo apt install python3-pip
pip3 install rospkg dataclasses scipy numpy pyqtgraph
```

# 
add at the end of your .bashrc file:
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.sh

to run across multiple machin, also add:
export ROS_IP=*your local ip*
export ROS_MASTER_URI=http://*your local ip*:11311


