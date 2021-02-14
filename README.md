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
```bash
git clone https://gitlab.com/libeigen/eigen.git
sudo cp -r -t /usr/local/include/ eigen/Eigen/ eigen/unsupported/
```
3. Install python dependancies:
```bash
sudo apt install python3-pip
pip3 install rospkg dataclasses scipy numpy 
```
