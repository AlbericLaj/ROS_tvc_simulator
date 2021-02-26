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
3. Install [OSQP](https://osqp.org/)
```bash
git clone https://github.com/oxfordcontrol/osqp
cd osqp
mkdir build
cd build
cmake -G "Unix Makefiles" ..
cmake --build . --target install
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

5. You may need a cmake version >3.12. (Check with cmake --version). To install the latest version:
   Download the latest version from [here](https://cmake.org/download/) and copy the script in /opt/.
```bash
chmod +x /opt/cmake-3.*your_version*.sh
sudo bash /opt/cmake-3.*your_version.sh*
sudo ln -s /opt/cmake-3.*your_version*/bin/* /usr/local/bin
```

6. Install python dependencies:
```bash
sudo apt install python3-pip
pip3 install rospkg dataclasses scipy numpy pyqtgraph
```
