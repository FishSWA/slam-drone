## 项目简介
一个使用VINS定位，使用IPC导航和避障的自主无人机的上位机程序

暂时在ROS1下构建，整体跑通后计划迁移到ROS2上

## 使用方法
安装ROS
``` bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full    #带有调试工具的完整版
sudo apt install ros-noetic-ros-base        #或者 - 基础ROS包

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
创建工作空间
``` bash
mkdir -p ~/catkin_ws/src
```
下载源码 & 编译
``` bash
cd ~/catkin_ws/src
git clone https://github.com/FishSWA/slam-drone
cd ~/catkin_ws
catkin_make
```