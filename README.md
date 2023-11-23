# orne-box
Platform hardware and software for autonomous robot

* [Purpose](https://github.com/open-rdc/orne_box/wiki/Initial-Purpose)
* This project is derived from [orne_navigation](https://github.com/open-rdc/orne_navigation).

## Software
### Install
Precondition:  
1) Install ROS noetic (Ubuntu20.04/docker)  
2) Install [python3-catkin-tools](https://github.com/open-rdc/orne-box/issues/79#issuecomment-1818041979)  

```
cd ~/catkin_ws/src
git clone -b noetic-devel https://github.com/open-rdc/orne-box
wstool init
wstool merge orne-box/orne_box_pkgs.install
wstool up
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
cd ~/catkin_ws
catkin build
source ~/.bashrc
```

### Excecution

On simulator (gazebo)  
```
roslaunch orne_box_bringup orne_box_sim.launch
roslaunch orne_box_navigation_executor nav_static_map.launch
```

[![IMAGE](http://img.youtube.com/vi/HwTbgvv611k/0.jpg)](https://youtu.be/HwTbgvv611k)

On real robot  
Under constructing  

## Hardware
* [Design Data (Under Constructing)](https://drive.google.com/drive/folders/1FTzKjHyfmug_UDPVUtk7wh9Z_zvEPqiV?usp=sharing)

![image](https://user-images.githubusercontent.com/5755200/76318342-eb89c780-6320-11ea-900b-02a052fb53ae.png)

![DSC_0245](https://user-images.githubusercontent.com/5755200/80554308-b0923f00-8a07-11ea-80c8-d2e2097a1d2a.jpg)

### Reference
* [orne-x](https://drive.google.com/drive/folders/1ViINGsmbruIFg-iK9aN-tVQHTLGuMvhR?usp=sharing) (designed in 2017)

## Paper
1) 井口 颯人、樋高 聖人、石江 義規、上田 隆一、林原 靖男，”屋外自律移動ロボットプラットフォーム ORNE-box の開発”，3H2-03，SI2021(2021)
2) 井口颯人，樋高聖人，野村駿斗，村林孝太郎，上田隆一，林原靖男，"屋外自律移動ロボットプラットフォームORNE-box の開発"，日本機械学会ロボティクス・メカトロニクス講演会'23予稿集，1P1-I06(2023)
