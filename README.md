# orne-box
Platform hardware for autonomous robot.

- [Purpose](https://github.com/open-rdc/orne_box/wiki/Initial-Purpose)
- [Design Data (Under Constructing)](https://drive.google.com/drive/folders/1FTzKjHyfmug_UDPVUtk7wh9Z_zvEPqiV?usp=sharing)
- This project is derived from [orne_navigation](https://github.com/open-rdc/orne_navigation).

![image](https://user-images.githubusercontent.com/5755200/76318342-eb89c780-6320-11ea-900b-02a052fb53ae.png)
![DSC_0245](https://user-images.githubusercontent.com/5755200/80554308-b0923f00-8a07-11ea-80c8-d2e2097a1d2a.jpg)

## Reference
- [orne-x](https://drive.google.com/drive/folders/1ViINGsmbruIFg-iK9aN-tVQHTLGuMvhR?usp=sharing) (designed in 2017)

## Install

### 1. Clone
`ros2_ws/src` で以下を実行します。

```bash
git clone https://github.com/open-rdc/orne-box.git -b humble-devel
```

### 2. Clone dependent repositories (`wstool`)
`wstool` は **必ず `src` 直下（例: `~/ros2_ws/src`）で実行** してください。

```bash
cd ~/ros2_ws/src
```

#### For simulation
```bash
wstool init
wstool merge orne-box/orne_box3_simulation_pkgs.install
wstool up
```

#### For real robot
```bash
wstool init
wstool merge orne-box/orne_box3_simulation_pkgs.install # TODO: 実機向け install ファイルは後日追記
wstool up
```

### 3. Install dependencies

```bash
sudo apt install -y \
  libpcap-dev \
  ros-$ROS_DISTRO-navigation2 \
  ros-$ROS_DISTRO-nav2-bringup \
  ros-$ROS_DISTRO-turtlebot3-gazebo \
  ros-$ROS_DISTRO-robot-localization \
  ros-$ROS_DISTRO-pointcloud-to-laserscan \
  ros-$ROS_DISTRO-xacro \
  ros-$ROS_DISTRO-joint-state-publisher \
  ros-$ROS_DISTRO-laser-filters 

```

## Simulation

### Start simulation
以下のどちらかを選択して起動してください。 
gazebo Classicで起動する場合:
```bash
ros2 launch orne_box_simulation box_cit3f.launch.py
```
Ignition gazeboで起動する場合:
```bash
ros2 launch orne_box_simulation ign_box_cit3f.launch.py
```

### Start navigation
```bash
ros2 launch orne_box_navigation_executor play_waypoints_nav.launch.py
```

## Real Robot

### Bringup
```bash
ros2 launch orne_box_bringup orne_box_bringup.launch.py
```

### Navigation
```bash
ros2 launch orne_box_navigation_executor play_waypoints_nav.launch.py
```

IMU calibration は後日追記予定です。
