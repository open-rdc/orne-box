# orne-box
Platform hardware for autonomous robot

* [Purpose](https://github.com/open-rdc/orne_box/wiki/Initial-Purpose)
* [Design Data (Under Constructing)](https://drive.google.com/drive/folders/1FTzKjHyfmug_UDPVUtk7wh9Z_zvEPqiV?usp=sharing)
* This project is derived from [orne_navigation](https://github.com/open-rdc/orne_navigation).

![image](https://user-images.githubusercontent.com/5755200/76318342-eb89c780-6320-11ea-900b-02a052fb53ae.png)

![DSC_0245](https://user-images.githubusercontent.com/5755200/80554308-b0923f00-8a07-11ea-80c8-d2e2097a1d2a.jpg)

# Reference
* [orne-x](https://drive.google.com/drive/folders/1ViINGsmbruIFg-iK9aN-tVQHTLGuMvhR?usp=sharing) (designed in 2017)

# Install
```
git clone https://github.com/open-rdc/orne-box.git -b humble-devel
```
* sim利用だけであれば,pacakge.xmlとCmake.txtからypspurに関する記述をコメントアウトしてください //
wstoolで必要なリポジトリをクローンします //
* sim利用の場合
```
wstool init
wstool merge orne-box/orne_box3_simulation_pkgs.install
wstool up
```
* 実ロボットの場合
後日記載 //
rosdepで依存パッケージをインストールします
```
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```
その他で必要なパッケージをインストール
```
sudo apt install -y ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup ros-$ROS_DISTRO-turtlebot3-gazebo ros-$ROS_DISTRO-robot-localization
```
# simlation環境
simの起動
```
ros2 launch orne_box_simulation box_cit3f.launch.py
```
robot_localizationの起動
```
ros2 launch orne_box_bringup robot_localization_ekf.launch.py
```
navの起動
```
ros2 launch orne_box_navigation_executor play_waypoints_nav.launch
```

# ロボット起動(実環境)
bringup
```
ros2 launch orne_box_bringup orne_box_bringup.launch.py
```
imuのキャリブレーション（後日記載） //
navの起動
```
ros2 launch orne_box_navigation_executor play_waypoints_nav.launch
```


