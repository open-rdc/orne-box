# icart_mini_drive
このリポジトリはypspurのROS 2ラッパです.  


ステート：devel  

# Subscribe topic
- `cmd_vel` ([geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html))
  - joy_conやNav2 controller serverなどから生成されるgeometry_msgs/Twist形式のトピックです．  
  サブスクライブしたこのトピックを，速度指令としてyppsurへ送信します．

# Publish topic
- `odom` ([nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html))
  - ロボットの移動量から求めた，ホイールオドメトリを提供します．現在は2つの計算方式が選択できます． 
    
    １．ypspurの関数を利用する方法
     - ypspurが提供する現在位置取得コマンドを用いて，オドメトリを計算します．  
     エンコーダの値などの実測値を用いるため，手押しした場合でもオドメトリを取得することが可能です．
   
    ２．cmd_velから計算する方法
      - サブスクライブしたcmd_velを用いて，オドメトリを計算します．  
     エンコーダの値を用いず，指令値から直接計算するため，正確ではない計算方法です．
- `tf` ([tf/tfMessage](http://docs.ros.org/en/api/tf/html/msg/tfMessage.html))
  - odomからロボットのbese_frame_idへのtfを提供します．  
    例：odom → base_footprint
- `joint_states` ([sensor_msgs/JointState](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/JointState.html))
  - ypspurから取得した，左右輪の角度，角速度を提供します． 
# Parameters
- `odom_frame_id`(string default: odom)
  - odomトピック，tfトピックで用いるオドメトリ座標系のフレームIDを設定します
- `base_frame_id`(string default: base_footprint)
  - odomトピック，tfトピックで用いるロボットの基準フレームIDを設定します  
  base_footprintやbase_linkなどを設定してください
- `Hz`(int default:100)
  - ノードのループ周期を設定します． 
- `left_wheel_joint`(string default: left_wheel_joint)
  - joint_statesトピックで使用する左車輪の名前を設定します 
- `right_wheel_joint`(string default: right_wheel_joint)
  - joint_statesトピックで使用する右車輪の名前を設定します 
- `liner_vel_lim`(double default: 1.5)
  - ロボットの速度の上限を設定します．  
  config上で設定したypspurのparamと同じ値にすることを推奨します．
- `liner_accel_lim`(double default: 1.5)
  - ロボットの加速度の上限を設定します．  
  config上で設定したypspurのparamと同じ値にすることを推奨します．
- `angular_vel_lim`(double default: 3.14)
  - ロボットの角速度の上限を設定します．  
  config上で設定したypspurのparamと同じ値にすることを推奨します．
- `angular_accel_lim`(double default: 3.14)
  - ロボットの角加速度の上限を設定します．  
  config上で設定したypspurのparamと同じ値にすることを推奨します．
- `calculate_odom_from_ypspur`(bool default: true)
  - odomの計算方法を設定します
    - true: ypspurの関数と実測値を用いて，odomを計算します
    - false: cmd_velからodomを計算します
# Install
```
git clone https://github.com/haruyama8940/icart_mini_driver
git clone https://github.com/openspur/yp-spur
git clone https://github.com/ros2/teleop_twist_joy
```
# Build
```
cd install_your_workspace
colcon build --symlink-install
```
# Run
```
ros2 launch icart_mini_driver icart_mini_bringup_launch.py
```
# Notes
- odom，tf，joint_stateではモータの取り付けの向きによって，座標を調整する必要があります．
rvizなどで可視化しながら，進行方向と一致するように調整してください．
(後にパラメータ化します)
- scripts内に存在するypspur_coordinator_bridgeに関して，接続しているデバイス(ttyACM0 or icart-mini)を適宜選択する必要があります．

