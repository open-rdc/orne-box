# waypoint_nav
This program enables you to execute waypoint navigation.

## Feature
- infinite loop
- enroling functions in yaml

## Usage
### Build
```
$ mkdir -p ~/ws/src
$ cd ~/ws/src
$ git clone 
$ cd ../
$ catkin_make
```

### Run
```
$ source ~/ws/devel/setup.bash
$ roslaunch wayopint_nav run.launch
$ rosservice call /start_wp_nav "{}"
```

## Settings
### ROS Param
For detail, see [run.launch](https://github.com/tiger0421/waypoint_nav/blob/main/launch/run.launch)
- robot_frame  
  ex) base_link

- world_frame  
  ex) map

- max_update_rate  
  ex) 1.0

- filename  
  Set your path to yaml

- dist_err  
  Regarding your robot as arriving at the target waypoint

- loop_flg  
  If this flg is true, do all over again after arriving at last waypoint

- wait_time  
  This is a time which your robot waits for moving  
  Send a waypoint again, if your robot doesn't move for this time

- resend_thresh  
  This is a threshold which how many times to resend the waypoint  
  Skip the waypoint if this threshold is over

### Yaml Setting
```
waypoints:
  - point:
    x:
    y:
    z:
    function:
```
You can set some functions if you add function to source program.  
This program has two functions, "run" and "suspend".  
However, "function" is not compulsory.  
See this [sample.yaml](https://github.com/tiger0421/waypoint_nav/blob/main/config/sample.yaml)

- run  
  This is normal mode.  
  When your robot gets at target waypoint, makes for next waypoint.

- suspend  
  This makes your robot suspend mode after moving.

## Suspend Mode
When you want a robot to start or resume navigation, call `rosservice` like below.  
`
$ rosservice call /start_wp_nav "{}"
`

When you want a robot to set suspend mode, call `rosservice` like below.  
`
$ rosservice call /suspend_wp_nav "{}"
`

## Movie
- loop version  
https://youtu.be/5eXg1V7NM3A

- an obstacle on a waypoint  
https://youtu.be/TDLN0mZlM6w
