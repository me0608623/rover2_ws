# campusrover_navigation

A 2D navigation stack that takes in information from odometry, sensor streams, and a goal pose and outputs safe velocity commands that are sent to a mobile base.

### Usage
runing code without any obstacle avoidance function ex: over take and pullover
```
roslaunch campusrover_move path_following.launch 
```
runing code with over take function
```
roslaunch campusrover_move path_following.launch enble_dwa_obstacle_avoidance:=true
```
runing code with pullover function
```
roslaunch campusrover_move path_following.launch enble_pullover_mode:=true
```
---
## path_following
the local planner is currently attempting to follow specific path.

use PlannerFunction.srv to enable or disable path following node
#### PlannerFunction.srv
```
uint8 MODE_GLOBAL_PATH = 1
uint8 MODE_ELEVATOR_PATH = 2
uint8 MODE_BUTTON_PARKING = 3
uint8 MODE_PULLOVER_PATH = 4
std_msgs/Bool action
std_msgs/Bool direction_inverse
std_msgs/Bool obstacle_avoidance
uint8 mode
geometry_msgs/Twist speed_parameter
---

```	
- "action": to enable or disable path following node
- "direction_inverse": false:foeward, true:backward
- "obstacle_avoidance": enable costmap or costom obstacle input.
- "speed_parameter": setting max velocity parameter
### Subscribed topics
- `/global_path` ([nav_msgs/Path](http://docs.ros.org/en/lunar/api/nav_msgs/html/msg/Path.html))
	- Global plan that the local planner is currently attempting to follow.

- `/elevator_path` ([nav_msgs/Path](http://docs.ros.org/en/lunar/api/nav_msgs/html/msg/Path.html))
	- Elevator plan that the local planner is currently attempting to follow.

- `/costmap` ([nav_msgs/OccupancyGrid](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html))
	- 2D occupancy grid of the data that repersent the sensors data.

- `/odom` ([nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html))
	- Odometry information that gives the local planner the current speed of the robot

- `/obstacles` ([costmap_converter/ObstacleArrayMsg](http://docs.ros.org/en/lunar/api/costmap_converter/html/msg/ObstacleArrayMsg.html))
	- Provide custom obstacles that the local planner need to avoid collision .
  
### Published topics
- `/cmd_vel` ([geometry_msgs::Twist](http://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/Twist.html)
	- output velocity command for driver.
- `/reach_goal` ([std_msgs::Empty](http://docs.ros.org/en/lunar/api/std_msgs/html/msg/Empty.html)
	- Msgs for demo code when planner finish task.
  
### Parameters
The parameters are listed in alphabetical order.

- `robot_frame` (default: "base_link")
	- The frame in which the robot's position.

- `arriving_range_dis` (default: 0.2)
	- The iterations of target point range.

- `arriving_range_angle` (default: 0.1)
	- The iterations of target direction range.

- `target_point_dis` (default: 0.5)
	- The point which in following path distance between robot and target point.

- `threshold_occupied` (default: 10)
	- The value define as obstacle when costmap value over that.

- `obstacle_range` (default: 0.3)
	- The value represent obstacle radius.

- `obstacle_detect_max_dis` (default: 1.5)
	- The maximum distance of obstacle forward search on following path.

- `obstacle_detect_min_dis` (default: 0.8)
	- The forward minimum distance of obstacle need to stop.

- `back_obstacle_detect_max_dis` (default: 2.3)
	- The distance that need to search back obsatcle on following path.

- `speed_pid_k` (default: 0.06)
	- The frame in which the robot's position.

- `min_angle_of_linear_profile` (default: 0.1)
	- The frame in which the robot's position.

- `max_angle_of_linear_profile` (default: 0.5)
	- The frame in which the robot's position.

- `enble_costmap_obstacle` (default: false)
	- The frame in which the robot's position.

- `enble_dwa_obstacle_avoidance` (default: false)
	- The frame in which the robot's position.

- `enble_pullover_mode` (default: false)
	- The frame in which the robot's position.

- `enable_linear_depend_angular` (default: false)
	- The frame in which the robot's position.


---
## dwa_planner

---
## pullover_path_planner
