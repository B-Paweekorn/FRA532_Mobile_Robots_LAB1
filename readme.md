# LAB 1: Local Planner

This project aims to develop a local planner and controller while minimizing the reliance on nav2 as much as possible. In this lab, we compare three solutions for robot navigation:

1.) nav2 + pure pursuit

2.) nav2 + pure pursuit + virtual force field

3.) potential field + pure pursuit (without nav2)

#### This project is part of FRA532 Mobile Robots at the @ Institute of Field Robotics, King Mongkut’s University of Technology Thonburi.
<br>

## Changing differential drive controller (diff_cont) to joint group velocity controller (velocity_controllers)
1. In `simulation.launch.py`, the launch configuration for the differential drive controller is changed to velocity controller.
```py
# controller = Node(
#   package="controller_manager",
#   executable="spawner",
#   arguments=["diff_cont", "-c", "/controller_manager"],
#)
controller = Node(
  package="controller_manager",
  executable="spawner",
  arguments=["velocity_controllers", "-c", "/controller_manager"],
)
```
2. A new node `velocity_controller.py` is created. It subscribes to `/cmd_vel`, compute inverse kinematics for differential drive robot, then publish the joint speed to the JointGroupVelocityController at `/velocity_controllers/commands`. The node also publish the wheel speed to `feedback_wheelspeed`.
3. Another new node `odom_publisher.py` subscribes to `/feedback_wheelspeed`, compute forward kinematics and publish to `/odom` and `/tf`.

<br>

## Local planner pseudocode

- #### Pseudocode: Dynamic Pure Pursuit

  ```py
  # - The controller should start when point click is sent and stop when robot is at goal.
  # - The controller will continuously request new path as long as it is active
  # - When new path comes, the index is reset
  # - The controller will default to the nearest point in the path if its index is higher than the current one,
  #   this prevent the robot from running back to the start of the path if there are delay in global planner.
  #   It also allow the robot to come back to the path if it somehow temporily went off-track.
  
  path = None
  controller_enabled = 0
  path_index = 0
  
  callback(goal given by point click):
    controller_enabled = 1
  callback(path received):
    path = msg.path
    path_index = 0

  def calculate_goal_point(path_index, path, robot_pose){
    return (furthest path index within lookahead distanch)
  }
  def calculate_velocity(robot_pose, goal){
    return (pure pursuit inverse kinematic of differential drive robot)
  }
  
  void loop():
    if (controller_enabled):
      if (robot at goal):
        controller_enabled = 0
        return
      if (no path request in queue):
        request path to final goal point from global planner
      path_index = (index of path point that is closest to the robot)
      calculate_goal_point()
      calculate_velocity()
  ```

- #### Pseudocode: Virtual Force Field (VFF)

  PLACEHOLDER

- #### Pseudocode: Potential Field Global Planner (extra)

  PLACEHOLDER

<br>

## Lab experiment design
PLACEHOLDER

<br>

## Coding
The code can be found here

- [Dynamic Pure Pursuit](https://github.com/Nopparuj-an/FRA532_Mobile_Robots_LAB1/blob/master/src/robot_control/scripts/purepursuit_dynamic.py)
- [Virtual Force Field](https://github.com/Nopparuj-an/FRA532_Mobile_Robots_LAB1/blob/nav2%2Bpurepursuit%2BVFF/src/robot_control/scripts/vff_avoidance_from_purepursuit.py)
- [Potential Field Global Planner](https://github.com/Nopparuj-an/FRA532_Mobile_Robots_LAB1/blob/potentialfield%2Bpurepursuit/src/robot_control/scripts/global_planner.py)

<br>

## Simulation implementation
#### 1. Switch to one of these branch

- [**nav2+purepursuit**](https://github.com/Nopparuj-an/FRA532_Mobile_Robots_LAB1/tree/nav2%2Bpurepursuit) Nav2 global planner with pure pursuit local planner
- [**nav2+purepursuit+VFF**](https://github.com/Nopparuj-an/FRA532_Mobile_Robots_LAB1/tree/nav2%2Bpurepursuit%2BVFF) Nav2 global planner with pure pursuit and virtual force field local planner
- [**potentialfield+purepursuit**](https://github.com/Nopparuj-an/FRA532_Mobile_Robots_LAB1/tree/nav2%2Bpurepursuit) Potential field global planner with pure pursuit local planner

#### 2. While you are in the workspace, run the following command

```bash
source install/setup.bash
colcon build
ros2 launch robot_control main.launch.py
```

#### 3. Use the `Publish Point` tool to mark where the robot should go

<br>

## Conclusions and analysis
PLACEHOLDER
