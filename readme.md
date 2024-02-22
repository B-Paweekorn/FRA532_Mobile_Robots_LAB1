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
      publish_cmd_vel()
  ```

- #### Pseudocode: Virtual Force Field (VFF)

  ```py
  # - The VFF node receives `/cmd_vel_purepursuit` from the pure pursuit node and `/scan` from LIDAR
  # - It performs virtual force field calculation if there are object detected near the robot
  #   and then gives output to `/cmd_vel`

  callback(cmd_vel):
    cmd = msg.cmd
  callback(scan):
    scan = msg.distance

  void loop():
    min_i = argmin(scan)
    if scan[min_i] < 1.0:
      calculate angle from min_i
      intensity = 1.0 - scan[min_i]
      linear = cmd.linear.x
      angular = cmd.angular.z + (vff angle + pi)
      publish_cmd_vel(linear, angular)
  ```

- #### Pseudocode: Potential Field Global Planner (extra)
  ```py
  # Function to create a filtered map from received map data
  def create_map():
      # Receive map data
      Filter costmap to improve calculation speed
      Convert pixel units to cartesian coordinates
      Identify significant obstacles using k-means clustering
      Return filtered map
  
  # Function to calculate the vector field forces
  def vff_calc():
      Calculate attractive potential
      Calculate repulsive potential
      Combine attractive and repulsive potentials
      Assign weight values to each point
      Return potential map
  
  # Function to plan a path using potential field method
  def planner():
      Potential map = vff_calc()
      Find a path using the potential map
      Execute the planned path
  
  # Function to smoothen the planned path
  def smoother_path():
      Return a smoother path array
  
  # Main loop of the planner
  def main_loop():
      If request for a path is received:
          Plan a path using the Potential Field method
          Smooth the planned path
          Visualize the smoothed path in rviz
  
  # Main program entry point
  if __name__ == "__main__":
      Run the main loop continuously
  ```
<br>

## Potential field (Extra)
![image](https://github.com/Nopparuj-an/FRA532_Mobile_Robots_LAB1/assets/122732439/11bf6f54-df69-47e6-8501-963ebe57e135)

Template for global planner (without ROS2 action):  
### Parameter 

`self.visualize`: This setting allows you to control whether you want to see the results while editing parameters. It's recommended to set it to False if you plan to run more than one iteration to avoid unnecessary visualizations that might slow down the process.

`self.robot_x`: This represents the initial pose of the robot along the x-axis.

`self.robot_y`: This represents the initial pose of the robot along the y-axis.

`self.goal_x` : This denotes the target position along the x-axis where the robot needs to reach.

`self.goal_y`: This denotes the target position along the y-axis where the robot needs to reach.

`self.robot_radius`: This parameter defines the dimensions of the robot, which are used to create a potential map.

`self.calc_res`: This parameter represents the resolution of the calculation. Increasing it will result in smoother but slower calculations.

`self.obs_cluster`: This involves simplifying raw map data by using significant points.

`self.kattrac`: This is the gain factor for the attractive forces in the robot's navigation.

`self.krepuls`: This is the gain factor for the repulsive forces in the robot's navigation.

## Lab experiment design
### Experiment Setup:

#### Without Obstacles:

1. **Pure Pursuit (PP) Alone:**
   - The robot navigates using only the Pure Pursuit algorithm without any additional enhancements like obstacle avoidance.
   - This serves as the baseline for comparison.

2. **Pure Pursuit (PP) with Virtual Force Field (VFF):**
   - The robot uses Pure Pursuit along with a Virtual Force Field (VFF) for obstacle avoidance.
   - The VFF assists the robot in navigating around potential obstacles.

3. **Potential Field (PF) with Pure Pursuit (PP):**
   - The robot navigates using the Potential Field method in conjunction with Pure Pursuit.
   - The Potential Field method helps guide the robot by considering both attractive and repulsive forces in the environment.

#### With Obstacles:

4. **Pure Pursuit (PP) with Obstacles:**
   - The robot navigates using only the Pure Pursuit algorithm in an environment with obstacles.
   - This is to observe how well the robot navigates when obstacles are present without additional assistance.

5. **Pure Pursuit (PP) with Virtual Force Field (VFF) and Obstacles:**
   - The robot uses Pure Pursuit along with a Virtual Force Field (VFF) in an environment with obstacles.
   - The VFF aids the robot in maneuvering around obstacles effectively.

6. **Potential Field (PF) with Pure Pursuit (PP) and Obstacles:**
   - The robot navigates using the Potential Field method along with Pure Pursuit in an environment with obstacles.
   - This combination helps the robot navigate while considering both the attractive and repulsive forces of obstacles.

### Objective:

The objective of these experiments is to evaluate and compare the performance of different navigation methods in terms of efficiency and safety. Efficiency is measured by the time taken to reach the goal, while safety is assessed based on the ability of the robot to navigate without colliding with obstacles.

### Evaluation:

After conducting each experiment, the performance of each navigation method will be evaluated based on the defined criteria:
- Time taken to reach the goal: Lower is better.
- Safety of the path: A binary measure (0 or 1), where 1 indicates a safe path without collisions and 0 indicates otherwise.

The formula for the score can be represented as:

$Score\ =\ \left(\frac{1}{Time}\right)\cdot Safepath$
<br>

## Coding
The code can be found here

- [Velocity Controller](https://github.com/Nopparuj-an/FRA532_Mobile_Robots_LAB1/blob/master/src/robot_control/scripts/velocity_controller.py)
- [Odometry Publisher](https://github.com/Nopparuj-an/FRA532_Mobile_Robots_LAB1/blob/master/src/robot_control/scripts/odom_publisher.py)
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
