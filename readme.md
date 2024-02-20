# LAB 1: Local Planner
PLACEHOLDER

<br>

## Changing differential drive controller (diff_cont) to joint group velocity controller (velocity_controllers)
PLACEHOLDER

<br>

## Local planner pseudocode

- #### Pseudocode: Dynamic Pure Pursuit

  PLACEHOLDER

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
