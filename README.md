# checkpoint16
---

This package aims to understand the kinematic of an holonomic robot,
by applying kinematic courses onto a ROSBot XL simulated inside a Gazebo environment

## Instalation
---

#### prerequisites

 - ROS2 Galactic or higher  
 - GAZEBO  

#### Install

 1- Clone the repository inside your workspace:  
`git clone https://github.com/maxime-cognie/checkpoint16.git`  

 2- Build and setup the environment:
Navigate to the root of your workspace, then use the following command:  
```bash
colcon build
source install/setup.bash
```

## Task1    
---

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch kinematic_model kinematic_model.launch.py
``` 

## Task2       
---

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch eight_trajectory eight_trajectory.launch.py
```