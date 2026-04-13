# 08_Robotprogramming_in_ROS

Further info of the course and its contents can be found [here](https://abc-irobotics.github.io/ros_course_materials_en/).

## Setup
### Create a ROS2 Workspace
```bash
mkdir -p ~/08_Robotprogramming_in_ROS/ros2_ws/src
```

### Create a ROS2 Package
```bash
cd ~/08_Robotprogramming_in_ROS/ros2_ws/src
ros2 pkg create --build-type ament_python <package-name>
```

### Build the Workspace
```bash
cd ~/08_Robotprogramming_in_ROS/ros2_ws
colcon build --symlink-install
```

### Setup Bash
> Generated during init process of a new workspace. Extends the shell environment. ROS can find any resources that have been installed or built to that location:

```bash
source ~/08_Robotprogramming_in_ROS/ros2_ws/install/setup.bash
```

### Test Built Package
```bash
ros2 run <package-name> <entry-point> 
```

## Turtle Sim
> Start `turtlesim_node` node with the following command, in separate terminal windows:

```bash
ros2 run turtlesim turtlesim_node
```

## dVRK Commands
### Simulation
- Terminal 1.: dVRK main console
```bash
ros2 run dvrk_robot dvrk_console_json -j ~/08_Robotprogramming_in_ROS/ros2_ws/install/sawIntuitiveResearchKitAll/share/sawIntuitiveResearchKit/share/console/console-PSM1_KIN_SIMULATED.json
```

- Terminal 2.: ROS 2 joint and robot state publishers
```bash
# Copy file to the right path
cp /home/dev/08_Robotprogramming_in_ROS/ros2_ws/src/dvrk/dvrk_model/model/PSM1.urdf.xacro /home/dev/08_Robotprogramming_in_ROS/ros2_ws/install/dvrk_model/share/dvrk_model/modelPSM1.urdf.xacro
```

```bash
ros2 launch dvrk_model dvrk_state_publisher.launch.py arm:=PSM1
```

- Terminal 3.: RViz
```bash
ros2 run rviz2 rviz2 -d ~/08_Robotprogramming_in_ROS/ros2_ws/install/dvrk_model/share/dvrk_model/rviz/PSM1.rviz
```
