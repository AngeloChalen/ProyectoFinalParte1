Unitree Go2 (Simulación – ROS 2 + Gazebo + RViz)



> Descripción general del proyecto

En este proyecto, los estudiantes deberán implementar un planificador de trayectorias globales para un robot cuadrúpedo Unitree Go2 en simulación, que incluya:

    Mapeo del entorno (SLAM)

    Planificación global de trayectoria

    Visualización del resultado en RViz


## Tested on:
- Ubuntu 22.04 (ROS2 Humble)

## Current state of package:

- &check; Configure go2 robot with champ config
- &check; Robots Configurations.
    - &check; Porting of robot description packages to ROS 2.
    - &check; Porting of robot URDF to ROS2 (add new ros2_control tag).
    - &check; Porting of robot configurationf to ROS2.
    - &check; Porting of robot launch Files to ROS2.
- &check; Upgrade go2 description model for ros2 humble
- &check; Spawning go2 in gazebo environment.
- &check; Working rviz only demo.
- &check; Working Gazebo with teleoperated robot.
- &check; Adding IMU and 2D LiDAR.
- &check; Adding 3D LiDAR (Velodyne).
- &cross; Working Gazebo demo with SLAM.
- &cross; Working Gazebo demo with nav2 integration.

## 1. Installation

### 1.0 Install ROS-based dependencies:
```bash
sudo apt install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-xacro
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-velodyne
sudo apt install ros-humble-velodyne-gazebo-plugins
sudo apt-get install ros-humble-velodyne-description
```

### 1.1 Clone and install all dependencies:
    
```bash
sudo apt install -y python3-rosdep
rosdep update

mkdir -p go2_ws/src
cd go2_ws/src
git clone https://github.com/widegonz/unitree-go2-ros2.git
cd ~/go2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 1.2 Build your workspace:
```bash
cd ~/go2_ws
colcon build
. go2_ws/install/setup.bash
```
## 2. Activate the lidar 


```bash
ros2 launch go2_config gazebo_velodyne.launch.py
```


### 3 export the gazebo-path
```bash
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/israel/go2_ws/src/unitree-go2-ros2/robots/configs/go2_config/models/' >> ~/.bashrc
```




## 4. Run the gazebo in factory map


```bash
cd ~/go2_ws
colcon build
source install/setup.bash
ros2 launch go2_config gazebo.launch.py world:=factory 

```

Change the `<your_user>` part to your username.


## 5. Execute the launch file that utilizes the slam_toolbox package

```bash
ros2 launch go2_config slam.launch.py use_sim_time:=true
```

The RViz panel will open as shown below:
<img width="1850" height="1053" alt="Screenshot from 2025-12-22 22-11-50" src="https://github.com/user-attachments/assets/b01bac03-87a5-4777-ba94-e691a3091d9c" />


##6. Run the teleoperation node
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

##7. Navigate the robot:
  - It is crucial to mention that during the mapping process, the robot must not collide with any obstacles. This is primarily because a collision disrupts the odometry, causing the LiDAR data to mismatch with the measurements, which results in a corrupted map.

##8. Save the map using the following command:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

  - This command will generate the `.pgm` and `.yaml` files for the map in the root directory. Once this map is generated, trajectory planning can be performed.


## Acknowledgements

This project builds upon and incorporates work from the following projects:

* [Unitree Robotics](https://github.com/unitreerobotics/unitree_ros) - For the Go2 robot description (URDF model).
* [CHAMP](https://github.com/chvmp/champ) - For the quadruped controller framework.
* [CHAMP Robots](https://github.com/chvmp/robots) - For robot configurations and setup examples.

We are grateful to the developers and contributors of these projects for their valuable work.
