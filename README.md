# ROS2_multi_robot_sim

# Warning!!do
Everything here is still a big work in progress, the project originally started from a fork of Victoria Tuck's [Multi Robot Task Allocation Stack](https://github.com/victoria-tuck/multi-robot-task-allocation-stack)! made specifically for WSL2 on windows.  
But now it is being reworked from the ground up since the use case is different and not completely compatible with the original project.

If you want to use wsl, it takes as a given that the WSL2 environment is set up and that `nvidia-smi` is posting on the command line.

## Hardware requirements
- NVIDIA discrete GPU (for gazbeo)
- Diplay should use NVIDA and not integrated graphics
- Docker installed (docker and nvidia-docker)
- https://docs.docker.com/engine/install/ubuntu/
- https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html

## Installation
This documentation is for the `main` branch of `ROS2_multi_robot_sim` repository.

To setup the Multi robot costmap plugin, first add the submodules by running
```

git submodule init
git submodule update
```

Then to build the environment, run
```
# For WSL2
COMPOSE_PROFILES=wsl docker-compose up 
# For Linux
COMPOSE_PROFILES=linux docker-compose up 

docker compose up -d
docker exec -it multi-robot-task-allocation-stack-ros-1 bash
```
from the highest level of the repository where `<service>` is `ubuntu` or `wsl` depending on your operating system.

Now, you have to first build the colcon (ROS2) workspace. Navigate to
```
cd /home/colcon_ws
colcon build --symlink-install
```
Note sometimes ROS does not figure out package dependency order properly when multiple ROS packages are present. In this case, it may take multiple runs of colcon build to be successful. If one error is shown, after this step, you can still proceed successfully.

Now source the installed packages with following command
```
source ~/.bashrc
```
Finally, to give docker environment permission to use graphics of hist machine, run the following command **from host machine**
```
xhost +
```

## Running the Code
Then run the code in the following sequence. 

1. To launch the gazebo environment with the robots inside it

```
rgazebo
```
Alternatively, you can launch a specific input file with 
```
ros2 launch aws_robomaker_hospital_world hospital.launch.py input_file:=<input_file_path>
```


Note: After this step, the Gazebo environment should the robots (which are in the middle of blue circles). Upon start-up, the robot installation will occasionally fail. If this occurs, exit and rerun the above command.

2. To start the costmap fusion node

```
gcostmap
```

The global costmap can also be launched straight from 

```
ros2 launch multi_robot_costmap_plugin
multi_robot_costmap_launch.py
```

3. To launch the ROS2 navigation stack (to use its planners)

```
rnav2
```
Example:
```
 ros2 launch multi_robot_sim test_multi_robot_launch.py input_file:=<input_file_path>
```
