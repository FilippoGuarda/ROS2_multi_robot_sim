# Need this cuda image
FROM hardikparwana/cuda118desktop:ros-humble-rmf

RUN apt-get update
RUN apt-get install -y wget build-essential libncursesw5-dev libssl-dev libsqlite3-dev tk-dev libgdbm-dev libc6-dev libbz2-dev libffi-dev zlib1g-dev
RUN apt-get install -y software-properties-common
RUN apt-get install -y curl
RUN curl -sS https://bootstrap.pypa.io/get-pip.py | python3.10
RUN apt-get install -y python3.10-dev

RUN apt-get install -y ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3*
RUN apt-get install -y ros-humble-nav2-simple-commander
RUN apt-get install -y ros-humble-rmw-cyclonedds-cpp
RUN pip3 install -U setuptools
RUN pip3 install polytope cvxpy jax jaxlib testresources cvxpylayers gurobipy

RUN python3 -m pip install setuptools==58.2.0
RUN python3 -m pip install networkX
RUN python3 -m pip install numpy==1.26.4 matplotlib 
RUN python3 -m pip install matplotlib==3.7.1 pillow==9.5.0 kiwisolver==1.4.4 polytope
RUN python3 -m pip install myst-parser sphinx sphinx-rtd-theme

WORKDIR /home/

RUN git clone https://github.com/robotics-upo/lightsfm.git
WORKDIR /home/lightsfm
RUN make && make install

# Setup SMrTa
ADD workspace/src/multi_robot_sim/multi_robot_sim_py/multi_robot_sim_py/SMrTa /home/workspace/src/multi_robot_sim/multi_robot_sim_py/multi_robot_sim_py/SMrTa
WORKDIR /home/workspace/src/multi_robot_sim/multi_robot_sim_py/multi_robot_sim_py/SMrTa
RUN pip3 install -r requirements.txt
RUN pip3 install .
WORKDIR /home/workspace/src/multi_robot_sim/multi_robot_sim_py/multi_robot_sim_py/SMrTa/bitwuzla
RUN pip3 install .
WORKDIR /home/workspace

RUN echo "export PYTHONPATH=\$PYTHONPATH:$(pwd)/build/src/api/python" >> ~/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

RUN echo "export PYTHONPATH=\$PYTHONPATH:/home/workspace/src/multi_robot_sim/src" >> ~/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
RUN echo "source /home/workspace/install/local_setup.bash" >> ~/.bashrc

RUN echo "alias rgazebo='ros2 launch aws_robomaker_hospital_world view_hospital.launch.py input_file:=/home/workspace/install/multi_robot_sim/share/multi_robot_sim/config/robot_setup_6.json'" >> ~/.bashrc && \
    echo "alias rnav2='ros2 launch multi_robot_sim test_multi_robot_launch.py input_file:=/home/workspace/src/multi_robot_sim/config/robot_setup_6.json'" >> ~/.bashrc && \
    echo "alias gcostmap='ros2 launch multi_robot_costmap_plugin multi_robot_costmap_launch.py'" >> ~/.bashrc && \
    echo "alias rsmrta='ros2 launch multi_robot_sim smrta_multi_robot_launch.py input_file:=/home/workspace/src/multi_robot_sim/config/robot_setup_6.json'" >> ~/.bashrc 

ENV GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH}:/home/workspace/src/aws-robomaker-hospital-world/models:/home/workspace/src/aws-robomaker-hospital-world/fuel_models"
