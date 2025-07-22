# Need this cuda image
FROM hardikparwana/cuda118desktop:ros-humble-rmf

RUN apt-get update
RUN apt-get install -y wget build-essential libncursesw5-dev libssl-dev libsqlite3-dev tk-dev libgdbm-dev libc6-dev libbz2-dev libffi-dev zlib1g-dev
RUN apt-get install -y software-properties-common
RUN apt-get install -y curl
RUN curl -sS https://bootstrap.pypa.io/get-pip.py | python3.10
RUN python3 -m pip install numpy
RUN apt-get install -y python3.10-dev

RUN apt-get install -y ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3*
RUN apt-get install -y ros-humble-nav2-simple-commander
RUN pip3 install -U setuptools
RUN pip3 install polytope numpy cvxpy jax jaxlib testresources cvxpylayers gurobipy

RUN python3 -m pip install setuptools==58.2.0
RUN python3 -m pip install numpy==1.26.4 matplotlib
RUN python3 -m pip install --upgrade "jax[cuda11_pip]==0.4.25" -f https://storage.googleapis.com/jax-releases/jax_cuda_releases.html jaxlib==0.4.25
RUN python3 -m pip install matplotlib==3.7.1 pillow==9.5.0 kiwisolver==1.4.4 polytope
RUN python3 -m pip install myst-parser sphinx sphinx-rtd-theme

RUN echo "export PYTHONPATH=\$PYTHONPATH:/home/workspace/src/multi_robot_sim/src" >> ~/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
RUN echo "source /home/workspace/install/local_setup.bash" >> ~/.bashrc

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
RUN echo "export PYTHONPATH=\$PYTHONPATH:$(pwd)/build/src/api/python" >> ~/.bashrc

WORKDIR /home/workspace
#RUN add-apt-repository ppa:deadsnakes/ppa
#RUN apt-get install -y python3.11
#RUN apt-get install -y python3.11-distutils
#RUN apt-get install -y python3.11-dev
#RUN apt-get install -y curl
#RUN curl -sS https://bootstrap.pypa.io/get-pip.py | python3.11

#RUN python3.11 -m pip install numpy matplotlib
#RUN python3.11 -m pip install --upgrade "jax[cuda11_pip]" -f https://storage.googleapis.com/jax-releases/jax_cuda_releases.html jaxlib



#RUN apt-get install -y ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3*
#RUN apt-get install -y ros-humble-nav2-simple-commander
#RUN pip3 install --upgrade setuptools
#RUN pip3 install cvxpy jax jaxlib testresources cvxpylayers
#RUN pip3 install --upgrade numpy
#RUN pip3 install gurobipy
#RUN python3 -m pip install jaxopt
#RUN pip3 install -U setuptools
#RUN pip3 install polytope
#WORKDIR /home/
#RUN echo "export PYTHONPATH=$PYTHONPATH:/home/workspace/src/multi_robot_sim/src" >> ~/.bashrc
#RUN echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc
#RUN echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
#RUN echo "source /home/workspace/install/local_setup.bash" >> ~/.bashrc

RUN echo "alias rgazebo='ros2 launch aws_robomaker_hospital_world view_hospital.launch.py'" >> ~/.bashrc
RUN echo "alias rnav2='ros2 launch multi_robot_sim nav2_tb3_aws_launch.py'" >> ~/.bashrc
RUN ec