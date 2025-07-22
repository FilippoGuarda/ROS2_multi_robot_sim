FROM hardikparwana/cuda118desktop:ros-humble-rmf


# WSL2-specific environment variables
ENV DISPLAY=:0
ENV LIBGL_ALWAYS_INDIRECT=0
ENV WSL_DISTRO_NAME=Ubuntu

RUN rm /var/lib/apt/lists/*ros*
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg 
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu \
     $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Combine RUN commands to reduce layers (better for WSL2 performance)
RUN apt-get update && apt-get install -y \
    wget \
    build-essential \
    libncursesw5-dev \
    libssl-dev \
    libsqlite3-dev \
    tk-dev \
    libgdbm-dev \
    libc6-dev \
    libbz2-dev \
    libffi-dev \
    zlib1g-dev \
    software-properties-common \
    curl \
    python3.10-dev \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3* \
    ros-humble-nav2-simple-commander \ 
    ros-humble-rqt-tf-tree \
    ros-humble-topic-tools \
    ros-humble-robot-localization \
    ros-humble-nav2-msgs \
    && rm -rf /var/lib/apt/lists/* 

# Python setup
RUN curl -sS https://bootstrap.pypa.io/get-pip.py | python3.10
RUN python3 -m pip install numpy

# Python packages installation (combined for efficiency)
# RUN pip3 install -U setuptools && \
#     pip3 install polytope numpy cvxpy jax jaxlib testresources cvxpylayers gurobipy

RUN python3 -m pip install setuptools==58.2.0 && \
    python3 -m pip install numpy==1.26.4 matplotlib

# JAX with CUDA support (ensure WSL2 CUDA drivers are installed)
# RUN python3 -m pip install --upgrade "jax[cuda11_pip]==0.4.25" -f https://storage.googleapis.com/jax-releases/jax_cuda_releases.html jaxlib==0.4.25

# RUN python3 -m pip install matplotlib==3.7.1 pillow==9.5.0 kiwisolver==1.4.4 polytope && \
#     python3 -m pip install myst-parser sphinx sphinx-rtd-theme

# Environment setup
RUN echo "export PYTHONPATH=\$PYTHONPATH:/home/workspace/src/social_navigation/src" >> ~/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /home/workspace/install/local_setup.bash" >> ~/.bashrc && \
    echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc

WORKDIR /home/

# # Build lightsfm
RUN git clone https://github.com/robotics-upo/lightsfm.git
WORKDIR /home/lightsfm
RUN make && make install

# # update gazebo version for wsl2 compatibility
# RUN apt-get remove -y ros-*-gazebo*
# RUN apt-get remove -y libgazebo*
# RUN apt-get remove -y gazebo*
# RUN wget https://packages.osrfoundation.org/gazebo.gpg \
#   -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg  
# RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
#   http://packages.osrfoundation.org/gazebo/ubuntu-stable \
#   $(lsb_release -cs) main" \
#   | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null  
# RUN sudo apt-get update  
# RUN apt-get install -y lsb-release wget gnupg  
# RUN apt-get install -y ros-humble-ros-gzgarden

WORKDIR /home/workspace

# Aliases (unchanged as they work fine in WSL2)
RUN echo "alias rgazebo='ros2 launch aws_robomaker_hospital_world view_hospital.launch.py'" >> ~/.bashrc && \
    echo "alias rnav2='ros2 launch multi_robot_sim test_multi_robot_launch.py'" >> ~/.bashrc && \
    echo "alias six_robots='input_file:=/home/workspace/install/multi_robot_sim/share/multi_robot_sim/configs/robot_setup_6.json'" >> ~/.bashrc

# ENTRYPOINT [ "/bin/bash -i ros2 launcnh aws_robomaker_hospital_world view_hospital.launch.py" ]
