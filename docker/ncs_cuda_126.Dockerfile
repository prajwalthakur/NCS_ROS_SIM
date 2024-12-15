# Use Ubuntu 20.04 as the base image
FROM ubuntu:20.04

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

# Update and upgrade the system
RUN apt-get update && apt-get upgrade -y

# Install basic and required dependencies
RUN apt-get install -y --no-install-recommends \
    lsb-release \
    build-essential \
    python3 python3-dev python3-pip \
    cmake \
    git \
    vim \
    tmux \
    ca-certificates \
    libzmqpp-dev \
    libopencv-dev \
    gnupg2 \
    software-properties-common \
    qtbase5-dev \
    qttools5-dev-tools \
    libxcb-cursor0 \
    ffmpeg \
    python3-opengl \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Add PPA for updated Mesa drivers (useful for visualization tools like RViz)
RUN add-apt-repository ppa:kisak/kisak-mesa && apt-get update && apt-get install -y --no-install-recommends mesa-utils

# Add the ROS repository
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# https://github.com/osrf/docker_images/blob/df19ab7d5993d3b78a908362cdcd1479a8e78b35/ros/noetic/ubuntu/focal/ros-core/Dockerfile

# Update and install ROS
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    ros-noetic-desktop

# Install catkin tools
RUN apt-get install -y python3-setuptools python3-catkin \
    python3-rosinstall python3-vcstools


RUN apt-get install -y python3-tk 
ENV ROS_DISTRO=noetic


# Setup ROS environment
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc
RUN apt-get install -y ros-$ROS_DISTRO-plotjuggler-ros
# Install Python packages
RUN pip3 install --no-cache-dir \
    matplotlib \
    PyQt5 \
    dill \
    pandas \
    pyqtgraph \
    Cython

# Install HPIPM and BLASFEO
#hpipm install
RUN git clone https://github.com/giaf/blasfeo.git && cd blasfeo && make shared_library -j 4 && sudo make install_shared
RUN git clone https://github.com/giaf/hpipm.git  && cd hpipm && git checkout 6a0267dca70d6377859efc82dca8a5a1c509c2ae && make shared_library -j 4 && sudo make install_shared  && cd  /hpipm/interfaces/python/hpipm_python/ && pip3 install .
#RUN git clone https://github.com/prajwalthakur/hpipm-master.git  && mv hpipm-master hpipm && cd hpipm && make shared_library -j 4 && sudo make install_shared  && cd  /hpipm/interfaces/python/hpipm_python/ && pip3 install .

# Add library paths to LD_LIBRARY_PATH
RUN echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/usr/local/lib:/blasfeo/lib:/hpipm/lib" >> /root/.bashrc

RUN echo "source /hpipm/experiments/python/env.sh" >> /root/.bashrc
# Copy workspace files
ENV WORKSPACE_PATH=/root/workspace
COPY workspace/ $WORKSPACE_PATH/src/

# Initialize and update rosdep
# RUN rosdep init && rosdep update 
#&& cd $WORKSPACE_PATH && rosdep install --from-paths src -y --ignore-src

# Copy setup scripts
#COPY scripts/setup/ /root/scripts/setup
#RUN /root/scripts/setup/workspace.sh

# Set shell to bash
SHELL ["/bin/bash", "-c"]

# Final cleanup
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

# Set default shell to bash
CMD ["/bin/bash"]
