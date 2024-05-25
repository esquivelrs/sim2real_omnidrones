FROM ros:humble-ros-base-jammy


ENV DEBIAN_FRONTEND=noninteractive

RUN apt update
RUN apt upgrade -y

# # create workspace
# RUN mkdir -p /ws

# 

# Install Dependencies
RUN apt install libboost-program-options-dev libusb-1.0-0-dev python3-pip -y


# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \    
    libxkbcommon-x11-0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-xinerama0 \
    libxcb-xfixes0 \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# crazyswarm2 dependencies 
RUN sudo pip install rowan nicegui cflib transforms3d

# create workspace
RUN mkdir -p /ros_ws/src

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop=0.10.0-1* \
    ros-humble-motion-capture-tracking \
    ros-humble-tf-transformations \
    && rm -rf /var/lib/apt/lists/*




# Set environment variables
ENV MESA_LOADER_DRIVER_OVERRIDE=i965
ENV QT_X11_NO_MITSHM=1


# Crazyflie dependencies
RUN sudo apt update && sudo apt install -y libxcb-xinerama0 libxcb-cursor0 swig make gcc-arm-none-eabi
RUN sudo python3 -m pip install --upgrade pip
RUN sudo pip install cfclient

#USER root
RUN sudo usermod -a -G plugdev root
#USER cf

COPY files/99-bitcraze.rules /etc/udev/rules.d/99-bitcraze.rules
COPY files/docker-entrypoint.sh /docker-entrypoint.sh
RUN sudo chmod +x /docker-entrypoint.sh

RUN mkdir -p /ros2_ws/src
WORKDIR /home/ros2_ws/src

# clone the repository
RUN git clone https://github.com/IMRCLab/crazyswarm2 --recursive
RUN git clone https://github.com/tud-phi/ros2-mocap_optitrack.git

# build the repository
WORKDIR /home/ros2_ws
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# source the workspace
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /home/ros2_ws/install/setup.bash" >> ~/.bashrc

RUN git clone --recursive https://github.com/bitcraze/crazyflie-firmware.git
WORKDIR /home/ros2_ws/crazyflie-firmware
RUN make cf2_defconfig
RUN make -j 12
RUN make bindings_python

WORKDIR /home/ros2_ws/crazyflie-firmware
RUN cd build && python3 setup.py install

# export PYTHONPATH=/home/ros2_ws/crazyflie-firmware/build:$PYTHONPATH
ENV PYTHONPATH=/home/ros2_ws/crazyflie-firmware/build:$PYTHONPATH

RUN mkdir -p /ws

WORKDIR /home/ws
ENTRYPOINT ["/docker-entrypoint.sh"]
CMD bash

