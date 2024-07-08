FROM althack/ros2:humble-cuda-base

# # create workspace
# RUN mkdir -p /ws

# 

# Install Dependencies
RUN apt install libboost-program-options-dev libusb-1.0-0-dev python3-pip -y

RUN apt update && apt install curl -y && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

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
RUN git clone --recurse-submodules https://github.com/IMRCLab/motion_capture_tracking

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


# Model Dependencies:
RUN mkdir -p /dep
WORKDIR /home/dep
RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
RUN git clone https://github.com/pytorch/tensordict.git
RUN cd /home/dep/tensordict && pip3 install tomli && python3 setup.py develop

RUN git clone https://github.com/pytorch/rl.git
RUN cd /home/dep/rl && python3 setup.py develop

RUN git clone https://github.com/btx0424/OmniDrones.git
RUN cd /home/dep/OmniDrones && pip3 install -e .


RUN mkdir -p /ws

WORKDIR /home/ws
ENTRYPOINT ["/docker-entrypoint.sh"]
CMD bash

