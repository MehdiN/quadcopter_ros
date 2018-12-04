FROM ubuntu:16.04

# UPDATE 

RUN apt-get update && apt-get install -y \
            apt-utils \
            git \
            build-essential \
            g++ \
            python \ 
            python-pip \
            lsb-release && \
            rm -rf /var/lib/apt/lists/*

RUN python -m pip install --upgrade pip
RUN python -m pip install numpy scipy matplotlib pandas

#ROS INSTALL

#RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list'

RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

#RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

RUN apt-get update && apt-get install -y ros-kinetic-desktop-full
RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc && /bin/bash -c "source ~/.bashrc"
RUN apt-get install -y  python-rosinstall \ 
                        python-rosinstall-generator \ 
                        python-wstool build-essential && \
                        rm -rf /var/lib/apt/lists/*

RUN rosdep init \
    && rosdep update

ENV ROS_DISTRO kinetic
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

COPY .. /workspace
