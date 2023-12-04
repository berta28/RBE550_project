FROM osrf/ros:noetic-desktop-full

ENV ROS_WORKSPACE /root/ws_moveit
SHELL ["/bin/bash", "-c"]
RUN apt-get update -y
RUN apt-get install python3.10 -y
RUN apt-get -y dist-upgrade
RUN apt-get -y install python3-catkin-tools ros-noetic-catkin python3-osrf-pycommon python3-wstool

RUN apt-get -y install vim
RUN apt-get -y install git

RUN mkdir ~/ws_moveit
RUN mkdir ~/ws_moveit/src
WORKDIR ${ROS_WORKSPACE}/src
RUN git clone https://github.com/ros-planning/moveit_tutorials.git -b master
RUN git clone https://github.com/ros-planning/panda_moveit_config.git -b noetic-devel
WORKDIR ${ROS_WORKSPACE}
RUN apt-get install -y ros-noetic-moveit
# RUN source /opt/ros/noetic/setup.bash
RUN catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN apt-get -y install ros-noetic-moveit-opw-kinematics-plugin
RUN git clone --recursive -b melodic-devel https://github.com/ros-industrial/fanuc.git src/fanuc
RUN rosdep update -y
RUN echo 'source ${ROS_WORKSPACE}/devel/setup.bash'
RUN rosdep install -y --from-paths src/ --ignore-src --rosdistro noetic

COPY moveit_configs/fanuc_cr7ial_moveit_config src/fanuc/moveit_cfgs/fanuc_cr7ial_moveit_config

RUN source /opt/ros/noetic/setup.bash && catkin build --no-env-cache
RUN source ${ROS_WORKSPACE}/devel/setup.bash

WORKDIR /opt/app
RUN apt-get install -y python3-pip python-dev ros-noetic-warehouse-ros-mongo
COPY requirements.txt /opt/app/requirements.txt
RUN python3 -m pip install --upgrade pip setuptools build
RUN pip install --no-cache-dir install -r requirements.txt

RUN source ${ROS_WORKSPACE}/devel/setup.bash
WORKDIR ${ROS_WORKSPACE}

ENTRYPOINT ["/bin/bash", "-c"]

CMD ["/bin/bash"]
