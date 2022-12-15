FROM ros:noetic
RUN mkdir -p /home/ros_ws/src
WORKDIR /home/ros_ws
RUN apt-get update &&\ 
    apt-get install -y git 
RUN apt-get update
RUN apt-get install -y python3-scipy
Run apt-get install -y iputils-ping
RUN cd src && git clone https://github.com/ros-industrial/kuka_experimental.git
RUN sudo apt update -qq
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -y
SHELL ["/bin/bash","-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN source /opt/ros/noetic/setup.bash && \
    apt-get install -y ros-noetic-moveit && \
    apt-get install -y ros-noetic-pcl-ros && \
    catkin_make && \
    source devel/setup.bash
CMD ["bash"]