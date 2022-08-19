FROM ros:foxy-ros-core

RUN apt-get update && apt-get install -y curl build-essential python-is-python3 ros-foxy-navigation2 ros-foxy-nav2-bringup '~ros-foxy-turtlebot3-.*' ros-foxy-py-trees ros-foxy-py-trees-ros ros-foxy-py-trees-ros-interfaces
RUN curl -sSL https://bootstrap.pypa.io/get-pip.py -o get-pip.py
RUN python get-pip.py
RUN pip install py-trees

COPY ./go_to_pose_tree.py /etc/robot_control/

WORKDIR /etc/robot_control/

COPY ./ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/bin/bash", "/ros_entrypoint.sh"]