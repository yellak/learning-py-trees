#!/bin/bash
set -e

# setup ros2 environment
echo "source \"/opt/ros/$ROS_DISTRO/setup.bash\"" >> /root/.bashrc
source /root/.bashrc
echo "source \"/opt/ros/$ROS_DISTRO/setup.bash\"" >> /root/.profile
source /root/.profile

tail -f /dev/null

exec "$@"