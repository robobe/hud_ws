#!/bin/bash

sudo rosdep init

# cat <<'EOF' >>  /etc/ros/rosdep/sources.list.d/30-custom.list
sudo tee /etc/ros/rosdep/sources.list.d/30-custom.list > /dev/null <<'EOF'
    yaml file:///workspace/custom_rosdep.yaml
EOF

rosdep update --rosdistro $ROS_DISTRO