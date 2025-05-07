#!/bin/sh

cat <<'EOF' >>  /etc/ros/rosdep/sources.list.d/30-custom.list
    yaml file:///workspace/custom_rosdep.yaml
EOF