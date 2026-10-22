#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /opt/kyon-replay/share/xbot_msgs/local_setup.bash
source /opt/kyon-replay/share/kyon_actions/local_setup.bash
exec "$@"
