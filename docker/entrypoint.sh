#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /opt/xbot/setup.sh
source /home/user/env/bin/activate
source /home/user/kyon_ws/setup.bash
exec "$@"
