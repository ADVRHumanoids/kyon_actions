#!/usr/bin/env bash
set -euo pipefail

task_script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
task_baseline=false
task_args=()
for task_arg in "$@"; do
    case "$task_arg" in
        --baseline) task_baseline=true ;;
        --prepare-only) task_baseline=true; task_args+=("$task_arg") ;;
        *) task_args+=("$task_arg") ;;
    esac
done
if [[ "$task_baseline" == false && -x "$task_script_dir/experiments/run_measured_step_up.sh" ]]; then
    exec "$task_script_dir/experiments/run_measured_step_up.sh" "${task_args[@]}"
fi

task_workspace=$(cd -- "$task_script_dir/../.." && pwd)
task_container=kyon-step-up-sim
exec 9>"/tmp/${task_container}-${UID}.lock"
flock -n 9 || { echo 'Another step-up experiment is running.' >&2; exit 1; }

if ! docker container inspect "$task_container" >/dev/null 2>&1; then
    : "${DISPLAY:?Set DISPLAY to run the MuJoCo viewer}"
    : "${XAUTHORITY:?Set XAUTHORITY to the current display authorization file}"
    docker run -d --name "$task_container" --runtime=runc --shm-size=512m \
        -e ROS_DOMAIN_ID=77 -e LIBGL_ALWAYS_SOFTWARE=1 \
        -e DISPLAY="$DISPLAY" -e XAUTHORITY=/tmp/.Xauthority \
        --mount "type=bind,src=$task_workspace,dst=/hhcm_ws" \
        --mount type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix \
        --mount "type=bind,src=$XAUTHORITY,dst=/tmp/.Xauthority,readonly" \
        hhcm-dev:noble sleep infinity
elif [[ $(docker inspect --format '{{.State.Running}}' "$task_container") != true ]]; then
    docker start "$task_container"
fi

if ! docker exec "$task_container" test -f /usr/include/GLFW/glfw3.h; then
    docker exec -u root "$task_container" bash -lc \
        'apt-get update -qq && apt-get install -y --no-install-recommends libglfw3-dev'
fi

task_tty=()
if [[ -t 0 && -t 1 ]]; then task_tty=(-t); fi
docker exec -i "${task_tty[@]}" -w /hhcm_ws "$task_container" bash -lc '
    set -e
    source /opt/ros/jazzy/setup.bash
    source /opt/xbot/setup.sh
    source /hhcm_ws/setup.bash
    export ROS_DOMAIN_ID=77
    cmake --install /hhcm_ws/build/iit-kyon-ros-pkg/kyon_urdf > /tmp/step-up-model-install.log
    cmake --build /hhcm_ws/build/xbot2_mujoco --target mujoco_simulator -j4
    cmake --install /hhcm_ws/build/xbot2_mujoco
    cmake -S /hhcm_ws/src/kyon_actions -B /hhcm_ws/build/kyon_actions \
        -DCMAKE_INSTALL_PREFIX=/hhcm_ws/install
    cmake --build /hhcm_ws/build/kyon_actions -j4
    cmake --install /hhcm_ws/build/kyon_actions
    exec ros2 run kyon_actions replay_sim_ros2.py "$@"
' bash "${task_args[@]}"
