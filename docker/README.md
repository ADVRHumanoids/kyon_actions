# ROS 2 replay Docker

Image: `kyon-replay:ros2` (already built locally). Contains the controller,
replayer and MAT files. Uses the URDF from your running XBotCore; no 100 Nm
cap is applied. The controller YAML sets stiffness and damping.

## Build when the code or MAT files change

Use `kyon_actions` branch `ros2` and `kyon_controller` branch `ros2-replay`
as sibling checkouts under `src/`. From the HHCM workspace root:

```bash
# Only if the base image is missing:
docker build -f src/kyon_controller/docker/Dockerfile \
  -t kyon-controller:latest src/kyon_controller

docker build --network none -f src/kyon_actions/docker/Dockerfile \
  -t kyon-replay:ros2 .
```

## Transfer to the robot computer

```bash
docker image save kyon-replay:ros2 | gzip > kyon-replay-ros2.tar.gz
```

Copy the archive and this folder's `compose.yaml` to the robot computer. There:

```bash
docker image load -i kyon-replay-ros2.tar.gz
```

## Start the container

Start XBotCore normally. From the directory containing the copied `compose.yaml`,
set the robot's actual ROS domain and calibrated controller-config directory:

```bash
export ROS_DOMAIN_ID=0  # Example: must match XBotCore, not the simulation's 77
export ROBOT_CONFIG_DIR=/absolute/path/to/robot/config
export REPLAY_DATA_DIR="$PWD/recordings"
mkdir -p "$REPLAY_DATA_DIR"
docker compose up -d --no-build
docker compose exec replay kyon-replay-entrypoint bash
```

This starts no motion. Feedback CSV files are saved in `recordings/`.

## Home, start the controller, replay

In the container shell, inspect feedback and home:

```bash
ros2 topic echo /xbotcore/joint_states --once
MAT="$(ros2 pkg prefix --share kyon_actions)/mat_files/step_up_30kg_optimized_v2.mat"
ros2 run kyon_actions replayer_ros2.py info -f "$MAT"
ros2 run kyon_actions replayer_ros2.py homing -f "$MAT" \
  --output /data/homing_feedback.csv
```

After homing, open another container shell from a second host terminal:

```bash
docker compose exec replay kyon-replay-entrypoint bash
ros2 launch kyon_actions controller_step_up.launch.py \
  config:=/robot-config/robot_controller.yaml rate:=500
```

Replace `robot_controller.yaml` with your calibrated YAML filename.
Then, in the first container shell:

```bash
ros2 run kyon_actions replayer_ros2.py replay -f "$MAT" \
  --feedforward --rate 500 --output /data/replay_feedback.csv
```

Use fresh CSV names for each trial. The live model must match the optimized MAT.
Image checks passed; full standalone feedforward replay and hardware validation
remain outstanding. See [REAL_ROBOT.md](../REAL_ROBOT.md) for hardware checks,
recording and stop behavior.
