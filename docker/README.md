# Step-up replay

XBotCore must already be running. This container uses its live URDF and feedback.
Use `kyon_controller` branch `ros2-replay` and `kyon_actions` branch `ros2`,
as sibling folders under `src/`.

## Build once on the robot PC

From the workspace root:

```bash
docker build -f src/kyon_controller/docker/Dockerfile \
  -t kyon-controller:latest src/kyon_controller
docker build --network none -f src/kyon_actions/docker/Dockerfile \
  -t kyon-replay:ros2 .
```

## 1. Open the container and home

From the workspace root, set the same ROS domain as XBotCore (replace `0`):

```bash
export ROS_DOMAIN_ID=0
mkdir -p artifacts/robot_replay
cd src/kyon_actions/docker
docker compose up -d --no-build
docker compose exec replay kyon-replay-entrypoint bash
```

Inside the container:

```bash
MAT="$(ros2 pkg prefix --share kyon_actions)/mat_files/step_up_30kg_optimized_v2.mat"
ros2 run kyon_actions replayer_ros2.py homing -f "$MAT" \
  --output /data/homing_feedback.csv
```

## 2. Start the controller

After homing, open a second host terminal in `workspace/src/kyon_actions/docker`.
Use the same ROS domain:

```bash
export ROS_DOMAIN_ID=0
docker compose exec replay kyon-replay-entrypoint bash
ros2 launch kyon_actions controller_step_up.launch.py \
  config:="$(ros2 pkg prefix --share kyon_actions)/config/step_up_30kg_optimized_v2.yaml" \
  rate:=500
```

The included YAML sets stiffness **2000** and damping **120** when replay starts.

## 3. Replay

In the first container terminal, where `MAT` is defined:

```bash
ros2 run kyon_actions replayer_ros2.py replay -f "$MAT" \
  --feedforward --rate 500 --output /data/replay_feedback.csv
```

Feedback is saved in `workspace/artifacts/robot_replay/`. Use fresh CSV names
for each run. No experimental 100 Nm cap is applied. Hardware replay remains
unvalidated; [REAL_ROBOT.md](../REAL_ROBOT.md) covers checks and stop behavior.
