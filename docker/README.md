# Direct step-up replay

XBotCore must already be running. Only `kyon_actions` branch `ros2` is needed:
no `kyon_controller`, Horizon, horizon_rti or phase_manager.

## 1. Build one image

From the `kyon_actions` repository:

```bash
docker build -f docker/Dockerfile -t kyon-replay:ros2 \
  --build-arg REPLAY_UID="$(id -u)" --build-arg REPLAY_GID="$(id -g)" .
```

## 2. Open the container

Use XBotCore's ROS domain (replace `0`):

```bash
export ROS_DOMAIN_ID=0
mkdir -p recordings
docker compose -f docker/compose.yaml up -d --no-build
docker compose -f docker/compose.yaml exec replay kyon-replay-entrypoint bash
```

The shell opens in `/workspace/kyon_actions`, your writable host checkout.
Edit the Python files, `config/` or `mat_files/` there; changes persist on the
host and the next `ros2 run` uses them. No root access or rebuild is needed.

## 3. Home and replay — same terminal

The default file is `step_up_30kg_optimized_v2.mat`:

```bash
ros2 run kyon_actions replayer_ros2.py homing --output /data/homing_feedback.csv

ros2 run kyon_actions replayer_ros2.py replay --feedforward \
  --config config/step_up_30kg_optimized_v2.yaml \
  --output /data/replay_feedback.csv
```

`--config` applies stiffness **2000** and damping **120** during replay; omit it
to preserve the current gains. Homing preserves the current gains. References
are sent directly to `/xbotcore/command` at 500 Hz. No separate controller launch.

CSV feedback is saved in `recordings/`; use new names for each run.
No experimental 100 Nm cap is applied. See [REAL_ROBOT.md](../REAL_ROBOT.md)
for model checks and stop behavior. Physical robot validation is outstanding.
