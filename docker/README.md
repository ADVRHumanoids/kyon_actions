# Direct step-up replay

XBotCore must already be running. Only `kyon_actions` branch `ros2` is needed:
no `kyon_controller`, Horizon, horizon_rti or phase_manager.

## 1. Build one image

From the `kyon_actions` repository:

```bash
docker build -f docker/Dockerfile -t kyon-replay:ros2 .
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

Choose the MAT explicitly. This example uses the latest successful full motion
for the **centred 30 kg payload**, 30 cm step and 10 cm clearance:

```bash
MAT="/workspace/kyon_actions/mat_files/candidates/step_up_30kg_centered_step30cm_order0321_landing_y_no_torque_cost_20261026.mat"

ros2 run kyon_actions replayer_ros2.py info -f "$MAT"
ros2 run kyon_actions replayer_ros2.py homing -f "$MAT" \
  --output /data/homing_feedback.csv

ros2 run kyon_actions replayer_ros2.py replay -f "$MAT" --feedforward \
  --config config/step_up_30kg_optimized_v2.yaml \
  --output /data/replay_feedback.csv
```

`--config` applies stiffness **2000** and damping **120** during replay; omit it
to preserve the current gains. Homing preserves the current gains. References
are sent directly to `/xbotcore/command` at 500 Hz. No separate controller launch.

Use the **same MAT** for homing and replay. Other successful motions are listed
in [the candidate table](../mat_files/candidates/README.md). The centred payload
mount is `[0, 0, 0.20]` m relative to the pelvis. Omitting `-f` selects the older
rear-payload motion. The repository mount exposes new MATs without rebuilding.

CSV feedback is saved in `recordings/`; use new names for each run.
No experimental 100 Nm cap is applied. See [REAL_ROBOT.md](../REAL_ROBOT.md)
for model checks and stop behavior. Physical robot validation is outstanding.
