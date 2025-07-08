# Kyon Actions

This README provides instructions for setting up and using the `kyon_actions` package for replaying motion trajectories with XBotCore.

---

## Pull _kyon_action_ repo

```bash
cd /xbot2_ws/recipes/multidof_recipes/recipes  
git pull origin kyon-cetc
```

---

## Grow _kyon_action_ repo

Inside your `xbot_ws`:

```bash
forest grow kyon_actions
```

---

## Check Information of the Bag File

To inspect the `.mat` file containing the optimization output for the desired action, ready to be replayed on the robot:

```bash
cd kyon_actions/
python3 replayer.py -f mat_files/action_paw.mat info
```

---

## Replay the Trajectory

> **Note**: Ensure that `xbot-core` is active and filters are set to **medium** or **fast**.

1. Move the robot to the homing position:

    ```bash
    python3 replayer.py -f mat_files/action_paw.mat homing
    ```
1. Launch the controller:

    ```bash
    mon launch kyon_actions controller_step_up.launch
    ```

1. Replay the trajectory:

    ```bash
    python3 replayer.py -f mat_files/action_paw.mat replay
    ```

---