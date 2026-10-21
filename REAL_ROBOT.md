# Real robot: ROS 2 replay

Follow [docker/README.md](docker/README.md) for the command sequence:
**home → start controller → replay**. The container runs only the controller
and replayer; start your normal hardware XBotCore separately.

## Before motion

- Use the actual arm-free, 12-joint robot model and the 30 kg payload setup
  matching `step_up_30kg_optimized_v2.mat`. The feedforward command requires
  its exact live URDF signature and mass. If it rejects the model, check the
  robot description and regenerate the trajectory for it; do not bypass the check.
- Match the container's `ROS_DOMAIN_ID` to XBotCore. Simulation used 77;
  the robot's domain may differ. The Compose file uses host networking.
- Select the robot's calibrated controller YAML. The packaged K=2000/D=120
  profile was tested in simulation. The controller sets these gains when the
  first trajectory message arrives; editing the YAML does not reload it live.
- Verify the robot's existing control modes, total-effort limiting, support
  and stop procedure. The 100/115 Nm caps were simulation experiments only.
  The source leg URDF retains 185 Nm; the publisher sets no hardware torque cap.
- Use a single command source. Do not start the general controller launch
  alongside replay: it also starts an MPC publisher.

## Check the live connection

Inside the replay container, these commands only inspect state:

```bash
echo "${ROS_DOMAIN_ID:-0}"
ros2 node list
ros2 topic info /xbotcore/robot_description
ros2 topic info /xbotcore/robot_description_semantic
ros2 topic echo /xbotcore/joint_states --once
```

Inspect joint names, measured positions/velocities, efforts, stiffness and
damping. Before homing the robot must be stationary. Homing moves only the
12 leg joints to the MAT's initial posture over five seconds, preserving the
current gains. Replay requires an initial posture error below 0.05 rad.

## Record feedback

In another container shell, choose a fresh recording name:

```bash
ros2 bag record -o /data/step_up_trial1 \
  /xbotcore/joint_states /mpc_solution
```

Add actual IMU/base-state topics if the hardware publishes them. The replayer
also writes measured joint feedback and references to its selected CSV file.

## Completion and stopping

The standalone publisher requests a position hold with zero velocity,
acceleration and feedforward effort when it finishes or is interrupted.
The controller launch enables a 0.5 s wall-time command timeout with the same
hold behavior if messages stop. A hold is not balance recovery or an emergency
stop; use the robot's established stop procedure.

Homing and zero-feedforward replay were exercised against MuJoCo. The optimized
motion climbed using the experiment publisher. Full standalone feedforward
replay, including its final zero-feedforward hold transition, still requires
integration testing. Physical robot validation remains outstanding.
