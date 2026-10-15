# Real robot: ROS 2 step-up workflow

**Status: the standalone ROS 2 `info`, `homing`, and `replay` commands are
implemented and tested against MuJoCo. Physical robot validation is outstanding.**
The simulation launcher starts MuJoCo; use the standalone commands below with
the robot’s normal XBot stack.

## 1. Open the robot's normal ROS 2 terminal

Use the environment and ROS domain normally used with the robot. The `ros2`
branch must be available in its workspace; the branch created during development
is currently local and has not been pushed.

Inspect the environment and installed commands:

```bash
echo "$ROS_DISTRO"
echo "$ROS_DOMAIN_ID"
ros2 pkg prefix kyon_controller
ros2 pkg executables kyon_actions
```

The actions package exposes `replayer_ros2.py` for the live XBot stack,
`replay_sim_ros2.py` for the full simulation pipeline, and `plot_experiment.py`.
Build and install the latest `kyon_controller` as well: its command-timeout
parameter is required for standalone replay. Do not set domain 77 solely because
it was used by the isolated simulation container.

## 2. Start the normal XBot hardware stack

Use the robot team's existing startup command and calibrated hardware YAML.
That command and YAML have not been supplied in this workspace, so there is no
verified startup command to copy here yet.

The simulation script starts XBot with `-H mj -S`; those settings select MuJoCo
and simulation time. The generated experiment YAML and `config/step_up.yaml`
have only been tested in simulation.

## 3. Inspect the live robot before commanding motion

These commands only inspect state:

```bash
ros2 node list
ros2 topic list
ros2 topic info /xbotcore/joint_states --verbose
ros2 topic echo /xbotcore/joint_states --once
ros2 topic info /xbotcore/robot_description --verbose
ros2 topic info /xbotcore/robot_description_semantic --verbose
ros2 topic info /mpc_solution --verbose
```

The joint-state message contains names, measured positions and velocities,
efforts, references, stiffness, and damping. Compare the names and settings with
the physical configuration: 12 leg joints, no arms, and the actual payload.

If `/mpc_solution` does not exist yet, its inspection command can report that the
topic is unknown. If it does exist, inspect its publishers: replay must have a
single command source. The general `kyon_controller/controller.launch.py` starts
`mpc_cpp` as well as the controller and is not a replay-only launch.

## 4. Start recording in another terminal

Record the essential feedback and replay-command topics:

```bash
ros2 bag record -o kyon_step_up_unloaded \
  /xbotcore/joint_states \
  /mpc_solution
```

Choose a new output name for each run. If `ros2 topic list` includes an IMU or
base-state estimate, add those actual topic names to the recording command.
The simulator's pelvis-pose topic is not assumed to exist on hardware.

Ctrl+C in this terminal stops the recording. It does not stop the robot.
The automatic PDF currently consumes simulation `telemetry.npz`; conversion of
hardware recordings into that report is not implemented yet.

## 5. Home the 12 leg joints to the MAT file's initial posture

Inspect the default trajectory without connecting to the robot:

```bash
ros2 run kyon_actions replayer_ros2.py info
```

The default is `step_up_clearance.mat`. Add `-f /path/to/trajectory.mat` to every
command below to use another file.

Run homing before starting the replay controller:

```bash
ros2 run kyon_actions replayer_ros2.py homing --output homing_feedback.csv
```

This interpolates the 12 measured leg joint positions to `q[7:, 0]` over five
seconds, preserves the current gains, and verifies the final measured error is
within 0.05 rad. The first seven MAT entries describe the floating base; there
are no arm or gripper targets. Every homing command has zero velocity and effort
references. `--duration` changes the homing time; `--position-tolerance` sets the
measured convergence tolerance in radians.

The replayer reads the live URDF and feedback, checks the model/joint limits,
requires a stationary robot and a single command publisher, and rejects stale
feedback. It does not start XBot or replace its robot description.

## 6. Start the existing controller, then replay

In a separate sourced terminal, start the existing controller with the robot's
calibrated controller YAML:

```bash
ros2 launch kyon_actions controller_step_up.launch.py \
  config:=/absolute/path/to/robot_controller.yaml
```

Replace the path with the verified hardware gains/modes YAML. This starts only
`kyon_controller`, uses the live XBot robot descriptions and wall time, and
initializes joint control modes. The default `mpc_config` is the installed
`config/replay_model.yaml`, containing a one-interval replay model with no fixed
arm joints. Override it with `mpc_config:=/path/to/replay_model.yaml` if needed.
The controller loop defaults to 500 Hz.

After it starts, run the MAT publisher in the previous terminal:

```bash
ros2 run kyon_actions replayer_ros2.py replay --output replay_feedback.csv
```

The publisher requires the measured initial joint posture to be within the same
0.05 rad tolerance. It interpolates the saved joint references at 200 Hz using
wall time. Saved full-body feedforward torques are disabled, matching the tested
leg-only experiment. CSV output records measured positions, velocities, efforts,
and requested references in joint-name order.

Normal completion and Ctrl+C request a position hold with zero velocity,
acceleration, and feedforward effort. The controller-only launch also enables
`command_timeout:=0.5`: after half a wall second without a trajectory message,
the controller holds its last position with zero velocity and feedforward.
Standalone replay refuses a controller without a positive timeout. Other
controller launches retain their existing behavior unless they enable this
parameter.

A position hold is not balance recovery or an emergency stop. Use the robot
team's established support and stop procedure for physical validation. Homing,
full MAT replay, and command expiry were checked with the live ROS 2 interfaces
against MuJoCo; the physical robot has not been commanded.

## 7. Validate unloaded before the 30 kg experiment

Begin with the unloaded robot and the verified hardware entry point. The current
30 kg simulation uses a saved full-body joint trajectory with feedforward torque
disabled; it has not been reoptimized for the loaded leg-only robot. Across runs,
tracking error reached 18.3 degrees and joint torque reached the model's 185 Nm
limit. Simulation completion alone does not establish hardware readiness.

The robot-specific inputs are its normal XBot startup command and calibrated
controller YAML. They remain supplied by the robot setup; the package does not
guess hardware gains or start a hardware driver.
