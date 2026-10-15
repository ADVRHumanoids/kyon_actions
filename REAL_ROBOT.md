# Real robot: ROS 2 step-up workflow

**Status: the ROS 2 controller exists and the MAT publisher works in simulation.
The standalone hardware homing/replay command still needs porting.** The simulation
launcher starts MuJoCo and cannot be used as the hardware entry point.

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

Currently, the actions package exposes `replay_sim_ros2.py` and
`plot_experiment.py`. These do not include a hardware replayer. Do not set domain
77 solely because it was used by the isolated simulation container.

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

## 5. Home the 12 leg joints to the trajectory's initial posture

A hardware homing command must interpolate the 12 measured leg joint positions
to `q[7:, 0]` from the MAT file, using the robot's calibrated modes and gains.
There are no arm/gripper targets. The first seven entries describe the floating
base and are not joint homing targets.
It must verify measured convergence before replay starts. The simulation's
initial keyframe places the robot directly into its start state and is not a
hardware homing procedure.

There is no ROS 2 hardware homing command in this package yet. The ROS 1
implementation on `master` assumes arm/gripper joints and a different interface.

## 6. Start the existing ROS 2 controller, then publish the MAT trajectory

The controller is already implemented in `kyon_controller` and is the same one
used by the simulation pipeline. The old ROS 1 `controller_step_up.launch` only
started that controller; the ROS 1 `replayer.py` provided the separate `homing`
and `replay` modes.

The equivalent ROS 2 controller-only launch is now available:

```bash
ros2 launch kyon_actions controller_step_up.launch.py \
  config:=/absolute/path/to/robot_controller.yaml \
  mpc_config:=/absolute/path/to/replay_model.yaml
```

Replace the paths with the verified hardware controller configuration and replay
model configuration. `config` supplies joint gains/modes; `mpc_config` supplies
`n_nodes`, `dt`, and `model.fixed_joint_map`. Both files are required. The launch
uses the live XBot robot descriptions and wall time, and starts no MPC publisher.
Launch it after homing with the correct hardware configuration: controller
initialization changes joint control modes.

The remaining port is the standalone ROS 2 equivalent of the old replayer's two
modes: measured-state homing, followed by wall-time publishing of the saved
`WBTrajectory` samples on `/mpc_solution`. The simulation replayer already
publishes that ROS 2 message; its simulator startup, initial keyframe, `/clock`,
and simulator-only feedback dependencies must be separated from the hardware
entry point. No new tracking controller is needed.

Completion/interruption handling also needs attention: the current controller
continues sending the last position, velocity, and torque
references after its publisher disappears. Stopping the publisher with Ctrl+C
does not provide a controlled robot stop. Validate the stop behavior using the
robot team's established support and stop procedure before a motion trial.

## 7. Validate unloaded before the 30 kg experiment

Begin with the unloaded robot and the verified hardware entry point. The current
30 kg simulation uses a saved full-body joint trajectory with feedforward torque
disabled; it has not been reoptimized for the loaded leg-only robot. Across runs,
tracking error reached 18.3 degrees and joint torque reached the model's 185 Nm
limit. Simulation completion alone does not establish hardware readiness.

To finish the executable hardware instructions, the next required information is
the robot's normal startup command and XBot hardware YAML path, including the
joint modes and gains it uses.
