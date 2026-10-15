# Real robot: ROS 2 step-up workflow

**Status: inspection and recording commands are available. Hardware homing and
trajectory replay are not implemented on this branch yet.** The simulation
launcher cannot be used to command the physical robot.

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

## 5. Home to the trajectory's initial posture — implementation pending

A hardware homing command must interpolate from measured joint positions to the
MAT file's first joint posture, using the robot's calibrated modes and gains.
It must verify measured convergence before replay starts. The simulation's
initial keyframe places the robot directly into its start state and is not a
hardware homing procedure.

There is no ROS 2 hardware homing command in this package yet. The ROS 1
implementation on `master` assumes arm/gripper joints and a different interface.

## 6. Start the replay controller and replay — implementation pending

The remaining implementation needs:

- A controller-only launch using the live robot descriptions and hardware gains.
- A hardware replayer with measured-state homing, wall-time pacing, initial-pose
  checks, and recording of the actual joint feedback.
- A controlled stop for completion, interruption, invalid input, and lost or
  stale trajectory messages.

The current controller continues sending the last position, velocity, and torque
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
