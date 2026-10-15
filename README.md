# Kyon Actions — ROS 2

The `ros2` branch contains the ROS 2 replay pipeline. The original ROS 1 implementation remains on `master`.

## ROS 2 simulation without arms

`replay_sim_ros2.py` runs `step_up_clearance.mat` through the current
`kyon_controller` in MuJoCo, using a robot without arms and a 30 cm obstacle.
The saved file was optimized for the full robot. This replay uses its joint
positions and velocities with **feedforward torques disabled**; regenerate the
optimization for the leg-only model before treating this as a dynamics experiment.

Run the complete pipeline from the host workspace:

```bash
# Unloaded baseline
./src/kyon_actions/run_step_up_sim.sh

# Same motion with 30 kg on the existing pelvis payload mount
./src/kyon_actions/run_step_up_sim.sh --payload-kg 30
```

The launcher creates or starts `kyon-step-up-sim` from the local `hhcm-dev:noble`
image, installs missing GLFW development headers, installs the robot description,
builds the simulator and the `ament_cmake` actions package, and runs the replay on ROS domain 77 with CPU rendering.
Creating the container requires `DISPLAY` and `XAUTHORITY` for the viewer.
The workspace must already contain its built ROS 2/XBot dependencies. The simulator
changes are on `xbot2_mujoco` branch `ros2`; the payload mount fix is on
`iit-kyon-ros-pkg` branch `ros2`. `kyon_codesign` remains a separate, pending ROS 2 migration.

The replay starts the simulator, XBot in `mj` mode, and the controller; it stops
them after replay or a detected fall. `result.json` and `tracking.csv` record
the outcome; each process has a separate log. Results go to
`artifacts/step_up_clearance_legs_only` or
`artifacts/step_up_clearance_legs_only_payload_30kg`; `--output` overrides that path
inside the container. Reusing an output path replaces its previous results.
A fall returns exit code 2.
`--prepare-only` generates the model and configuration without starting simulation.
The MAT file's duration is computed from its 507 intervals at 0.04 s: 20.28 s.

After each completed replay or detected fall, the pipeline automatically writes
`plots/report.pdf` and six PNG graphs in the experiment directory:

- `torques.png`: applied torque for all 12 joints, with URDF torque limits.
- `velocities.png`: measured and requested joint velocities, with velocity limits.
- `positions.png` and `tracking_errors.png`: measured and requested positions and their errors.
- `base_motion.png`: base position, roll, pitch, and yaw against the replay reference.
- `mechanical_power.png`: joint mechanical power, calculated as applied torque times measured velocity.

Each row of the joint plots corresponds to one leg. Time zero is the start of
the motion; the initial and final posture holds are shaded. Torque and velocity
come from simulation feedback. Mechanical power excludes electrical losses.
The underlying per-joint measurements and replay references are stored in
`telemetry.npz`, sampled once per new joint-state message.

To regenerate graphs from a recorded run without repeating the simulation,
run this inside the container (no ROS environment is needed):

```bash
python3 /hhcm_ws/install/lib/kyon_actions/plot_experiment.py \
  /hhcm_ws/artifacts/step_up_clearance_legs_only_payload_30kg
```

The payload is a rigid 20 cm cube at the existing `pelvis_to_payload` mount:
`[-0.2, 0, 0.2]` meters in the pelvis frame. Its mass, box inertia, and collision
geometry are included in both controller and simulation models. Use
`--payload-position X Y Z` only to model a different mounting position.
The original mount height parameter (`pelvis_height=0.20`) was restored from the
robot-description history. No arms are present.

The 30 kg runs completed the 30 cm climb in simulation with total mass 98.654 kg.
Peak joint tracking error varied between 0.130 and 0.319 rad (7.5 to 18.3 degrees),
and reported joint effort reached the model's 185 Nm limit. This is a saved-motion
replay with feedforward disabled; the trajectory has not been optimized for the
loaded robot.

This requires the workspace's updated MuJoCo simulator, including initial `home`
keyframe support, simulated-clock transport, and bounded forces applied directly
to the joints of a URDF model without actuators. To run checks in the container:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/xbot/setup.sh
source /hhcm_ws/setup.bash
cmake --build /hhcm_ws/build/xbot2_mujoco --target test_joint_forces -j4
ctest --test-dir /hhcm_ws/build/xbot2_mujoco --output-on-failure
cd /hhcm_ws/src/kyon_actions
python3 -m unittest test_replay_sim_ros2 -v
```

## Real robot with ROS 2

The launcher above is simulation-only. It creates its own robot description,
starts XBot with `-H mj -S`, and uses an initial MuJoCo keyframe instead of robot
homing. Hardware replay has not been implemented for this ROS 2 branch.

The real-robot sequence remains: start the robot's normal XBot hardware stack,
home to the first trajectory posture, start a replay-only controller, and stream
the trajectory while recording joint feedback. A ROS 2 hardware replayer is not
implemented yet. It must use the robot's live descriptions and calibrated joint
modes/gains, wall time, measured-state homing, and a controlled stop on lost or
invalid data. The current controller keeps sending the last position, velocity,
and torque references after the trajectory publisher disappears; its replay
input has no stale-message timeout.

On the robot's normal sourced ROS 2 terminal, these commands only inspect state:

```bash
echo "$ROS_DISTRO"
ros2 node list
ros2 topic info /xbotcore/joint_states --verbose
ros2 topic echo /xbotcore/joint_states --once
ros2 topic info /mpc_solution --verbose
```

Use the robot's configured ROS domain. The isolated simulation domain 77 is not
a hardware setting. The general `controller.launch.py` also starts `mpc_cpp`,
which is another source of `/mpc_solution`; it is not a replay-only launch.

The hardware startup command and XBot YAML are required to prepare and validate
the ROS 2 hardware entry point. Begin hardware validation without the payload,
using the robot team's established support and stop procedure, before attempting
the loaded step-up. Simulation completion alone does not validate the current
30 kg replay for hardware, given the measured tracking error and torque saturation.
