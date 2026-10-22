# Kyon motion replay — ROS 2

Run homing and saved MAT motions through an already running ROS 2 XBotCore.
Use the [Docker instructions](docker/README.md) for building and
running the replay container, and [REAL_ROBOT.md](REAL_ROBOT.md) for hardware
setup and feedback checks. The controller and replayer use XBotCore's live
URDF/SRDF; the container starts no hardware drivers or motion automatically.

## Selected motion

[step_up_30kg_optimized_v2.mat](mat_files/step_up_30kg_optimized_v2.mat):
12 leg joints, no arms, 30 kg at the existing payload mount, 30 cm step,
20.28 s duration. Total modeled mass is 98.654 kg. Replay with
`--feedforward --rate 500`; the live model must match the MAT's URDF signature.

The optimization minimizes peak torque. Its selected feasible checkpoint has
89.4 Nm predicted peak and passed dynamics and sampled mesh checks; the solver
reached its iteration limit without proving optimality. MuJoCo replays completed
once at a 115 Nm applied cap and once at 100 Nm. The 100 Nm replay had 7.0° peak
joint error and 13.75 cm final body-position error. These are individual
simulation runs, not hardware validation or a repeatability claim.

[step_up_30kg_optimized_v2.yaml](config/step_up_30kg_optimized_v2.yaml) records
the tested K=2000/D=120 simulation gains. The controller applies its selected
YAML gains on the first trajectory message.
No experimental 100 Nm cap is applied by the robot replay commands.

The original MAT files are preserved. Experiments and generated results stay
local; they are not required on the robot.
