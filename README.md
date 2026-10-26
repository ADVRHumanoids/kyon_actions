# Kyon direct motion replay — ROS 2

Home and replay a saved MAT trajectory directly through an already running
XBotCore. Only this repository and one replay image are needed. There is no
`kyon_controller`, Horizon, horizon_rti, phase_manager or CasADi dependency
in the replay image.

Follow [docker/README.md](docker/README.md) to build and run it. The publisher
uses XBotCore's live URDF; it starts no hardware driver or motion automatically.
[REAL_ROBOT.md](REAL_ROBOT.md) describes model, feedback and stop checks.
The container mounts this checkout at `/workspace/kyon_actions`; Python, MAT
and gain-file edits take effect on the next run without rebuilding.

## Selected motion

The [candidate collection](mat_files/candidates/README.md) contains the successful
centred 30 kg trajectories and their MuJoCo results. Use the explicit `MAT`
selection in [docker/README.md](docker/README.md) for both homing and replay.
The latest full motion lasts 20.28 s; a 15.08 s front-right-finish option is
also available. All use 12 leg joints, no arms, a 30 cm step and 10 cm clearance.
The centred payload mount is `[0, 0, 0.20]` m relative to the pelvis.

The command's unchanged default, `step_up_30kg_optimized_v2.mat`, uses the older
rear payload mount `[-0.20, 0, 0.20]` m. Select centred motions with `-f`.

The publisher checks the trajectory against the live joint limits. Feedforward
requires a matching robot and payload; saved model signatures are not checked.

[step_up_30kg_optimized_v2.yaml](config/step_up_30kg_optimized_v2.yaml) contains
the K=2000/D=120 simulation profile. Pass it with `replay --config ...` to apply
those gains; otherwise the publisher preserves the active gains. Homing also
preserves the active gains. No experimental 100 Nm hardware cap is applied.

The original MAT files are preserved. Tests, experiments and generated results
stay local and are not needed in the replay image. Optional simulation/plotting
commands can be installed outside this image with `INSTALL_SIMULATION_TOOLS=ON`;
they require the separate simulation workspace. Physical replay remains unvalidated.
