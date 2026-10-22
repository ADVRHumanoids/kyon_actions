# Kyon direct motion replay — ROS 2

Home and replay a saved MAT trajectory directly through an already running
XBotCore. Only this repository and one replay image are needed. There is no
`kyon_controller`, Horizon, horizon_rti, phase_manager or CasADi dependency
in the replay image.

Follow [docker/README.md](docker/README.md) to build and run it. The publisher
uses XBotCore's live URDF; it starts no hardware driver or motion automatically.
[REAL_ROBOT.md](REAL_ROBOT.md) describes model, feedback and stop checks.

## Selected motion

[step_up_30kg_optimized_v2.mat](mat_files/step_up_30kg_optimized_v2.mat):
12 leg joints, no arms, 30 kg at the existing payload mount, 30 cm step,
20.28 s duration. Total modeled mass is 98.654 kg. It is the default file.
Use `--feedforward` for the optimized torques; other files can be selected with `-f`.

The MAT includes exact signatures for the planning URDF and XBot's serialized
export, with their corresponding masses. These allow validation of XBotCore's
published model without a dynamics library. Motion arrays and planned torques
are unchanged by this metadata addition. Unexpected models are rejected.

[step_up_30kg_optimized_v2.yaml](config/step_up_30kg_optimized_v2.yaml) contains
the K=2000/D=120 simulation profile. Pass it with `replay --config ...` to apply
those gains; otherwise the publisher preserves the active gains. Homing also
preserves the active gains. No experimental 100 Nm hardware cap is applied.

The original MAT files are preserved. Tests, experiments and generated results
stay local and are not needed in the replay image. Optional simulation/plotting
commands can be installed outside this image with `INSTALL_SIMULATION_TOOLS=ON`;
they require the separate simulation workspace. Physical replay remains unvalidated.
