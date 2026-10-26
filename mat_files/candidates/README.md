# Step-up candidates

All centred candidates below completed one recorded MuJoCo climb with a
30 kg payload at `[0, 0, 0.20]` m relative to the pelvis, 12 leg joints, no arms,
a 30 cm step and 10 cm clearance. **Physical robot replay remains unvalidated.**

Use [the Docker commands](../../docker/README.md), selecting the same MAT for
homing and replay. All tests used feedforward, 500 Hz, K=2000 Nm/rad,
D=120 Nm*s/rad and normal 185 Nm effort limits, with no experimental torque cap.
These gains are the simulation profile, not hardware-validated gains.

| Motion | Duration | Predicted peak | Measured peak | Peak joint error |
| --- | ---: | ---: | ---: | ---: |
| [Latest: full 0321, fixed lateral landings](step_up_30kg_centered_step30cm_order0321_landing_y_no_torque_cost_20261026.mat) | 20.28 s | 111.4 Nm | 136.6 Nm | 4.3° |
| [Full 0321, joint limits](step_up_30kg_centered_step30cm_order0321_jointlimits_no_torque_cost_20261026.mat) | 20.28 s | 111.4 Nm | 135.9 Nm | 3.2° |
| [Original full 0321](step_up_30kg_centered_step30cm_clearance10cm_20261026.mat) | 20.28 s | 111.4 Nm | 131.2 Nm | 4.0° |
| [Front-right finishing step, 0321](step_up_30kg_centered_step30cm_front_right_finish_20261026.mat) | 15.08 s | 117.8 Nm | 130.7 Nm | 4.3° |
| [Stop after climb, 0321](step_up_30kg_centered_step30cm_stop_after_climb_20261026.mat) | 13.88 s | 120.4 Nm | 137.3 Nm | 3.6° |
| [Short order 2301](step_up_30kg_centered_step30cm_order2301_20261026.mat) | 15.08 s | 105.3 Nm | 148.9 Nm | 3.1° |
| [Short order 0123](step_up_30kg_centered_step30cm_order0123_20261026.mat) | 15.08 s | 130.9 Nm | 155.7 Nm | 3.2° |
| [Full order 2301](step_up_30kg_centered_step30cm_order2301_full_20261026.mat) | 20.28 s | 101.6 Nm | 159.0 Nm | 7.3° |

The latest full motion preserves all 16 lateral landing positions from the
original successful motion and enforces URDF joint-position bounds. It converged
in 51 iterations **without the added torque cost**. No successful torque-cost
trial is included. The finishing-step option has 12 swings and aligns the front
feet; it does not end in an exact homing posture.

## Historical rear-payload files

Already present in `mat_files/`; no duplicate copies are needed. These used
`[-0.20, 0, 0.20]` m payload mounting and an experimental **115 Nm applied-torque
cap**. Their results do not establish performance with the centred payload:

- [optimized_v2](../step_up_30kg_optimized_v2.mat): 1 climb; nonconverged feasible checkpoint.
- [optimized_seed](../step_up_30kg_optimized_seed.mat): 1 climb; nonconverged checkpoint.
- [candidate_v1](../step_up_30kg_candidate_v1.mat): 3 climbs, no feedforward; manually corrected, reference-clearance check fails.

[candidates.json](candidates.json) records hashes, solver status, model and replay
conditions. Its workspace-local evidence paths are provenance only; logs, graphs,
failed trials and simulator files are not runtime dependencies or shipped here.
Preserve accepted MATs and give new candidates descriptive, unique filenames.
