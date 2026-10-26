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
| [Full 0321, fixed lateral landings](step_up_30kg_centered_step30cm_order0321_landing_y_no_torque_cost_20261026.mat) | 20.28 s | 111.4 Nm | 136.6 Nm | 4.3° |
| [Full 0321, COM margin 1 cm](step_up_30kg_centered_step30cm_order0321_com_margin1cm_20261026.mat) | 20.28 s | 116.0 Nm | 142.2 Nm | 5.7° |
| [Full 0321, joint limits](step_up_30kg_centered_step30cm_order0321_jointlimits_no_torque_cost_20261026.mat) | 20.28 s | 111.4 Nm | 135.9 Nm | 3.2° |
| [Original full 0321](step_up_30kg_centered_step30cm_clearance10cm_20261026.mat) | 20.28 s | 111.4 Nm | 131.2 Nm | 4.0° |
| [Front-right finishing step, 0321](step_up_30kg_centered_step30cm_front_right_finish_20261026.mat) | 15.08 s | 117.8 Nm | 130.7 Nm | 4.3° |
| [Front-right finish, body tracking + torque cost](step_up_30kg_centered_step30cm_front_right_finish_bodytrack_tau001_20261026.mat) | 15.08 s | 115.6 Nm | 163.1 Nm | 5.0° |
| [Front-right finish, force smoothing](step_up_30kg_centered_step30cm_front_right_finish_force_smooth003_20261026.mat) | 15.08 s | 116.5 Nm | 154.3 Nm | 10.5° |
| [Front-right finish, 0.6 s support](step_up_30kg_centered_step30cm_front_right_finish_stance600ms_20261026.mat) | 17.28 s | 120.9 Nm | 145.8 Nm | 3.0° |
| [0321 then 1032, earlier right-front steps](step_up_30kg_centered_step30cm_order0321_then1032_20261026.mat) | 15.08 s | 131.2 Nm | 146.4 Nm | 4.5° |
| [Front-right finish, first two RF landings +10 cm](step_up_30kg_centered_step30cm_front_right_finish_rf_forward10cm_20261026.mat) | 15.08 s | 117.7 Nm | 134.8 Nm | 3.8° |
| [Stop after climb, 0321](step_up_30kg_centered_step30cm_stop_after_climb_20261026.mat) | 13.88 s | 120.4 Nm | 137.3 Nm | 3.6° |
| [Short order 2301](step_up_30kg_centered_step30cm_order2301_20261026.mat) | 15.08 s | 105.3 Nm | 148.9 Nm | 3.1° |
| [Short order 0123](step_up_30kg_centered_step30cm_order0123_20261026.mat) | 15.08 s | 130.9 Nm | 155.7 Nm | 3.2° |
| [Full order 2301](step_up_30kg_centered_step30cm_order2301_full_20261026.mat) | 20.28 s | 101.6 Nm | 159.0 Nm | 7.3° |

The fixed-lateral-landing baseline preserves all 16 lateral landing positions from the
original successful motion and enforces URDF joint-position bounds. It converged
in 51 iterations **without the added torque cost**. The shorter body-tracking comparison includes a torque cost; it climbed but
measured a higher peak than the original finishing-step candidate. The finishing-step option has 12 swings and aligns the front
feet; it does not end in an exact homing posture.

The COM-margin comparison adds a 1 cm support-triangle margin only during the
last rear-leg climb (13.00–13.76 s), with the torque cost disabled. It converged
in 35 iterations and climbed, but measured peak torque increased. It is preserved
as a comparison, not selected as the replay default.

The force-smoothing comparison keeps body tracking and the landing/COM
constraints, replaces the torque cost with a 0.003 residual on force differences
between 40 ms nodes, and converged in 32 iterations. It climbed but did not beat
the original finishing-step candidate.

The 0.6 s support comparison uses the original short generator costs, 0.8 s
swings and URDF joint-position bounds. Its phase-matched position seed comes
from the original short candidate; no torque, body-tracking or force-smoothing
cost is added. It converged in 33 iterations.

The mixed-order comparison uses `0321` for the first cycle, then `1032` for the
remaining two cycles, with 12 swings and the original costs. It converged in
39 iterations. Measured right-front hip pitch during upper-step stance peaked
at 73 degrees; no hip-posture penalty was used.

The forward-foothold comparison moves only the first two right-front landings
10 cm forward, with matching soft swing references. Its last landing, original
0321 gait, timing and costs are preserved. It converged in 66 iterations.
Measured right-front hip pitch still reaches **81.5 degrees**: the reported
stance-posture concern remains unresolved, despite completing the simulated climb.

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
