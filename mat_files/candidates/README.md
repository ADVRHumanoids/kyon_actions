# Step-up candidates

Current successful motions use a centred 30 kg payload at `[0, 0, 0.20]` m
relative to the pelvis, a 30 cm step and 10 cm clearance. Success here means
completing the recorded MuJoCo climb and final hold. **Hardware replay remains
unvalidated.**

Follow [the Docker commands](../../docker/README.md). Copy one **Set MAT**
assignment below and use the same `"$MAT"` for homing and replay.
Simulation settings: feedforward, 500 Hz, K=2000 Nm/rad, D=120 Nm*s/rad,
normal 185 Nm effort limits; no experimental torque cap.

## Without arms

| Motion | Duration | Predicted peak | Measured peak | Peak joint error | Set MAT |
| --- | ---: | ---: | ---: | ---: | --- |
| [Short 0321, COM margin 2 cm](step_up_centered30_order0321_support_margin2cm_short_trial1.mat) — **Worked: climb and hold completed** | 15.08 s | 119.2 Nm | 185 Nm | 21.4° | `MAT="/workspace/kyon_actions/mat_files/candidates/step_up_centered30_order0321_support_margin2cm_short_trial1.mat"` |

Total model mass is **98.65 kg**, including the 30 kg payload. The optimization
converged in 52 iterations. The last cycle of the full motion is removed;
the final right-front step aligns the front feet before stopping.

The replay completed the climb, but its 21.4-degree tracking peak exceeds the
tracking bound and measured COM briefly crosses the planned support edge by 5 mm.

## With arms

| Motion | Duration | Predicted peak | Measured peak | Peak joint error | Set MAT |
| --- | ---: | ---: | ---: | ---: | --- |
| [Short 0321, COM margin 2 cm, arms fixed backward](step_up_centered30_order0321_support_margin2cm_short_fixed_arms_back_trial1.mat) — **Worked: 2/2 MuJoCo climbs and holds** | 15.08 s | 134.2 Nm | 185 Nm | 7.9° | `MAT="/workspace/kyon_actions/mat_files/candidates/step_up_centered30_order0321_support_margin2cm_short_fixed_arms_back_trial1.mat"` |

Arms are **fixed backward**: shoulder pitches +90 degrees left / -90 degrees
right; elbows, wrists and grippers at zero. The arms add **13.22 kg**, bringing
total mass to **111.88 kg** including the 30 kg payload. Costs, constraints,
timing and foot landing targets match the version without arms. The accepted
short motion supplies the initial guess; the new model is reoptimized rather
than reusing the old torques. It converged in 316 iterations.

Both recorded MuJoCo climbs completed; tracking peaks were 7.9 and 7.2 degrees
(the table shows the worst). The first replay's measured COM stayed at least
1.17 cm inside the triangle inferred from the planned supporting feet.

**Requires the matching fixed-arm model and posture.** The MAT commands only
the 12 leg joints; it does not position the arms. Arms were rigidly fixed in
simulation, so arm servo holding has not been validated.

Homing and replay accept arm joints in the live URDF and feedback, but command
only the legs, including when applying gains or requesting the final hold.
Set and hold the arms separately at the posture above. This candidate was tested
with rigid arms in simulation; real arm servo holding remains unvalidated.

## Motion specifications and explanation

Both working candidates use these settings:

| Specification | Value |
| --- | --- |
| Leg order | `0321`: left-front → right-rear → left-rear → right-front |
| Actual sequence | Three repetitions of `0321`: 12 swings, one foot at a time |
| Timing | 1 s initial hold; 0.8 s swings; 0.4 s between swings; 0.08 s after the last swing |
| End of motion | 15.08 s, after the third right-front landing; planned final velocity is zero |
| Foot advancement | 35 cm forward per foot swing |
| Obstacle | 30 cm high; front edge at `x = 0.60 m` in the trajectory frame |
| Initial obstacle distance | About 23.2 cm ahead of the front-foot contact centres at the MAT's initial posture |
| Foot clearance setting | 10 cm |
| MAT sampling | 40 ms between nodes, interpolated by the replayer at 500 Hz |

**Balance constraint.** During a swing, the other three feet form the support
triangle. The optimizer keeps the horizontal projection of the whole robot's
centre of mass (COM), including the payload and any fixed arms, at least 2 cm
inside each edge at the optimization nodes. It also checks the node immediately
before lift-off, so the body can transfer weight before lifting a foot. Landing
targets and swing timing stay fixed while the optimizer adjusts body and leg
posture. URDF joint-position bounds are enforced; no added torque-minimization
cost is enabled.

The 2 cm bound applies at the 40 ms nodes. The interpolated reference reaches
smaller margins: **0.75 cm without arms / 0.76 cm with fixed arms**. This is a
geometric balance measure, not a guarantee against falling on the real robot.
Neither motion is constrained to finish in an exact homing posture.

**Torque figures.** Predicted peak is the optimizer's inverse-dynamics torque.
Measured peak is the applied MuJoCo joint effort, including feedforward and
position/velocity feedback. Both working motions reach the normal **185 Nm**
limit in simulation; the lower predicted peaks do not establish a lower applied
torque limit. The replay follows saved leg references and does not correct body
balance from IMU or contact feedback.

<details>
<summary>Older comparisons and historical motions</summary>

### Without arms — earlier comparisons

These centred-payload motions completed recorded MuJoCo climbs. Detailed
optimization changes and limitations are recorded in [candidates.json](candidates.json).

| Motion | Duration | Predicted peak | Measured peak | Peak joint error | Set MAT |
| --- | ---: | ---: | ---: | ---: | --- |
| [Full 0321, fixed lateral landings](step_up_30kg_centered_step30cm_order0321_landing_y_no_torque_cost_20261026.mat) | 20.28 s | 111.4 Nm | 136.6 Nm | 4.3° | `MAT="/workspace/kyon_actions/mat_files/candidates/step_up_30kg_centered_step30cm_order0321_landing_y_no_torque_cost_20261026.mat"` |
| [Full 0321, COM margin 1 cm](step_up_30kg_centered_step30cm_order0321_com_margin1cm_20261026.mat) | 20.28 s | 116.0 Nm | 142.2 Nm | 5.7° | `MAT="/workspace/kyon_actions/mat_files/candidates/step_up_30kg_centered_step30cm_order0321_com_margin1cm_20261026.mat"` |
| [Full 0321, joint limits](step_up_30kg_centered_step30cm_order0321_jointlimits_no_torque_cost_20261026.mat) | 20.28 s | 111.4 Nm | 135.9 Nm | 3.2° | `MAT="/workspace/kyon_actions/mat_files/candidates/step_up_30kg_centered_step30cm_order0321_jointlimits_no_torque_cost_20261026.mat"` |
| [Original full 0321](step_up_30kg_centered_step30cm_clearance10cm_20261026.mat) | 20.28 s | 111.4 Nm | 131.2 Nm | 4.0° | `MAT="/workspace/kyon_actions/mat_files/candidates/step_up_30kg_centered_step30cm_clearance10cm_20261026.mat"` |
| [Front-right finishing step, 0321](step_up_30kg_centered_step30cm_front_right_finish_20261026.mat) | 15.08 s | 117.8 Nm | 130.7 Nm | 4.3° | `MAT="/workspace/kyon_actions/mat_files/candidates/step_up_30kg_centered_step30cm_front_right_finish_20261026.mat"` |
| [Front-right finish, body tracking + torque cost](step_up_30kg_centered_step30cm_front_right_finish_bodytrack_tau001_20261026.mat) | 15.08 s | 115.6 Nm | 163.1 Nm | 5.0° | `MAT="/workspace/kyon_actions/mat_files/candidates/step_up_30kg_centered_step30cm_front_right_finish_bodytrack_tau001_20261026.mat"` |
| [Front-right finish, force smoothing](step_up_30kg_centered_step30cm_front_right_finish_force_smooth003_20261026.mat) | 15.08 s | 116.5 Nm | 154.3 Nm | 10.5° | `MAT="/workspace/kyon_actions/mat_files/candidates/step_up_30kg_centered_step30cm_front_right_finish_force_smooth003_20261026.mat"` |
| [Front-right finish, 0.6 s support](step_up_30kg_centered_step30cm_front_right_finish_stance600ms_20261026.mat) | 17.28 s | 120.9 Nm | 145.8 Nm | 3.0° | `MAT="/workspace/kyon_actions/mat_files/candidates/step_up_30kg_centered_step30cm_front_right_finish_stance600ms_20261026.mat"` |
| [0321 then 1032, earlier right-front steps](step_up_30kg_centered_step30cm_order0321_then1032_20261026.mat) | 15.08 s | 131.2 Nm | 146.4 Nm | 4.5° | `MAT="/workspace/kyon_actions/mat_files/candidates/step_up_30kg_centered_step30cm_order0321_then1032_20261026.mat"` |
| [0321 / 0123 / 1032, front-first follow-up cycles](step_up_30kg_centered_step30cm_order0321_0123_1032_20261027.mat) | 15.08 s | 133.3 Nm | 139.3 Nm | 6.0° | `MAT="/workspace/kyon_actions/mat_files/candidates/step_up_30kg_centered_step30cm_order0321_0123_1032_20261027.mat"` |
| [0321 / 0123 / 1032, centred finish](step_up_30kg_centered_step30cm_order0321_0123_1032_final_y0_20261027.mat) | 15.08 s | 130.0 Nm | 150.2 Nm | 3.7° | `MAT="/workspace/kyon_actions/mat_files/candidates/step_up_30kg_centered_step30cm_order0321_0123_1032_final_y0_20261027.mat"` |
| [0321 / 0123 / 1032, centred finish and heading](step_up_30kg_centered_step30cm_order0321_0123_1032_final_y0_yaw0_20261027.mat) | 15.08 s | 125.7 Nm | 142.1 Nm | 3.8° | `MAT="/workspace/kyon_actions/mat_files/candidates/step_up_30kg_centered_step30cm_order0321_0123_1032_final_y0_yaw0_20261027.mat"` |
| [Front-right finish, first two RF landings +10 cm](step_up_30kg_centered_step30cm_front_right_finish_rf_forward10cm_20261026.mat) | 15.08 s | 117.7 Nm | 134.8 Nm | 3.8° | `MAT="/workspace/kyon_actions/mat_files/candidates/step_up_30kg_centered_step30cm_front_right_finish_rf_forward10cm_20261026.mat"` |
| [Front-right finish, body height +5 cm](step_up_30kg_centered_step30cm_front_right_finish_body_height5cm_20261027.mat) | 15.08 s | 106.1 Nm | 128.5 Nm | 4.0° | `MAT="/workspace/kyon_actions/mat_files/candidates/step_up_30kg_centered_step30cm_front_right_finish_body_height5cm_20261027.mat"` |
| [Stop after climb, 0321](step_up_30kg_centered_step30cm_stop_after_climb_20261026.mat) | 13.88 s | 120.4 Nm | 137.3 Nm | 3.6° | `MAT="/workspace/kyon_actions/mat_files/candidates/step_up_30kg_centered_step30cm_stop_after_climb_20261026.mat"` |
| [Short order 2301](step_up_30kg_centered_step30cm_order2301_20261026.mat) | 15.08 s | 105.3 Nm | 148.9 Nm | 3.1° | `MAT="/workspace/kyon_actions/mat_files/candidates/step_up_30kg_centered_step30cm_order2301_20261026.mat"` |
| [Short order 0123](step_up_30kg_centered_step30cm_order0123_20261026.mat) | 15.08 s | 130.9 Nm | 155.7 Nm | 3.2° | `MAT="/workspace/kyon_actions/mat_files/candidates/step_up_30kg_centered_step30cm_order0123_20261026.mat"` |
| [Full order 2301](step_up_30kg_centered_step30cm_order2301_full_20261026.mat) | 20.28 s | 101.6 Nm | 159.0 Nm | 7.3° | `MAT="/workspace/kyon_actions/mat_files/candidates/step_up_30kg_centered_step30cm_order2301_full_20261026.mat"` |

### Historical rear-payload files

Already present in `mat_files/`; no duplicate copies are needed. These used
`[-0.20, 0, 0.20]` m payload mounting and an experimental **115 Nm applied-torque
cap**. Their results do not establish performance with the centred payload:

- [optimized_v2](../step_up_30kg_optimized_v2.mat) — `MAT="/workspace/kyon_actions/mat_files/step_up_30kg_optimized_v2.mat"`: 1 climb; nonconverged feasible checkpoint.
- [optimized_seed](../step_up_30kg_optimized_seed.mat) — `MAT="/workspace/kyon_actions/mat_files/step_up_30kg_optimized_seed.mat"`: 1 climb; nonconverged checkpoint.
- [candidate_v1](../step_up_30kg_candidate_v1.mat) — `MAT="/workspace/kyon_actions/mat_files/step_up_30kg_candidate_v1.mat"`: 3 climbs, no feedforward; manually corrected, reference-clearance check fails.

</details>

[candidates.json](candidates.json) records file hashes, solver status, model and
replay conditions. Evidence paths refer to local development artifacts; logs,
graphs and simulator files are not shipped with the replay image.
