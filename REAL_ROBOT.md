# Real robot: direct ROS 2 replay

Use [docker/README.md](docker/README.md): **home → replay**, in one terminal.
XBotCore must already be running. Do not launch `kyon_controller` or another
command publisher alongside this replayer.

## Setup

- Match `ROS_DOMAIN_ID` to XBotCore; 77 was only the isolated simulation domain.
- Homing and replay command only the 12 leg joints. Arms may be present in the
  live URDF and feedback; no arm references or gains are sent. Match the arms
  and 30 kg payload setup to the MAT. For a fixed-arm MAT, position and hold the
  arms separately at its documented posture before homing and replay.
  Use feedforward only for a matching robot and payload; model signatures and
  payload mass are not checked. Live joint limits are checked.
- Select the same MAT with `-f "$MAT"` for both homing and replay. Centred
  candidates require payload mount `[0, 0, 0.20]` m relative to the pelvis;
  omitting `-f` uses the older rear-payload default.
- Homing preserves the live stiffness/damping and moves to the MAT's initial
  posture over five seconds and reports the final joint error. Replay starts
  directly from the MAT.
  The publisher rejects stale feedback; it does not detect competing publishers.
- `replay --config ...` sets the provided stiffness/damping. Omitting it retains
  the current gains. The included K=2000/D=120 profile was tested in simulation.
- Hardware control modes, motor limits and the stop procedure remain in your
  existing XBotCore setup. The replay image does not change the URDF or install
  a 100 Nm effort ceiling.

## Inspect

Inside the container, inspect current joint positions, velocities, efforts and gains:

```bash
ros2 topic echo /xbotcore/joint_states --once
```

## Stopping

Normal completion, Ctrl+C and handled errors request a position hold with zero
velocity and feedforward effort. Gains remain at their last setting. The
publisher also checks feedback age against a 0.5 s wall-time limit.

There is no intermediary controller continuously forwarding the last message.
If the publisher is forcibly killed, it cannot send its final hold; the response
then depends on XBotCore and the hardware's configured failsafe. A position hold
is not balance recovery or an emergency stop. Use the robot's established stop
procedure. Physical robot operation has not been validated.
