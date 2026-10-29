# Lite3 MuJoCo dynamics probes

These self-authored runners consume external Lite3 MJCF/mesh and ONNX policy
assets by path. They never overwrite those assets. The environment must already
provide MuJoCo, NumPy, and ONNX Runtime. Results describe simulation only.

## Obstacles on a step face

`mj_step_obstacles.py` composes a 0.06 m high deck with three declared-map cases:
`clear` (no box), `offset_box` (room to route around a box), and `blocked_box`
(a box spans the entire deck). The robot starts **on the deck**; this isolates
step-face obstacle planning/execution from the existing rear-foot ascent defect.
It does not prove climbing or descending a step.
The planning domain contains only the deck top, with no declared edge to the
surrounding ground. The full-width box is no-route within this domain; it does
not establish that a robot could never leave the deck and return by another way.

```bash
python3 -B mj_step_obstacles.py \
  --xml /path/to/original/flat/Lite3.xml \
  --policy /path/to/policy.onnx \
  --case all --seconds 24 --out /tmp/step-obstacles.json
```

The temporary composed model is compiled before planning. Its actual world geom
bounds supply both the top support footprint and `map.v2.obstacles`; boxes are
not inserted only into an evaluator. The planner receives a conservative square
XY envelope with radius 0.5 m and height 0.5 m above support. The support footprint
is inset by that radius from the physical deck edge. Every physics sample checks
the actual collision geometry remains within this declared envelope.

The positive cases require a nonzero command, physical progress, arrival,
four feet remaining on the top surface, no fall, and no box contact or robot
collision-AABB overlap. The blocked case requires `E_PLAN_NO_ROUTE` from the
global planner, exactly zero commands throughout, and less than 0.10 m drift.
The drift bound applies to the maximum XY excursion over the entire trace, not
the final displacement: a robot that moves out and returns still fails. Both
the controller history and commands recorded at each physics step must be zero;
an absent controller history cannot establish zero commands. Numeric shapes,
real types, finite values, ordered timestamps, geom bounds, pose, feet, contacts,
torques, velocities, and commands are validated before comparisons are evaluated.
In particular, NaN collision bounds cannot pass by making comparisons false.
Rejected input or other planner errors cannot count as successful blockage.
The same ONNX -> joint PD -> limited torque -> contact dynamics chain runs in
all three cases; the runner never moves a base or sets its velocity after the
one initial pose assignment.

The JSON records each physics step, commands, the complete global route, compiled
fixture geometry, map input, asset entry hashes, and separate checks. Input XML
hashes do not cover referenced meshes. The original cross-step baseline remains
`NOT_PASSED` (R855); `ascent_completed=false` and
`four_foot_ascent_acceptance=NOT_TESTED` are explicit. These are known-map cases,
not perception, ROS integration, true contact-layer transfer, or #151 evidence.

Pure negative controls (no physics engine or external assets required):

```bash
python3 -B -m unittest test_mj_step_obstacles test_mj_braking test_mj_navigation_stop
```

The tests reject endpoint-only collision checks, missing geometry, unexpected
planner errors, nonzero commands after refusal, false ascent claims, reused step
worlds that would stack fixtures, and an actual body exceeding the planning
envelope. Target-environment evidence must additionally run the command above;
the pure tests do not certify physical navigation.

## DreamWaQ 270-state adapter

`mj_dreamwaq_walk.py` consumes a 270-input DreamWaQ-style policy: 45 current
proprioceptive values plus five 45-value history frames (newest first), encoded
to a 19-value latent; the actor outputs 12 joint actions. It is not the 45-input
flat policy and the 225 history is not a height scan. Defaults follow the Lite3
history-5 training configs: pose `[0,-1.0,1.8]x4`, action scale
`[0.125,0.25,0.25]x4`, command scale `[2,2,0.25]`, 50 Hz, Kp 25, Kd 0.5,
30 N.m torque limit. The adapter can generate a temporary 3-step world from a
flat MJCF; it never edits the external model.

```bash
python3 -B mj_dreamwaq_walk.py --policy /path/to/lite3_walk_policy.onnx \
  --xml /path/to/Lite3.xml --mode stairs --step-h .06 \
  --seconds 10 --v-cmd .3 --out /tmp/dreamwaq-stairs.json
```

`stand`/`walk` use the flat XML. `stairs` reports `crossed_once` (xmax past the
top without falling and with |y|<1 m at each step boundary) and `stable_finish`
(pass plus final x past the top and |final y| within half the stair width);
they are different claims. This is MuJoCo torque dynamics only: no real robot,
no ROS execution, no #151 budget, and no contact-state acceptance.

Heading hold (`--heading-hold`) adds a bounded body yaw-rate command
`wz = clip(-yaw_gain*yaw - y_gain*y, +/-max_wz)` while keeping `vx`; R884 used
`--yaw-gain 2 --y-gain 1 --max-wz .5`. On the flat walk it reduced max|y| from
0.1429 to 0.0478; on 3-step 6/10/15 cm staircases all `stable_finish` became
true (15 cm had fallen without heading hold). This loop consumes simulated
world yaw/lateral position, so it is a MuJoCo command-level correction, not a
real-robot state-estimation result.
