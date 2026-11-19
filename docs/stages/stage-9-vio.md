# Stage 9: Stereo VIO

## Goal

Stereo visual-inertial odometry (VIO) on each module: position change and velocity that stay accurate for a few seconds without tags, including while the robot is being pushed, so shooting on the move keeps working under heavy defense. Tags (Stages 5 and 8) correct VIO's slow drift, so no map is built.

## Deliverables

- [PL] feature detection (FAST-style corners) and frame-to-frame tracking (KLT-style) for both lenses at full frame rate, with stereo matches for depth.
- [PL] onboard IMU sampled at ≥1 kHz and timestamped by the same counter as the camera exposures.
- [APU] VIO back end: a lightweight filter or small sliding-window estimator (MSCKF-style), built on an existing open-source VIO system where possible, fed tracked features, stereo depth and IMU data.
- Outlier rejection for moving objects (defenders): features whose motion disagrees with the IMU, and very close features during contact, are down-weighted.
- VIO pose deltas and velocity published per frame with exposure timestamps, following the [one-loop robot contract](../PLAN.md#one-loop-robot-contract).
- Robot-side reference code: VIO velocity used for shoot-on-the-move, and wheel-slip detection (IMU acceleration against wheel speeds, current spikes) that raises odometry standard deviations.
- Recorded defense and pushing datasets (Stage 3's dataset mode) with ground truth, and a desktop build of the back end for replaying them.

## Exit criteria

- [ ] Position and velocity error over 2–5 s gaps with no tags measured on the defense datasets, compared with wheel odometry and with QuestNav on the same robot.
- [ ] Update rate and A53 CPU use measured and recorded, with PL feature tracking.
- [ ] Shooting-on-the-move test under contact: hit rate with VIO fused against without.
- [ ] Running VIO doesn't change tag-path latency.

## Risks

- A defender filling the view breaks the static-world assumption. Mitigations: modules facing different directions, IMU-based outlier rejection, stereo depth; consider an upward-angled module that sees static venue structure.
- If the A53s are too slow for the back end, the fallback is the faster-CPU hardware option in [PLAN.md](../PLAN.md#candidates).
- Full VSLAM (mapping, loop closure) is out of scope: the tags are the map.
