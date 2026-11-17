# Stage 9: MCL localization

## Goal

Monte Carlo localization that keeps a global pose hypothesis (including after being bumped or in tag dead zones) and publishes MCLPose.

## Deliverables

- Motion model from RobotData (from Stage 8's shared-memory input).
- [PL] particle scoring engine: project expected tags into each camera per particle and score against detections, many particles in parallel.
- [RPU] resampling, pose and covariance output, published through the APU like tag measurements.
- Mono input only when manually triggered, stereo continuously, as in the structure diagram.
- Hook for the saved VSLAM map as a prior (used once Stage 10 exists).
- A desktop C++ model of the whole filter, used to tune the observation model before building hardware.

## Exit criteria

- [ ] Recovers from a "kidnapped robot" test (robot moved while blind) within a stated time.
- [ ] Particle count and update rate measured and recorded.
- [ ] Scoring engine matches the desktop model on recorded data.

## Risks

- Observation model quality matters more than particle count. Tune on desktop replays before building hardware.
