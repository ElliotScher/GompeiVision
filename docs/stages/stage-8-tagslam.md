# Stage 8: TagSLAM and refined tag map

## Goal

Estimate robot pose and refine the AprilTag map from tag detections and robot odometry, and publish a refined map.

## Deliverables

- RobotData input topic (heading, module positions) from robot code, placed in shared memory for other blocks.
- [APU] fixed-lag factor-graph smoother using an existing library: reprojection, odometry and field-layout prior factors.
- RobotPoseEstimate on PoseMeasurements; RefinedTagMap on a MapData topic.
- Save and load the refined map to storage.
- The same code builds on a desktop for testing on recorded datasets.
- One instance per module, anchoring that module's VIO track and poses to the field frame.
- Heading for pose disambiguation (see [PLAN.md](../PLAN.md#heading-and-pose-disambiguation)): RobotData carries heading and yaw rate, sent every robot loop. Optional heading-constrained single-tag solve on the [RPU], using the onboard IMU with a quality-gated heading filter.

## Exit criteria

- [ ] On recorded datasets, the refined map is closer to surveyed ground truth than the official layout when tags have been moved or mounted badly.
- [ ] Update rate and latency measured and recorded (target: pose ≥10 Hz, map ≥1 Hz).
- [ ] Running TagSLAM doesn't change tag-path latency.
- [ ] If the heading-constrained solve is built: single-tag pose error on recorded fast-turn datasets compared with robot-side disambiguation.

## Risks

- The A53s may be too slow for the window size we want. Shrink the window or lower the update rate, and record the measurement here.
