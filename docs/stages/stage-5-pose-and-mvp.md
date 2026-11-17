# Stage 5: Pose estimation and MVP parity

## Goal

**MVP:** the unit is a drop-in replacement for the CPU pipeline for AprilTag localization, with the whole tag path in PL and RPU.

## Deliverables

- [RPU] IPPE-square (single tag, both solutions) and SQPnP (multi-tag), corner undistortion and tag angles, ported from [`CameraPoseEstimator.cpp`](../../src/cpp/estimator/CameraPoseEstimator.cpp), [`SingleTagPoseEstimator.cpp`](../../src/cpp/estimator/SingleTagPoseEstimator.cpp) and [`TagAngleCalculator.cpp`](../../src/cpp/estimator/TagAngleCalculator.cpp).
- [APU] real-time publisher thread sending `observations` packed exactly like [`OutputPublisher.cpp`](../../src/cpp/io/OutputPublisher.cpp), plus `connected`, `capture_fps` and `processing_fps`.
- Decision, backed by measurements, on whether the raw-UDP fast path ([PL] Ethernet MAC + [RPU]) is needed.

## Exit criteria

- [ ] Poses match the golden model within a stated tolerance on all datasets.
- [ ] Unchanged robot code runs auto-align with the unit on the practice robot.
- [ ] LED-rig latency and jitter measured and compared with the Stage 1 CPU numbers.
- [ ] Tag-path latency unchanged (within a stated margin) while the APU is fully loaded.
- [ ] 1-hour soak on the robot with no resets, dropped connections or missed frames.
- [ ] Decision recorded: is the unit ready to be the competition system?

## Risks

- Parity in the lab but not on the robot (vibration, lighting, brownouts). The A/B test on the robot is what counts.
