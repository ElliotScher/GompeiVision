# Stage 5: Pose estimation and MVP parity

## Goal

**MVP:** the unit is a drop-in replacement for the CPU pipeline for AprilTag localization, with the whole tag path in PL and RPU.

## Deliverables

- [RPU] IPPE-square (single tag, both solutions) and SQPnP (multi-tag), corner undistortion and tag angles, ported from [`CameraPoseEstimator.cpp`](../../src/cpp/estimator/CameraPoseEstimator.cpp), [`SingleTagPoseEstimator.cpp`](../../src/cpp/estimator/SingleTagPoseEstimator.cpp) and [`TagAngleCalculator.cpp`](../../src/cpp/estimator/TagAngleCalculator.cpp).
- [APU] real-time publisher thread sending `observations` packed exactly like [`OutputPublisher.cpp`](../../src/cpp/io/OutputPublisher.cpp), plus `connected`, `capture_fps` and `processing_fps`.
- Decision, backed by measurements, on whether a fast path is needed, and which: raw UDP ([PL] Ethernet MAC + [RPU]) or CAN-FD to SystemCore ([PL] CAN-FD controller + [RPU], see [PLAN.md](../PLAN.md#can-fd-fast-path)).
- Robot-side reference code: read every queued measurement each loop, pick between single-tag solutions using the gyro history, and fuse each measurement with its exposure timestamp, following the [one-loop robot contract](../PLAN.md#one-loop-robot-contract).

## Exit criteria

- [ ] Poses match the golden model within a stated tolerance on all datasets.
- [ ] Unchanged robot code runs auto-align with the unit on the practice robot.
- [ ] LED-rig latency and jitter measured and compared with the Stage 1 CPU numbers.
- [ ] Tag-path latency unchanged (within a stated margin) while the APU is fully loaded.
- [ ] Robot-side vision processing time per loop measured at the worst-case measurement count, with nothing carried to the next loop except pose estimator state.
- [ ] 1-hour soak on the robot with no resets, dropped connections or missed frames.
- [ ] Decision recorded: is the unit ready to be the competition system?

## Risks

- Parity in the lab but not on the robot (vibration, lighting, brownouts). The A/B test on the robot is what counts.
