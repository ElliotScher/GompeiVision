# Stage 5: CAN-FD corners, robot-side PnP and MVP parity

## Goal

**MVP:** the module is a drop-in replacement for the CPU pipeline for AprilTag localization. The tag path runs in PL and RPU and sends corners over CAN-FD, and robot code on the SystemCore solves the pose with our robot library. Module-side poses over NT keep existing robot code and dashboards working.

## Deliverables

- [RPU] corner undistortion with the cached camera calibration, and the [CAN-FD corner frame](../PLAN.md#interfaces-and-message-formats) sender through a [PL] CAN-FD controller (see [CAN-FD tag path](../PLAN.md#can-fd-tag-path)), including multi-part frames, slot rotation and the zero-tag heartbeat.
- **Robot library** (`robot/`, Java first): reads every new CAN-FD part each loop, computes capture times from the age field, and solves PnP (see [Robot-side PnP](../PLAN.md#robot-side-pnp)):
  - IPPE-square per tag, multi-tag Gauss-Newton refinement on reprojection error;
  - picking between the two single-tag solutions, or a heading-constrained solve, using the gyro heading at capture time from the pose estimator's history;
  - quality gates, standard deviations and `addVisionMeasurement()`, following the [one-loop robot contract](../PLAN.md#one-loop-robot-contract);
  - preallocated arrays, no per-frame object creation.
- [RPU] module-side PnP: IPPE-square (both solutions) and SQPnP, plus tag angles, ported from [`CameraPoseEstimator.cpp`](../../src/cpp/estimator/CameraPoseEstimator.cpp), [`SingleTagPoseEstimator.cpp`](../../src/cpp/estimator/SingleTagPoseEstimator.cpp) and [`TagAngleCalculator.cpp`](../../src/cpp/estimator/TagAngleCalculator.cpp).
- [APU] real-time publisher thread sending `observations` packed exactly like [`OutputPublisher.cpp`](../../src/cpp/io/OutputPublisher.cpp), plus `connected`, `capture_fps` and `processing_fps`.
- NAPA as the module's fallback detector (a config switch, no reflash, same CAN-FD frames) and a 1-in-N background cross-check against the FPGA detector, with mismatches on the status topic.
- A desktop replay harness for the robot library: feeds logged CAN-FD frames and robot odometry through the solver, for testing without a robot.

## Exit criteria

- [ ] Robot-library poses match the golden model within a stated tolerance on all datasets, and agree with module-side poses.
- [ ] Auto-align on the practice robot works with the robot library (CAN-FD), and unchanged CPU-pipeline robot code works with the NT `observations`.
- [ ] LED-rig latency and jitter measured for corners at the SystemCore, and compared with the Stage 1 CPU numbers.
- [ ] Tag-path latency unchanged (within a stated margin) while the APU is fully loaded and while Linux reboots.
- [ ] Robot-side time per loop (CAN reads + PnP + `addVisionMeasurement()`) measured in Java at the worst-case tag count, with nothing carried to the next loop except pose estimator state. Decision recorded on whether the solver moves to C++ over JNI.
- [ ] Heading-constrained single-tag solve compared against picking between IPPE solutions on recorded fast-turn datasets.
- [ ] CAN-FD bus load measured for one module.
- [ ] 1-hour soak on the robot with no resets, dropped connections or missed frames.
- [ ] Switching a module to the NAPA fallback works on the robot, and the cross-check mismatch rate on recorded matches is recorded.
- [ ] Decision recorded: is the module ready to be the competition system?

## Risks

- Parity in the lab but not on the robot (vibration, lighting, brownouts). The A/B test on the robot is what counts.
- If SystemCore's CAN API only keeps the newest packet per ID and slot rotation turns out unreliable, fall back to fusing NT `observations` and record why.
