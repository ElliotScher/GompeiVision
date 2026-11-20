# Stage 6: Stereo module and multi-module fusion

## Goal

Both of a module's cameras running as a hardware-synchronized stereo pair with StereoTagPnP, and several modules working together on one robot.

## Deliverables

- [PL] two capture and tag pipelines, one per lens (shared logic or duplicated, based on the resource budget).
- One trigger line for the pair, and a check that their timestamps match.
- Stereo calibration flow (intrinsics + extrinsics) with ChArUco boards and OpenCV, run in setup mode on the module or on a laptop from recorded frames, and storage.
- [RPU] StereoTagPnP: triangulate matched corners, then joint PnP.
- Module identity: the board ID sets NT topics and the CAN ID, so a module can be swapped in the pits with config following its mounting position.
- Robot-side fusion reference code: each module's field-frame measurements fused in the pose estimator with their own timestamps and standard deviations, with all modules sharing one CAN-FD vision bus. See [Multiple modules](../PLAN.md#multiple-modules).

## Exit criteria

- [ ] Both lenses at target FPS simultaneously, with no drops for 1 hour.
- [ ] Stereo pair timestamps within 10 µs of each other.
- [ ] StereoTagPnP is more accurate at range than mono on recorded datasets, measured against ground truth.
- [ ] Depth error against distance measured for the module's baseline and recorded here.
- [ ] Two modules on one robot: the fused pose is more accurate than either module alone on recorded datasets, and the shared CAN-FD bus load is measured.

## Risks

- Two tag pipelines plus features may be tight on the K26; time-multiplexing one tag pipeline between the two lenses is the fallback.
- The short baseline of a Limelight-sized module limits stereo depth at range (see [PLAN.md](../PLAN.md#sensors)).
