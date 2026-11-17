# Stage 6: Multiple cameras and synced stereo

## Goal

2–4 cameras per unit, including a hardware-synchronized stereo pair with StereoTagPnP.

## Deliverables

- [PL] multiple capture and tag pipelines (shared or duplicated per camera, based on the resource budget).
- One trigger line for stereo pairs, and a check that their timestamps match.
- Stereo calibration flow (intrinsics + extrinsics) and storage.
- [RPU] StereoTagPnP: triangulate matched corners, then joint PnP.

## Exit criteria

- [ ] All cameras at target FPS simultaneously, with no drops for 1 hour.
- [ ] Stereo pair timestamps within 10 µs of each other.
- [ ] StereoTagPnP is more accurate at range than mono on recorded datasets, measured against ground truth.

## Risks

- Resource use grows with camera count; time-multiplexing the tag pipeline between cameras may be needed.
