# Stage 10: VSLAM

## Goal

Visual SLAM from each module's stereo pair, producing VSLAMPose and a LocalGeometricMap that can be saved and loaded as a prior for VSLAM and MCL.

## Deliverables

- [APU] VSLAM built on an existing open-source stereo SLAM system, fed frames and hardware timestamps from the frame store.
- Desktop prototype on recorded stereo datasets first, then port to the APU.
- Optional [PL] FAST corners and ORB descriptors to offload feature extraction, if the A53s are too slow.
- Map save to MapStorage and load as SavedVSLAMMap; prior for MCL.
- Mono VSLAM only when manually triggered.

## Exit criteria

- [ ] Trajectory error on recorded datasets measured against ground truth and compared with TagSLAM.
- [ ] Update rate on the APU measured and recorded, with and without PL features.
- [ ] Save → reboot → load → relocalize works.
- [ ] Running VSLAM doesn't change tag-path latency.

## Risks

- VSLAM on FRC fields (repetitive, moving robots, changing lighting) is hard on any hardware. A negative result here is acceptable and doesn't block other stages.
- The A53s may be too slow even with PL features. The documented fallback is the separate-CPU hardware option in [PLAN.md](../PLAN.md#candidates).
