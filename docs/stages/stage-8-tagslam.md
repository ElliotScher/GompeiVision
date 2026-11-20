# Stage 8: TagSLAM field calibration

## Goal

A desktop tool that refines the AprilTag map for a specific field from logged module corners and robot odometry, so robot code and the modules solve against where the tags really are.

## Deliverables

- `tools/tagslam/`: a factor graph over robot poses and tag landmarks, built on an existing factor-graph library, with reprojection factors from logged corners (all modules), odometry factors from the robot's WPILOG, and priors from the official field layout.
- Log import from the robot's WPILOG (CAN-FD corners as read by the robot library) and the modules' recording metadata.
- A field-calibration procedure: drive a set path on a new field (practice field or event field during calibration time), run the tool, review the per-tag changes.
- Output: a refined field layout loaded into robot code, and sent to the modules over `/field/tag_<id>` for module-side PnP. Saved on the module's eMMC for boot.
- A report per run: tags that moved, by how much, and the residual error.
- Optional, only if it proves useful: running the same code on the APU at normal priority for live map refinement.

## Exit criteria

- [ ] On recorded datasets, the refined map is closer to surveyed ground truth than the official layout when tags have been moved or mounted badly.
- [ ] Robot-library pose error on a held-out dataset is lower with the refined map than with the official one.
- [ ] The field-calibration procedure runs in under 15 minutes from driving to a loaded map.

## Risks

- Bad odometry (wheel slip) during the calibration drive. Drive slowly, and weight odometry factors accordingly.
- A refined map that's wrong is worse than the official one. Keep the official layout as the default and require a review step before loading a refined map.
