# Stage 1: Latency baseline and golden models

## Goal

Measure the CPU pipeline's real latency, and turn its code into golden models and datasets that the FPGA blocks are tested against.

## Deliverables

- **LED latency rig:** robot code toggles an LED in view of the camera and logs the time; the vision output timestamp is compared with the toggle time. Works for both CPU and FPGA systems.
- Measured exposure-to-robot latency and jitter (median, p95, p99) for the CPU pipeline at the competition resolution and FPS, recorded in this doc.
- Recorded datasets: raw frames plus CPU detections and poses, from practice fields and match-like motion (still, driving, fast turns), stored outside git with a manifest in `model/datasets/`.
- `model/`: C++ golden models for the threshold, connected components, quad fit, decode, refinement and PnP steps, built from [`src/`](../../src/) and the AprilTag library, with an option to dump intermediate results at each step.
- A replay tool: run any dataset through the models and diff the result against the CPU output.
- Golden models for the quad-finding options in [PLAN.md](../PLAN.md#apriltag-detector): 4A (boundary clustering + quad fit), 4B (corner scorecard + edge refinement) and 4C (contour tracer), plus whole-detector references: AprilTag 3 (full resolution and WPILib defaults), NAPA, OpenCV ArUco (36h11 dictionary) and ArUco Nano. A report on the recorded datasets compares detection rate, false IDs, corner error, speed and failure cases (blur, merged with background, occlusion, steep angles) for each.
- Auto-labeled datasets: frames where AprilTag (full resolution), NAPA and ArUco agree become ground truth; disagreements are flagged for review.

## Exit criteria

- [ ] Latency numbers for the CPU system measured and written down.
- [ ] Golden models reproduce the CPU pipeline's detections and poses on all datasets (identical IDs and corners; poses within a stated tolerance).
- [ ] Intermediate-step dumps exist for every block that Stage 4 will build.
- [ ] Quad-finding decision for Stage 4 recorded, with the numbers behind it. **Provisional (2026-11-21): 4B alone, with fallback LUTs reserved, from the synthetic comparison below. Confirm on recorded datasets, and decide then whether 4A or 4C is worth adding.**

## Progress (2026-11-20)

The golden models that need no recordings have started in [`model/`](../../model/README.md). There are 53 GoogleTest cases, which pass under GCC 13 and 16 and are clean under ASan, UBSan and TSan.

- **Decimation and adaptive threshold:** bit-exact with AprilTag v3.4.5 (the CPU pipeline's version since 2026-11-20; v3.3.0 gave identical results), with the tile statistics as intermediates.
- **Streaming connected components:** the same partition as AprilTag's single-threaded labeling. Per-component accumulators (area, bounding box, extreme pixel in 16 directions) are emitted as each component closes.
- **Quad finding 4B (corner scorecard):** a rough quad from the 16 extremes, with sanity checks that send failures to 4C.
- **Edge refinement:** subpixel corners from the rough quad, using the chosen window rule below.
- **Decode:** the homography, brightness model and bit sampling, following AprilTag's `quad_decode()`.
- **4C contour tracer (simplified):** outer boundary, hull, largest-area quad and an edge-fit check.
- **Replay tool:** `gv-model replay SESSION` runs the golden detector over a recorded session and diffs it against the CPU's detections, with an optional per-frame CSV.
- **Whole golden detector:** every block chained, compared with AprilTag 3 below.
- **Dataset recorder in the CPU pipeline:** off unless `GOMPEI_RECORD_DIR` is set. Format in [`model/datasets/`](../../model/datasets/README.md).
- **PnP:** IPPE-square, matching OpenCV to 1e-6 including solution order and errors, plus multi-tag LM refinement and the WPILib conversion.
- **ID lookup:** NAPA's four 9-bit block tables with up to 3 corrected bits. A third bit is accepted only when the wrong cells touch; 420 of the 7,140 three-cell placements qualify, matching NAPA.
- **`gv-model dump`:** writes every intermediate, plus the 4B verdict for each component.

**First 4B numbers (synthetic scenes only):**

| Setup | Refinable rough quad | Silent misses | Sent to 4C |
|---|---|---|---|
| 1280×800, decimate 4 (CPU config) | 95.1% of 162 tags | 1 | 7 |
| 640×400 full resolution | 95.2% of 168 tags | 1 | 7 |

- "Refinable" means every corner is within max(2 px, 5% of the side), assuming step 5 searches about ±3 px along each edge normal.
- A silent miss is a wrong quad that passes the sanity checks, so the tag is lost unless 4C also runs.
- **Defaults set by sweeps:**
  - **Off-quad limit** (any extreme outside the quad by more than max(2.0 px, 5% of the mean side) is rejected). A fixed 1.0 px falsely rejected 13 of 162 decimated tags, and rounded corners pushed 50 of 168 full-resolution tags out. A 1.5 px floor sent 18% of 20–40 px tags to 4C; 2.0 sends 4%, with no wrong detections anywhere.
  - **Fill ≤ 0.95** of the quad, since a tag's black region is at most 54/64.
  - **Minimum side 8 px:** a tag is 8 cells across, so nothing smaller decodes. This cut stray accepted quads from about 430 to about 15 per full-resolution frame, without losing a tag.
- **Sensor noise matters:** with ±6 noise, more than the threshold's `min_white_black_diff` of 5, flat background binarizes into speckle and doubles the strays.
- Recorded datasets are still needed before the 4A/4B decision.

**Whole golden detector vs AprilTag 3** (synthetic scenes; AprilTag configured as the CPU pipeline runs it):

| Scenario | Golden found | AprilTag found | False detections | Golden corner error (median / p95) | AprilTag corner error |
|---|---|---|---|---|---|
| CPU config, decimate 4 | 117 of 123 | 118 | 0 golden, 2 AprilTag | 0.027 / 0.058 px | 0.042 / 0.115 px |
| Full resolution | 109 of 114 | 111 | 0, 0 | 0.037 / 0.077 px | 0.048 / 0.118 px |
| Small tags 20–40 px | 235 of 240 | 237 | 0, 1 | 0.158 / 0.456 px | 0.139 / 0.398 px |

- **Agreement:** golden and AprilTag corners agree to a median of 0.04–0.07 px. The PLAN target is about 0.1 px.
- **Quick decode:** it runs on the rough quad grown 1 px outward, because rough corners sit 1–3 px inside. This took small tags from 230 to 235 at no extra cost.
- **4A vs 4B, measured (synthetic scenes; `QuadOptions.Compare`).** Each quad finder feeds the same golden quick decode, refinement and decode:

  | Scenario | 4B | 4B + 4C | 4A (AprilTag's quads) | AprilTag |
  |---|---|---|---|---|
  | CPU config | 117 of 123 | 117 | 120 | 118 |
  | Full res | 82 of 86 | 82 | 84 | 84 |
  | Small tags | 176 of 180 | 176 | 177 | 178 |
  | Dark background | 100 of 105 | 100 | 102 | 100 |
  | Dark, small | 175 of 180 | 176 | 179 | 179 |

  - **Tags found:** 4A finds 1–4 more tags per scenario.
  - **Cost:** 4A sends about 3× the candidate quads into the quick decode, with the same number of refinements. 4A also needs per-component boundary storage and sorting, where 4B is fully streaming.
  - **4C** adds almost nothing.
  - **The trade-off for the Stage 4 decision** is 1–3% more tags against the 4A hardware. Recordings should settle it.
- **Tags cut by the frame edge:** detections with a corner outside the image are flagged (`cut_by_frame`), because the extrapolated corner can be 7–14 px off. The CAN-FD corner frame should carry this flag so the robot solver can down-weight such tags (open item for Stage 5).
- **4A vs 4C (synthetic evidence):** the remaining misses are tags merged with dark background through the blurred one-cell quiet zone. The simplified 4C contour tracer (`model/`) recovered none of them, because an outline can't separate a merge. AprilTag, whose boundary clustering is our 4A, finds 2 of them. So 4A, not 4C, looks like the fallback that matters for 4B's real failure mode. Recordings will decide.
- **4C prefilter:** about 140 blobs per noisy full-resolution frame fail 4B's shape check. With a cap of a few 4C candidates per frame, the 4C design needs a prefilter or a ranking.
- **Refinement fix for decimated rough quads:** they need a wider first-pass window (cap × decimation). Otherwise a large tag's corners can stay up to 16 px off after two passes.

**Edge refinement window (decided 2026-11-20):** a window that scales with the cell, W = clamp(0.5 × cell + 1, 2, 6) px, with the dark-to-bright direction check. Measured on 4B's correct rough quads:

| Policy | CPU config, decimated (154 tags) | Full res (160) | Small tags 20–40 px (286) | Heavy blur (157) |
|---|---|---|---|---|
| **Scaled + direction (chosen)** | 4 over 1 px; **0 with a second pass**, p95 0.056 px | 0 over 1 px, p95 0.075 px | 0, p95 0.43 px | 0, p95 0.33 px |
| Scaled, any direction | 4 over, 2 failed | 5 over | 194 over | 16 over |
| Fixed ±2 + direction | 152 over | 1 over | 0, p95 0.56 px | 149 over |
| Fixed ±5 + direction | 11 over | 0 over | 6 over or failed | 0 over |
| Fixed ±5, any direction | 11 over, 3 failed | 14 over or failed | 279 over or failed | 27 over |

One pass is enough at full resolution, which is how the FPGA runs 4B. Rough quads from a decimated image need a second pass, because their corners can be beyond the 6 px cap.

**ID lookup budget:** at most 11 candidates per 9-bit bucket (mean 4.6), which sizes the RTL's compare list.

**Findings:**
- **CPU pipeline single-tag poses were wrong (fixed 2026-11-20, in `src/`).** A truth test (`src/tests/PoseEstimatorTruthTest.cpp`) found three bugs:
  - **Single-tag camera pose:** with one tag in view, the field-to-camera pose was **1.2 m and 180° off**. The tag's own axes weren't converted to WPILib's convention, only the camera's.
  - **`single_tag_poses`:** `SingleTagPoseEstimator` also passed `(tvec, rvec)` swapped, which made these outputs, including `distance`, meaningless.
  - **Unknown first tag:** the single-tag branch looked up the first tag seen rather than the one in the layout, which is undefined behavior when the first tag isn't in the layout.

  Multi-tag (SQPnP) poses were correct. After the fix, every output matches the truth to 0.1 mm, and CI runs the test. **Robot code that used single-tag poses, or worked around them, needs checking.**
- **Edge quirks the RTL must reproduce:** the partial-tile threshold and the edge-column join rules (see the [model README](../../model/README.md#reference-quirks-the-rtl-must-reproduce)).
- **AprilTag threading bugs:** (1) its threaded connected components skip the last row's joins at some image heights (h = 400 with 2 threads; not the CPU pipeline's 320×200). This is still present in v3.4.5. (2) v3.3.0's thread pool had a use-after-free race on first use, fixed upstream in v3.4.3. The CPU pipeline and the models were bumped to v3.4.5 on 2026-11-20.

**Still to do:**
- The LED latency rig and the latency measurements.
- Recorded datasets: the recorder and replay tool are ready, and the recordings themselves still need to be made.
- A model for 4A (AprilTag-style boundary clustering).
- The detector comparison and the quad-finding decision.

## Risks

- The AprilTag library uses floating point and threads in places; the golden model may need a deterministic single-threaded configuration.
