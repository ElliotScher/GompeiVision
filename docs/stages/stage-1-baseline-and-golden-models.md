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
- [ ] Quad-finding decision for Stage 4 (4A, or 4B with the 4C tracer) recorded, with the numbers behind it.

## Progress (2026-11-20)

The golden models that need no recordings have started in [`model/`](../../model/README.md). There are 35 GoogleTest cases, which pass under GCC 13 and 16 and are clean under ASan, UBSan and TSan.

- **Decimation and adaptive threshold:** bit-exact with AprilTag v3.4.5 (the CPU pipeline's version since 2026-11-20; v3.3.0 gave identical results), with the tile statistics as intermediates.
- **Streaming connected components:** the same partition as AprilTag's single-threaded labeling. Per-component accumulators (area, bounding box, extreme pixel in 16 directions) are emitted as each component closes.
- **Quad finding 4B (corner scorecard):** a rough quad from the 16 extremes, with sanity checks that send failures to 4C.
- **Edge refinement:** subpixel corners from the rough quad, using the chosen window rule below.
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
  - **Off-quad limit** (any extreme outside the quad by more than max(1.5 px, 5% of the mean side) is rejected). A fixed 1.0 px falsely rejected 13 of 162 decimated tags, and rounded corners pushed 50 of 168 full-resolution tags out.
  - **Fill ≤ 0.95** of the quad, since a tag's black region is at most 54/64.
  - **Minimum side 8 px:** a tag is 8 cells across, so nothing smaller decodes. This cut stray accepted quads from about 430 to about 15 per full-resolution frame, without losing a tag.
- **Sensor noise matters:** with ±6 noise, more than the threshold's `min_white_black_diff` of 5, flat background binarizes into speckle and doubles the strays.
- Recorded datasets are still needed before the 4A/4B decision.

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
- **Edge quirks the RTL must reproduce:** the partial-tile threshold and the edge-column join rules (see the [model README](../../model/README.md#reference-quirks-the-rtl-must-reproduce)).
- **AprilTag threading bugs:** (1) its threaded connected components skip the last row's joins at some image heights (h = 400 with 2 threads; not the CPU pipeline's 320×200). This is still present in v3.4.5. (2) v3.3.0's thread pool had a use-after-free race on first use, fixed upstream in v3.4.3. The CPU pipeline and the models were bumped to v3.4.5 on 2026-11-20.

**Still to do:**
- The LED latency rig and the latency measurements.
- Recorded datasets and the replay tool.
- Models for 4A and the 4C tracer, bit sampling and the brightness model, the homography, and PnP.
- The detector comparison and the quad-finding decision.

## Risks

- The AprilTag library uses floating point and threads in places; the golden model may need a deterministic single-threaded configuration.
