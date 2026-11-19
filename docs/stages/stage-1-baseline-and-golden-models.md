# Stage 1: Latency baseline and golden models

## Goal

Measure the CPU pipeline's real latency, and turn its code into golden models and datasets that the FPGA blocks are tested against.

## Deliverables

- **LED latency rig:** robot code toggles an LED in view of the camera and logs the time; the vision output timestamp is compared with the toggle time. Works for both CPU and FPGA systems.
- Measured exposure-to-robot latency and jitter (median, p95, p99) for the CPU pipeline at the competition resolution and FPS, recorded in this doc.
- Recorded datasets: raw frames plus CPU detections and poses, from practice fields and match-like motion (still, driving, fast turns), stored outside git with a manifest in `model/datasets/`.
- `model/`: C++ golden models for the threshold, connected components, quad fit, decode, refinement and PnP steps, built from [`src/`](../../src/) and the AprilTag library, with an option to dump intermediate results at each step.
- A replay tool: run any dataset through the models and diff the result against the CPU output.
- Golden models for **both** quad-finding candidates in [PLAN.md](../PLAN.md#apriltag-detector): 4A (boundary clustering + quad fit) and 4B (corner scorecard + edge refinement). A report on the recorded datasets compares detection rate, corner error and failure cases (tag merged with background, occlusion, steep angles) for each.

## Exit criteria

- [ ] Latency numbers for the CPU system measured and written down.
- [ ] Golden models reproduce the CPU pipeline's detections and poses on all datasets (identical IDs and corners; poses within a stated tolerance).
- [ ] Intermediate-step dumps exist for every block that Stage 4 will build.
- [ ] Quad-finding decision for Stage 4 (4A, 4B, or 4B with 4A fallback) recorded, with the numbers behind it.

## Risks

- The AprilTag library uses floating point and threads in places; the golden model may need a deterministic single-threaded configuration.
