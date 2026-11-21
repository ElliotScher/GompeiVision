# Stage 4: Streaming AprilTag detector

## Goal

An AprilTag 36h11 detector in FPGA logic that matches the golden model and finishes within ~2 ms of the last tag row being read out.

## Deliverables

Each step is a separate RTL block with its own cocotb testbench against the Stage 1 golden-model dumps:

- [PL] decimation and adaptive threshold
- [PL] streaming connected-components labeling with per-component feature accumulators (area, bounding box, K-direction extremes) and close detection
- Quad finding: 4B, the [PL] corner scorecard (decided provisionally 2026-11-21; see [PLAN.md](../PLAN.md#apriltag-detector)). Components it rejects are dropped and counted. A fallback (4A, or the 4C tracer) is added only if recordings show it's worth its LUTs, which stay reserved
- [PL] edge refinement and decode: quick decode before refinement, outlier-rejecting edge fits, closed-form homography and NAPA-style 9-bit block ID lookup, reading back from the on-chip strip buffer with DDR fallback
- [RPU] NAPA-style shape repairs for the few tags that need them
- Per-frame reset on the start-of-frame sideband, frame IDs on every result, and drop-and-count when a frame-store slot would be reused early
- Output of ID, decision margin, Hamming distance and ordered subpixel corners into a shared-memory ring, read by the RPU for CAN-FD corner output (Stage 5)
- Tag detection on the color sensor's luma stream, with corner accuracy compared against the mono sensor

## Exit criteria

- [ ] Every block matches its golden-model dump bit for bit on all Stage 1 datasets (or within a stated tolerance for refined corners).
- [ ] Full detector in simulation reproduces CPU detections on all datasets: same IDs, corners within 0.1 px.
- [ ] On hardware: detection tail measured and recorded (target ≤2 ms after the last tag row; stretch ≤0.2 ms with the corner scorecard and strip buffer) at the target FPS.
- [ ] Resource usage recorded and inside the Stage 0b budget.

## Risks

- With 4B, the first RTL avoids the riskiest piece in the project (boundary clustering and quad fitting: irregular memory access, variable work per frame). If recordings call for 4A, its fit runs on the [RPU] first, so the pipeline works end to end before any PL fit engines exist.
- Worst-case scenes (many tags, noisy backgrounds) can overflow cluster tables. Define and test overflow behavior: drop the smallest candidates, never stall.
