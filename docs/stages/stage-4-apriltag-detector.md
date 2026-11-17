# Stage 4: Streaming AprilTag detector

## Goal

An AprilTag 36h11 detector in FPGA logic that matches the golden model and finishes within ~2 ms of the last tag row being read out.

## Deliverables

Each step is a separate RTL block with its own cocotb testbench against the Stage 1 golden-model dumps:

- [PL] decimation and adaptive threshold
- [PL] streaming connected-components labeling
- [PL] boundary clustering by component pair
- Quad fitting: first on the [RPU] to get end-to-end results, then [PL] fit engines if the RPU can't keep up
- [PL] decode and edge refinement, reading back from the DDR frame store
- Output of ID, decision margin, Hamming distance and ordered subpixel corners into a shared-memory ring

## Exit criteria

- [ ] Every block matches its golden-model dump bit for bit on all Stage 1 datasets (or within a stated tolerance for refined corners).
- [ ] Full detector in simulation reproduces CPU detections on all datasets: same IDs, corners within 0.1 px.
- [ ] On hardware: detection tail measured and recorded (target ≤2 ms after the last tag row) at the target FPS.
- [ ] Resource usage recorded and inside the Stage 0b budget.

## Risks

- Boundary clustering and quad fitting are the riskiest pieces in the project (irregular memory access, variable work per frame). Keeping the RPU quad fit as a working fallback means the pipeline works end to end before the PL fit engines are done.
- Worst-case scenes (many tags, noisy backgrounds) can overflow cluster tables. Define and test overflow behavior: drop the smallest candidates, never stall.
