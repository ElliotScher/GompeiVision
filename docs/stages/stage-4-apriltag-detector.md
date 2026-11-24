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

## Progress (2026-11-21)

- **[PL] adaptive threshold:** [`rtl/threshold/threshold.sv`](../../rtl/threshold/threshold.sv). In Verilator it is bit-exact with the golden model on 6 cocotb tests (`sim/threshold`), including AprilTag's partial-tile quirk and a full 1280×800 frame.
  - **Design:** one pixel per clock. A 16-row pixel ring (20 KB at 1280 wide) holds rows until the tile row below them is complete. Tile statistics go in 4 single-port memories, and the 3×3 dilation runs on a sliding window with one read per tile column per row.
  - **Requirements:** at least 3 idle cycles per input row on average, and about 12 rows of vertical blanking; `o_overrun` flags either being broken.
  - **Checked:** two deliberate bugs, partial tiles treated as full and the missing-column flag ignored, are both caught.
  - **Not yet:** synthesized, timed or resource-counted.

- **[PL] connected components, first version:** [`rtl/components/components.sv`](../../rtl/components/components.sv). In simulation it emits exactly the golden model's components, in the same order: id, color, area, bounding box, closing row and all 16 extreme pixels. That's 3 cocotb tests covering odd sizes, tag-like scenes and noise.
  - **Design:** pixels form runs as they arrive; a run's 16 extremes follow from its two end pixels. At each row end, a sequential processor allocates labels, applies the queued up-joins with union-find, and emits closed components.
  - **Checked:** three deliberate bugs are all caught: the up-right join shortcut, the last-column rule and the extremes tie-break.
  - **Measured:** the longest row took about 1,000 processing cycles on 160-px-wide noise and tag-like scenes, roughly 6 cycles per pixel of width. Real frames at 1280 wide get about 1,300 cycles per row, so the next version needs overlapped merging and/or a faster fabric clock, plus label recycling. The second version below does both.

- **Fixed-point widths for refinement and decode (decided 2026-11-22).** The golden model can now emulate fixed point (`model/include/gv/model/fixed_point.h`). A sweep over full-resolution, small-tag, heavy-blur and large-tag scenes (243 tags) picked:

  | Quantity | Width | Register (with integer part) |
  |---|---|---|
  | Pixel coordinates (rough corners, sample points, edge points, corners) | 8 fractional bits | 20 bits unsigned (12.8) |
  | Bilinear weights | 8 fractional bits | 8 bits |
  | Edge direction and normal | 12 fractional bits | 13 bits signed |
  | Interpolated pixel values | 4 fractional bits | 12 bits (8.4) |
  | Homography coefficients (decode's projector) | h0..h5: 14 fractional bits; h6, h7: 24 | 32 bits signed each (19.14 and 7.24): 8–16× headroom over the largest seen |
  | Brightness-plane coefficients (decode) | 8 fractional bits, in tag coordinates × 8 | the fit's sums and determinants are exact integers (sample positions are −9..9), so only the three coefficients round. Even 2 bits changed no decode on synthetic tags; 8 leaves room for low-contrast tags (re-check on recordings) |

  - **Result:** together these add a median 0.004 px and at most 0.04 px of corner error to double precision, with no tag gained or lost. That's well under the detector's own 0.04–0.08 px. `FixedPoint.ChosenWidthsMeetBudget` guards it.
  - **The direction is the sensitive one.** Sample points are placed along the edge from its corner, so a direction error grows with edge length: 8 bits gave a 1.8 px error on a large tag.
  - **Every product fits one DSP slice** (27×18).
  - **How the RTL is verified:** discrete decisions (the outlier cut, the gradient lobe's extent) can flip on tiny differences at any precision. So the RTL is checked bit for bit against the golden model run with these widths, not against double precision with a tolerance.
  - **Not quantized yet:** the line fit and intersection, which are small enough for the RPU.

- **[PL] edge sampler (refinement):** [`rtl/refine/edge_sampler.sv`](../../rtl/refine/edge_sampler.sv). In simulation it returns exactly the golden model's edge points at the Stage 4 widths: 969 points from 32 edge jobs (8 tags: small, normal and large).
  - **The split:** the golden refinement is now explicitly three stages. RPU setup computes each edge's direction, normal, window and sample count (a square root and a division per edge). The PL sampler does the per-sample profile, gradient, lobe and centroid work. The RPU line fit and intersection follow.
  - **Arithmetic:** every rounding is one wide sum rounded once, half away from zero, so integer hardware reproduces the model exactly.
  - **Checked:** three deliberate bugs (the lobe cut, position rounding, bilinear rounding) are all caught, down to 1/256 px.
  - **Throughput:** this first version took about 64 cycles per point. The pipelined version is below.

- **[PL] ID lookup:** [`rtl/decode/id_lookup.sv`](../../rtl/decode/id_lookup.sv). It is bit-exact with the golden model's `decode()` on 1,350 reads at `max_bits` 0–3, plus every rotated code. Its worst case is 43 cycles per lookup.
  - **Checked:** three deliberate bugs are all caught: the touching rule dropped, a bucket cut one short, and rotations swapped.
- **Decode sampling stays in the PL (analysis, 2026-11-22).** Reading a tag's ~200 pixels from the strip buffer would cost an R5F on the order of 40 µs per tag, at roughly 100 cycles per AXI read. The quick decode also runs on every 4B candidate, 15–40 per synthetic frame, so that becomes a millisecond or more per frame on the RPU, against the PLAN's ~0.1 ms tag-finish target. In the PL it's under a microsecond per tag.
  - **Done:** the brightness-plane fit and the sharpening are now in the fixed-point model (planes in the table above; sharpening by 0.25 is exact).

- **[PL] tag sampler (decode):** [`rtl/decode/tag_sampler.sv`](../../rtl/decode/tag_sampler.sv). Given a tag's 100 projected sample points, it returns exactly the golden model's ok, code and decision margin: 58 code reads (54 ok, 4 failed) on synthetic scenes, covering tags and non-tag candidates. At most 139 cycles per read.
  - **Checked:** four deliberate bugs are all caught: no sharpening, truncated planes, a wrong border-pattern offset, and a reversed margin compare.
  - The read counts above are with the PL projector's points (58 reads, 53 ok).

- **Projection runs in the PL (measured 2026-11-22).** The RPU fits each quad's homography (it has the refined corners, and it's a handful of divisions) and sends 8 fixed-point coefficients. A PL projector turns them into the 100 sample points.
  - **Why not the RPU:** synthetic frames make up to 200 code reads (62 on average), because the quick decode runs on every candidate. At about 100 points × 2 double-precision divisions each, plus writing 200 coordinates to the PL, that's milliseconds per frame on an R5F. In the PL it's 8 register writes per read.
  - **Exact by construction:** sample points sit on a 1/16 grid of the unit square, so with fixed-point coefficients every coordinate is one exact integer division, rounded once. The model's `FixedProjector` is the reference.
  - **The sweep** (`FixedPoint.DISABLED_ProjectionSweep`, 4,223 reads): at 14/24 fractional bits every point is within 1/256 px of a double-precision projection and 99% are identical, with no tag gained or lost. The old 14-significant-bit budget moved points up to 0.4 px (still no tag lost, since the earlier sweep measured corners, which projection doesn't touch); it's retired.
  - **Validity:** the RPU sends a projector only if its coefficients fit, the denominator is positive at the four extreme sample positions (so everywhere, as it's linear) and those four points land within ±32,768 px (so every point does). Otherwise the read fails. Synthetic scenes never hit this.
- **[PL] projector (decode):** [`rtl/decode/projector.sv`](../../rtl/decode/projector.sv). Bit-exact with `FixedProjector` on all 100 points of 85 code reads from synthetic scenes, and on 60 extreme projectors at the format's limits (quads out to ±32,000 px, tags tens of thousands of pixels wide, strong perspective; `gv-model project`).
  - **Checked:** five deliberate bugs, and four are caught: truncating instead of rounding, a cell off by half, a wrong border-pattern offset, and a dropped sign. A quotient two bits too narrow is caught only by the extreme projectors (synthetic scenes stay under 4,096 px).
  - **Not yet:** one restoring divider per coordinate, a bit per cycle, so 2,601 cycles per read. (Pipelined since; see the decode entry below.)

- **[PL] corner scorecard (quad finding 4B, 2026-11-23):** [`rtl/quad/scorecard.sv`](../../rtl/quad/scorecard.sv). In simulation it returns exactly the golden model's verdict, rough corners and area. That's 26,397 components from synthetic scenes and noise (96 accepted), plus 1,990 made-up records that sit on the decision limits or at the 12-bit coordinate limits.
  - **Integer golden model:** `scorecard_quad()` is now integer arithmetic. Only the four side lengths round, to floor(256 × length). All enabled golden-model tests still pass.
  - **Cycles:** at most 832 per component on the scenes and noise. A component with 16 distinct extremes is the worst case, about 1,940.
  - **Checked:** 18 deliberate bugs are all caught, including the last bit of a side length (1/256 px) and exact equality at both fill bounds.
  - **Not yet:** synthesized or timed. The exhaustive search uses one candidate per cycle. It scores any component it's given; the quad finder below selects the black ones.

- **[PL] quad finder, the detector's front end (2026-11-24):** [`rtl/quad_finder/quad_finder.sv`](../../rtl/quad_finder/quad_finder.sv) joins threshold → pixel FIFO → components → candidate filter → record FIFO → scorecard. In simulation, whole frames in give exactly the golden model's quads, in order, with frame IDs and per-frame counts: four tag scenes up to 640×400, and seven frames of mixed sizes with up to four inside at once.
  - **Candidate filter:** black, area ≥ 16 px, bounding-box diagonal ≥ 8 px. None of these can change which quads are accepted. PLAN step 4's aspect and too-dark filters aren't in the golden model, so they aren't here either.
  - **Overload drops and counts.** A full pixel FIFO cuts the frame and the block resumes at the next start of frame; a full record FIFO drops the newest candidate (PLAN's "drop the smallest" isn't built). All three overloads are tested, including exact results on the frame after a cut.
  - **Bug found and fixed:** `components.sv` kept the joins of a frame cut mid-row if the next frame's first pixel was 127, which merged wrong components in that frame.
  - **Checked:** 12 deliberate bugs. 11 are caught, and the last (diagonal `<=`) can't change any output.
  - **Measured with the first components version:** it was the bottleneck (a 640-wide row took up to 3,225 cycles, and the pixel FIFO held about 80% of a frame), and 1280×800 would have needed 454k labels. Hence the second version.

- **[PL] connected components, second version (2026-11-24):** three overlapping stages (input, label resolver, record engine) with recycled labels, still exact against the golden model, including order. No recordings were available, so it's sized on synthetic scenes and noise.
  - **Measured:** the 1280×800 synthetic scene goes through the quad finder at one pixel per clock with 40 cycles of blanking per row: exact quads (178 of 5,278 candidates), the pixel FIFO never above 1, the longest row 1,195 cycles and at most 721 labels. Noise at 1280 wide: 1,203 cycles and 909 labels. The record FIFO reached 43 of 60 usable entries, so its default is now 128.
  - **Union by size matters:** always keeping the left root costs about 400 extra find cycles per row at 1280 wide, and a full frame then falls behind. Union by size costs about 35.
  - **Worst case:** a 1-pixel checkerboard takes 2.5 cycles per pixel. It overflows at one pixel per clock and is cut and counted.
  - **Tests:** 6 cocotb tests, adding 13 adversarial shapes, 1280-wide noise, and 40 cases of frames cut at any pixel.
  - **Checked:** 28 deliberate bugs. 24 are caught. Three can't change any output (redundant resets and a redundant marker check), and one (not adding sizes on a merge) only costs speed, within the test's bound.
  - **Not yet:** synthesized or timed; the record memory is about 4 Mbit at the default size.

- **[PL] decode joined and sped up (2026-11-24):** [`rtl/decode/decode.sv`](../../rtl/decode/decode.sv) takes homographies and returns reads (code, margin, ID, rotation, Hamming distance), in order. In simulation it's bit-exact with the golden model on every code read of the synthetic scenes and on about 300 random quads (reversed, tiny, huge, partly or wholly off the image).
  - **Speed:** about 214 cycles per read back to back, against about 2,780 for the old blocks in series. A 200-read frame is about 43,000 cycles, or 0.29 ms at an assumed 150 MHz, spread over the frame as tags close.
  - **How:** the projector's divider is pipelined (127 cycles per read, was 2,601). The tag sampler takes points as a stream, so it needs no point memory. Its six plane divisions and its margin division are bit-serial (they were single-cycle 64-bit divisions), and the next read's projection starts during the last one's tail.
  - **Pixels:** reads go through [`pixel_window.sv`](../../rtl/pixels/pixel_window.sv), a four-bank store with a registered 2×2 read that fits block RAM. Coping with rows that have left the ring (the DDR fallback) isn't built.
  - **Golden model:** `read_code()` can also start from a homography (`read_code_homography()`, `gv-model read`); its results are unchanged.
  - **Checked:** 31 deliberate bugs. 29 are caught, and two can't change any output (a 3-sample minimum that a zero determinant already covers, and a guard that only matters if the projector's latency grows).
  - **Not yet:** synthesized or timed. The sharpening step reads 5 cells from registers in one cycle, the lookup's ROMs are still read asynchronously, and image size must hold still while reads are in flight.

- **[PL] edge sampler pipelined, and sharing pixels with decode (2026-11-24):** [`rtl/refine/edge_sampler.sv`](../../rtl/refine/edge_sampler.sv) now takes one profile sample per clock, and [`rtl/tag_finish/tag_finish.sv`](../../rtl/tag_finish/tag_finish.sv) puts it and decode on one pixel window. Both are bit-exact with the golden model, separately and running at once.
  - **Speed:** 22.2 cycles per point, against about 64 (21,319 profile samples in 21,466 cycles). By calculation, a 40 px tag needs about 1,600 cycles (96 points of 17 samples) and a 90 px tag about 5,700 (211 points of 27), or 11–38 µs per tag at an assumed 150 MHz. Refinement is the largest part of a tag's PL time, and the profile reads are what limit it: a second pixel window would halve it.
  - **How:** points are placed, read, searched and divided in overlapping stages; the lobe grows both ways at once while its sums accumulate; the centroid's divider is pipelined.
  - **Golden model:** `gv-model sample` runs the edge sampler on arbitrary jobs.
  - **Checked:** 29 deliberate bugs, all caught.
  - **Not yet:** synthesized or timed, and the image size must hold still while jobs are in flight.

- **[RPU] finisher, and the whole detector in simulation (2026-11-24):** [`rpu/`](../../rpu/) now has the RPU's half as portable C++ (quick-read homography, edge jobs, line fits, corners, final read, duplicates), and [`rtl/detector/detector.sv`](../../rtl/detector/detector.sv) joins the PL half. With that C++ driving the simulated PL, 15 frames from 256×160 to 1280×800 give exactly the golden detector's 39 tags: IDs, Hamming distances, margins, and corners to 1/256 px.
  - **On the host:** the finisher with the golden model playing the PL gives 442 tags identical to the golden detector, over 118 scenes (normal, small, blurred and large tags) with events in random order.
  - **Jobs wait for their rows.** A read samples half a cell below the tag (25 px for a 400 px tag), and those rows may not be in when the quad arrives. The golden model sees whole frames, so it hid this. The PL now reports rows in, and the finisher holds each job until its rows are there. Large tags therefore finish up to half a cell later than their last row. Once the next frame overwrites a candidate's rows, it's dropped and counted.
  - **Tail after the last pixel (cycles, with an RPU that answers at once):** 3,200–4,400 at 256×160, 7,400–11,800 at 640×400 and 15,700 at 1280×800 (0.1 ms at an assumed 150 MHz). Most of that is the threshold's 12-row lag. The real RPU's time comes on top and isn't measured.
  - **The line fit needs no trigonometry:** its sums are exact integers and its direction comes from square roots, with the same corners as the golden model's `atan2`.
  - **Checked:** 32 deliberate bugs in the finisher. 29 are caught; two can't change any output, and one (the edge jobs' row margin) is covered by the quick read's longer wait. Advertising rows too early in the RTL is caught too.
  - **Against the CPU detector (not new; the golden model's own comparison, in double precision with 4C on):** median corner difference 0.04–0.07 px and 95th percentile 0.08–0.09 px, but 0.24 px on 20–40 px tags. It finds 1–2% fewer tags than AprilTag, with fewer false ones. So the 0.1 px exit criterion isn't met for every tag, small ones especially, where AprilTag and ours are about equally far from the truth (median 0.14 and 0.16 px). The criterion needs a decision: compare with the truth, or allow for tag size.
  - **Not yet:** recordings, 4C and shape repairs, decimation, the DDR fallback, changing frame sizes mid-stream, and anything on hardware.

- **First synthesis estimate, without Vivado (2026-11-24):** `scripts/synth-estimate.sh` runs the RTL through sv2v and Yosys for UltraScale+. The whole PL detector synthesizes. This is area only: Yosys gives no usable timing here, Vivado's counts will differ, and the DPU, camera and CAN-FD blocks aren't included.

  | Block | LUTs (logic + memory) | DSP | BRAM36 | URAM |
  |---|---|---|---|---|
  | Threshold | 0.8k | 0 | 4 | 1 |
  | Connected components | 5.9k + 4.6k | 42 | 5.5 | 14 |
  | Corner scorecard | 2.9k + 0.2k | 32 | 0 | 0 |
  | Quad finder (those three and their FIFOs) | 10.5k + 4.8k | 103 | 19 | 15 |
  | Projector | 6.0k | 12 | 0 | 0 |
  | Tag sampler | 12.8k + 0.2k | 43 | 0 | 0 |
  | ID lookup | 2.8k | 0 | 4 | 0 |
  | Edge sampler | 2.0k + 0.3k | 25 | 0 | 0 |
  | Pixel window (1,024 rows) | 1.6k | 3 | 0 | 36 |
  | **Whole detector** | **34.2k + 5.4k** | **161** | **23** | **51** |
  | K26 | 117k | 1,248 | 144 | 64 |

  - **Against PLAN's estimate for tag detection** (~25–40k LUTs including 8–15k held for a fallback quad finder, ~20–50 DSPs, ~2–3 Mbit): about 40k LUTs with no fallback, 161 DSPs, and about 15 Mbit of UltraRAM. So it's over on all three, though still about a third of the device's LUTs. PLAN.md isn't changed until Vivado's numbers are in.
  - **Found and fixed:** the pixel window needed 320 block RAMs and the component records 112, more than the whole device together. Both are in UltraRAM now, with 8 pixels packed per word. The lookup ROMs were loaded with code synthesis tools don't accept. The tag sampler was 24k LUTs (a 100-register cell grid and needlessly wide multipliers) and the projector 7.6k.
  - **UltraRAM needs a decision.** The 1,024-row pixel window and the records take 51 of 64 blocks (the window is 40 by hand count, so up to 55), and the DPU would share what's left. A 512-row window halves its share and a 256-row one, PLAN's 320 KB strip buffer, quarters it. The finisher now handles a ring shorter than the frame: it drops candidates whose rows are gone. But without the DDR fallback, that loses tags nearly as tall as the ring.
  - **Biggest remaining blocks:** the tag sampler (12.8k) and components. The scorecard's 32 DSPs and the quad finder's 103 are more than the work needs.

## Risks

- With 4B, the first RTL avoids the riskiest piece in the project (boundary clustering and quad fitting: irregular memory access, variable work per frame). If recordings call for 4A, its fit runs on the [RPU] first, so the pipeline works end to end before any PL fit engines exist.
- Worst-case scenes (many tags, noisy backgrounds) can overflow cluster tables. Define and test overflow behavior: drop the smallest candidates, never stall.
