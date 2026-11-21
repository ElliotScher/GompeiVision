# model/

C++ golden reference models for each PL and RPU block, plus a frame-replay harness. For integer blocks the model defines the exact expected output, and `sim/` compares RTL against it bit for bit. The robot library's solver is tested against the same models.

- **Built from:** the CPU pipeline in [`src/`](../src/), the AprilTag library, NAPA and ArUco Nano. See [Detectors and their roles](../docs/PLAN.md#detectors-and-their-roles).
- **Datasets:** recorded frames live outside git. `model/datasets/` holds only the manifest.
- **Filled in by:** [Stage 1](../docs/stages/stage-1-baseline-and-golden-models.md).

## Build and test

A standalone host build. CMake fetches AprilTag at the CPU pipeline's tag (v3.4.5) and GoogleTest:

```bash
cmake -S model -B build/model
cmake --build build/model -j
ctest --test-dir build/model --output-on-failure
```

CI runs the same commands on every push (`.github/workflows/ci.yml`).

## Models so far

| Block | Header | Matches |
|---|---|---|
| Decimation | [`decimate.h`](include/gv/model/decimate.h) | `image_u8_decimate()`, integer factors (the CPU pipeline uses 4) |
| Adaptive threshold | [`threshold.h`](include/gv/model/threshold.h) | `threshold()`, bit for bit, with the tile min/max and dilated stats exposed as intermediates |
| Connected components | [`components.h`](include/gv/model/components.h) | `connected_components()` with `nthreads = 1`: same partition. Streamed one row at a time, with per-component accumulators (area, bounding box, extreme pixel in 16 directions) emitted when each component closes |
| Quad finding 4B (corner scorecard) | [`quad.h`](include/gv/model/quad.h) | No reference (new design). Checked against ground-truth corners on synthetic scenes; see below |
| Edge refinement | [`refine.h`](include/gv/model/refine.h) | No reference (new design). Checked against ground-truth corners; see below |
| ID lookup and error correction | [`tag_lookup.h`](include/gv/model/tag_lookup.h) | NAPA's `nearest_code()` and `errors_connected()`: four 9-bit block tables, up to 3 corrected bits, and a third bit only when the wrong cells touch. Checked against brute force over all 2,348 rotated codes |

The tests compare each model with the library's own internal functions, or with brute force, on synthetic tag scenes ([`tests/synth.h`](tests/synth.h)), noise and blocky images, at many sizes, including the CPU pipeline's 1280×800 with decimation 4.

### Reference quirks the RTL must reproduce

- **Partial edge tiles are never "skipped".** When the width or height isn't a multiple of 4, the right and bottom partial tiles always binarize to 0/255, never 127, using the nearest full tile's statistics.
- **Columns 0 and w−1 are never reference pixels,** and AprilTag skips some joins it considers redundant. At the edges those skips change the partition: for example, a white pixel at x = w−1 joins only diagonally from the row below. The model applies AprilTag's exact join tests ([`reference_joins()`](include/gv/model/components.h)).
- **AprilTag's threaded labeling drops the last row's joins** when `(h - 1) % (1 + h / (10 * nthreads)) == 0`. Examples: h = 400 with 2 threads, and any h < 20. The model follows the correct single-threaded result. The CPU pipeline's 320×200 decimated image with 2 threads isn't affected, and a test checks that.

### Numbers so far

- **ID lookup:** each 9-bit bucket lists at most **11** candidate codes (mean 4.6). That's the worst-case compare list per block-RAM lookup in the RTL.
- **4B on synthetic scenes** (`Quad.ScorecardOnSyntheticScenes*`). "Refinable" means every rough corner is within max(2 px, 5% of the side), which assumes step 5 searches about ±3 px along each edge normal.

  | Setup | Refinable quad | Within 1.5 px | Sent to 4C | Silent misses (wrong quad accepted) | Accepted quads matching no tag |
  |---|---|---|---|---|---|
  | 1280×800, decimate 4 (CPU config), 162 tags | 95.1% | 85.8% | 7 | 1 | 64 in 40 frames |
  | 640×400 full resolution, 168 tags | 95.2% | 24.4% | 7 | 1 | 874 in 60 frames |

  At full resolution, corners come out 1.5–3 px inside the true corners: blur and the threshold round them, which is expected for a rough quad. The parameter sweeps that set the defaults are in the [Stage 1 doc](../docs/stages/stage-1-baseline-and-golden-models.md#progress-2026-11-20). These are synthetic scenes; recorded datasets will replace them.

- **Edge refinement** (`Refine.*`) on 4B's correct rough quads, final worst-corner error per tag. The default is a window scaled to the cell, capped at ±6 px, with the direction check:

  | Scenario | Median | p95 | Over 1 px |
  |---|---|---|---|
  | 1280×800, 4B on decimate 4 (two passes) | 0.027 px | 0.056 px | 0 of 154 |
  | 640×400, full resolution (one pass) | 0.036 px | 0.075 px | 0 of 160 |
  | Small tags, 20–40 px | 0.151 px | 0.430 px | 0 of 286 |
  | Heavy blur (edges spread over ~8 px) | 0.081 px | 0.330 px | 0 of 157 |

  The alternatives are worse somewhere:
  - **Fixed ±2:** 152 of 154 CPU-config tags over 1 px, and 149 of 157 blurred ones.
  - **Fixed ±5:** wrong edges on small tags (worst 6.7 px).
  - **Without the direction check:** 194 of 286 small tags over 1 px.

### AprilTag version

The models track the CPU pipeline's AprilTag, v3.4.5 since 2026-11-20. Moving from v3.3.0 changed no threshold, labeling or decimation result: every reference test passed unchanged. The bump was made for a thread-pool bug in v3.3.0. `workerpool_add_task()` appended to the task array without the lock, so a worker woken early could hold a pointer into the array across a reallocation. That use-after-free crashed about 1 in 20 runs of the threaded tests, and could crash the CPU pipeline's first detection with `nthreads = 2`. It was fixed upstream in v3.4.3 (aa5951aa, c16bba61). The threaded last-row quirk above is still present in v3.4.5.

## Dump tool

```bash
build/model/gv-model dump frame.pgm out/ --decimate 4
```

This writes the decimated image, the raw and dilated tile statistics, the thresholded image, a component color map, and `components.csv` (closed components in closure order, with each black component's 4B verdict and rough corners). RTL testbenches dump the same intermediates for diffing.
