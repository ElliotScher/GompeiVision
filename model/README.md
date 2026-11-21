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
| Decode (homography, brightness model, bit sampling) | [`decode.h`](include/gv/model/decode.h) | AprilTag's `quad_decode()` steps, with Heckbert's closed-form homography; checked by decoding every drawn tag from its true corners |
| Quad finding 4C (contour tracer, simplified) | [`tracer.h`](include/gv/model/tracer.h) | Outer-boundary trace, hull, largest-area quad, edge-fit check. Unit-tested; see the 4C finding below |
| Datasets and replay | [`dataset.h`](include/gv/model/dataset.h) | Loads sessions recorded by the CPU pipeline ([format](datasets/README.md)) and diffs the golden detector against them (`gv-model replay`) |
| Whole detector | [`detector.h`](include/gv/model/detector.h) | Decimate → threshold → components → 4B (or 4C for its shape rejects) → quick decode → refine → decode → lookup. Compared with AprilTag 3, as the CPU pipeline runs it, and with ground truth; see below |
| PnP | [`pnp.h`](include/gv/model/pnp.h) | IPPE-square matches OpenCV's `SOLVEPNP_IPPE_SQUARE` (both solutions, order and errors) to 1e-6 on 2,000 noisy, distorted cases (`src/tests/GoldenPnpCrossCheck.cpp`). Multi-tag Levenberg–Marquardt seeded from both IPPE solutions. WPILib conversion matches the CPU pipeline's fixed `PoseUtils` |
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
  | 1280×800, decimate 4 (CPU config), 162 tags | 95.1% | 85.8% | 7 | 1 | 201 in 40 frames |
  | 640×400 full resolution, 168 tags | 95.2% | 24.4% | 7 | 1 | 2,161 in 60 frames |

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

- **Whole detector vs AprilTag 3** (`Detector.*`). AprilTag runs as the CPU pipeline does (refine_edges, sharpening 0.25), with the same decimation. A detection counts if its ID is right and every corner, in order, is within 2 px of the truth:

  | Scenario | Golden found | AprilTag found | False (golden / AprilTag) | Corner error vs truth, golden | Corner error vs truth, AprilTag | Golden vs AprilTag |
  |---|---|---|---|---|---|---|
  | 1280×800, decimate 4 | 117 of 123 | 118 | 0 / 2 | median 0.027, p95 0.058 px | 0.042, 0.115 px | 0.044, 0.082 px |
  | 640×400, full res | 109 of 114 | 111 | 0 / 0 | 0.037, 0.077 px | 0.048, 0.118 px | 0.054, 0.090 px |
  | Small tags (20–40 px) | 235 of 240 | 237 | 0 / 1 | 0.158, 0.456 px | 0.139, 0.398 px | 0.074, 0.242 px |

  - **Agreement:** corners match AprilTag within the PLAN's ~0.1 px (median), and are closer to the truth at normal sizes.
  - **Quick-decode grow:** the quick decode runs on the rough quad grown outward by 1 px. Rough corners sit 1–3 px inside, which misreads the 2.5–5 px cells of small tags. Growing took small tags from 230 to 235 found at no extra cost. Skipping the quick decode reaches 237, but needs about 10× as many refinements.
  - **4C finding:** the simplified tracer recovered none of the remaining misses. They are tags merged with dark background through a blurred one-cell quiet zone, so no outline-based method can separate them. AprilTag still finds 2 of them, plausibly through its boundary clustering (4A), which works from black/white edge pairs. This points to 4A, not 4C, as the fallback that matters. Confirm on recordings.
  - **4C traffic needs a prefilter.** About 140 blobs per full-resolution synthetic frame fail 4B's shape check, mostly noise speckle. The PLAN caps 4C at a few candidates per frame, so it needs a prefilter or a ranking, or noise would crowd out the real tags.

- **PnP** (`Pnp.*`):
  - **Exact without noise:** 500 random poses each with and without distortion, error under 1e-6° and 1e-9 m.
  - **With 0.1 px corner noise:** the lower-error IPPE solution was the wrong one in 2 of 500 cases.
  - **Three tags, 0.3 px noise:** LM brings the mean position error to 1.2 cm, against 4.0 cm for the best single tag. That requires seeding from *both* IPPE solutions; seeding from the "best" one alone often lands in a local minimum at range.

- **Quad finders compared** (`QuadOptions.Compare`). Each feeds the same golden quick decode, refinement and decode; tags found, of those drawn:

  | Scenario | 4B | 4B + 4C | 4A (AprilTag's `apriltag_quad_thresh` quads) | AprilTag's whole detector |
  |---|---|---|---|---|
  | 1280×800, decimate 4 | 117 of 123 | 117 | **120** | 118 |
  | 640×400 full res | 82 of 86 | 82 | **84** | 84 |
  | Small tags 20–40 px | 176 of 180 | 176 | **177** | 178 |
  | Dark background (20–90) | 100 of 105 | 100 | **102** | 100 |
  | Dark background, small tags | 175 of 180 | 176 | **179** | 179 |

  - **Tags found:** 4A finds 1–4 more tags per scenario. With golden refinement it matches or beats AprilTag's whole detector.
  - **Cost:** 4A sends about 3× as many candidate quads to the quick decode (for example 129 vs 38 per full-resolution frame). Refinements per frame stay the same, because the quick decode filters them.
  - **Simplicity:** 4B is fully streaming and stores no edge points. 4A needs boundary points stored and sorted per component, the "irregular part" in the PLAN.
  - **4C** adds almost nothing over 4B.
  - **False detections:** the golden detector has none in any scenario. AprilTag's whole detector has up to 2 per scenario, all the right ID with corners 3–5 px off.
- **Tags cut by the frame edge** are flagged (`Detection::cut_by_frame`, any corner outside the image). Their outside corner is extrapolated and can be 7–14 px off. Refinement also no longer samples past the image border: clamped pixels used to fake an edge there.

### AprilTag version

The models track the CPU pipeline's AprilTag, v3.4.5 since 2026-11-20. Moving from v3.3.0 changed no threshold, labeling or decimation result: every reference test passed unchanged. The bump was made for a thread-pool bug in v3.3.0. `workerpool_add_task()` appended to the task array without the lock, so a worker woken early could hold a pointer into the array across a reallocation. That use-after-free crashed about 1 in 20 runs of the threaded tests, and could crash the CPU pipeline's first detection with `nthreads = 2`. It was fixed upstream in v3.4.3 (aa5951aa, c16bba61). The threaded last-row quirk above is still present in v3.4.5.

## Dump tool

```bash
build/model/gv-model dump frame.pgm out/ --decimate 4
build/model/gv-model replay /path/to/session --csv per_frame.csv   # exit 1 if any frame disagrees
```

This writes the decimated image, the raw and dilated tile statistics, the thresholded image, a component color map, and `components.csv` (closed components in closure order, with each black component's 4B verdict and rough corners). RTL testbenches dump the same intermediates for diffing.
