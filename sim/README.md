# sim/

Verilator + cocotb testbenches. Each one compares an `rtl/` block with its `model/` golden model on synthetic inputs and recorded frames.

- **Layout:** `sim/<block>/test_<block>.py` is a pytest file that builds the block with cocotb's Python runner and runs the cocotb tests next to it. [`tools/selftest/`](../tools/selftest/) shows the pattern.
- **Run:** `scripts/run-sims.sh` runs every testbench plus the toolchain self-test, the same way CI does. It first builds the golden model's `gv-model` tool (into `build/model-tool/`), because testbenches compare against it, and the RPU's code for the host (into `build/rpu-host/`).
- **Testbenches:**
  - [`threshold/`](threshold/): 6 cocotb tests covering odd sizes, low contrast and flat images, `min_white_black_diff` values, tag-like scenes, a full 1280×800 frame, and overrun detection.
  - [`components/`](components/): 6 cocotb tests (odd sizes, tag-like scenes, noise, adversarial shapes, 1280-wide noise within a cycle bound, frames cut at any pixel). The RTL must emit exactly the golden `components.csv`: every record, in order, including all 16 extreme pixels.
  - [`quad/`](quad/): 5 cocotb tests of the corner scorecard against `gv-model quads`: every component of synthetic tag scenes and noise, and records on the decision limits, to the last bit of a side length, or at the coordinate format's extremes.
  - [`quad_finder/`](quad_finder/): whole frames through threshold, components and scorecard, on four builds: one with room for everything, which must match the golden model exactly, and three short of pixels, records or labels, which must drop, count and recover (labels: flag it).
  - [`refine/`](refine/): the edge sampler, with its pixel port free and partly busy, on every edge job the detector makes on synthetic tag scenes (`gv-model edges`: 969 points from 32 jobs), and on 744 made-up jobs (`gv-model sample`: every half width, both direction rules, search lines that end exactly on the image's borders).
  - [`tag_finish/`](tag_finish/): code reads and edge jobs running at once on the shared pixel window.
  - [`decode/`](decode/): the whole decode block with its pixel memory, on every code read of synthetic scenes (back to back and with gaps) and on 300 random quads over noise and a scene, including reversed and off-image ones (`gv-model read`); the projector on every read's 100 sample points from the same scenes, plus 60 extreme projectors at the format's limits (`gv-model project`); the tag sampler on every code read the detector makes on synthetic tag scenes (`gv-model decodes`: tags, non-tag candidates and failed reads), and the ID lookup on 1,350 reads at every `max_bits` setting (exact codes; 1, 2 and 3 random flips; 3 touching flips; 4 or more flips; random patterns), plus all 2,348 rotated codes, against `gv-model lookup`.
  - [`detector/`](detector/): whole frames through the whole PL detector, with the RPU's C++ (`rpu/`, built for the host) driving its job ports. The finished tags must be the golden detector's to the last bit, and no read may be sent before its rows are in.
- **Shared helpers:** [`gvsim.py`](gvsim.py) has PGM I/O, `gv-model dump` and the test images. Test modules add `sim/` to `sys.path`, because cocotb's runner sets its own `PYTHONPATH`.
- **Filled in by:** Stage [3](../docs/stages/stage-3-camera-capture.md) onward.
