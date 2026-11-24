# rtl/

SystemVerilog for the FPGA logic: MIPI capture and the timestamp counter, FrameBus, the streaming AprilTag pipeline, the downscaler for the DPU, and the CAN-FD glue. Vendor IP (MIPI CSI-2 RX, DPU, VCU) is instantiated from the block design in `platform/`, not copied here.

- **Layout:** one folder per block (`rtl/<block>/`), each with its own top module.
- **Style:** `logic`, `always_ff`/`always_comb`, synchronous active-high reset. It must lint clean with Verible and with Verilator `-Wall`.
- **Filled in by:** Stages [3](../docs/stages/stage-3-camera-capture.md) and [4](../docs/stages/stage-4-apriltag-detector.md) onward.
- **Resource estimate:** `scripts/synth-estimate.sh` (Yosys; area only, see the [Stage 4 doc](../docs/stages/stage-4-apriltag-detector.md)). Keep multiplier operands at their natural width and memories in their own arrays, or the tools build far more than the design needs.
- **CI:** `scripts/lint-rtl.sh` (Verible lint and format check) runs on every `.sv` file here.

## Blocks

| Block | File | Status |
|---|---|---|
| Adaptive threshold | [`threshold/threshold.sv`](threshold/threshold.sv) | Bit-exact with the golden model in simulation (`sim/threshold`). One pixel per clock, 16-row pixel ring plus 4 tile-stat rows. Needs ≥3 idle cycles per row on average and about 12 rows of vertical blanking; `o_overrun` flags either being broken. Not yet synthesized |
| Connected components | [`components/components.sv`](components/components.sv) | Same partition, records (id, color, area, bounding box, 16 extremes) and closure order as the golden model in simulation (`sim/components`). One pixel per clock plus 2 cycles per row: an input stage forms runs and joins, a label resolver does union-find by size with recycled labels, and a record engine does one accumulator operation per clock. 3 × `MAX_W` labels are enough for any image. Not yet synthesized; see below |
| Corner scorecard (quad finding 4B) | [`quad/scorecard.sv`](quad/scorecard.sv) | Bit-exact with the golden model's `scorecard_quad()` (`sim/quad`): one component record in, verdict and rough quad out. At most 832 cycles on synthetic scenes and noise, and about 1,940 in the worst case (16 distinct extremes). One candidate quad per cycle and a bit-serial square root. Doesn't check the color; the quad finder feeds it black components. Not yet synthesized |
| Detector (PL top) | [`detector/detector.sv`](detector/detector.sv) | Quad finder and tag finish on one pixel stream, plus the row counter the RPU gates its jobs with. With the RPU's code in the loop it gives exactly the golden detector's tags (`sim/detector`). One frame size at a time; not yet synthesized |
| Quad finder (front end) | [`quad_finder/quad_finder.sv`](quad_finder/quad_finder.sv) | Threshold, components and scorecard joined: whole frames in, exactly the golden model's quads out, with frame IDs and per-frame counts (`sim/quad_finder`). Overload drops and counts. It keeps up with one pixel per clock on 1280×800. Not yet synthesized |
| Edge sampler (refinement, PL part) | [`refine/edge_sampler.sv`](refine/edge_sampler.sv) | Bit-exact with the golden model's `sample_edge()` at the Stage 4 fixed-point widths (`sim/refine`): edge jobs in (from the RPU), edge points out (to the RPU's line fit). Pipelined, one profile sample per clock: about 22 cycles per point on synthetic scenes. Its pixel port can be held off, so it shares the pixel window |
| Tag finish | [`tag_finish/tag_finish.sv`](tag_finish/tag_finish.sv) | Decode and the edge sampler on one pixel window; decode's reads go first. Both streams at once give exactly the golden results (`sim/tag_finish`) |
| ID lookup (decode) | [`decode/id_lookup.sv`](decode/id_lookup.sv) | Bit-exact with the golden model's `Tag36h11Lookup::decode()` (`sim/decode`): NAPA's four 9-bit block tables, up to 3 corrected bits, and a third only if the wrong cells touch. At most 43 cycles per lookup. ROMs come from `gv-model lookup-tables`. Reads are asynchronous for now; block RAM adds a cycle per compare |
| Tag sampler (decode, before the lookup) | [`decode/tag_sampler.sv`](decode/tag_sampler.sv) | Bit-exact with the golden model's `read_code()` at the Stage 4 widths (`sim/decode`): the projector's points stream in, the brightness planes are solved with bit-serial dividers while the cells are read, then cell values, sharpening, bits and the decision margin. Done at most 140 cycles after the last point. Pixels come 2×2 at a time from the pixel window, registered |
| Projector (decode, before the tag sampler) | [`decode/projector.sv`](decode/projector.sv) | Bit-exact with the golden model's `FixedProjector` (`sim/decode`): the 8 fixed-point homography coefficients the RPU sends (h0..h5 Q.14, h6, h7 Q.24) to the 100 sample points, each coordinate one exact division rounded once. Pipelined, one point per clock: 127 cycles per read |
| Decode | [`decode/decode.sv`](decode/decode.sv) | Projector, tag sampler and ID lookup joined: homographies in, reads out (code, margin, ID, rotation, Hamming distance) in order, with consecutive reads overlapping. About 214 cycles per read (`sim/decode`) |
| Pixel window | [`pixels/pixel_window.sv`](pixels/pixel_window.sv) | Pixel ring with a registered 2×2 read, in four banks. In UltraRAM, 8 pixels per word. The strip buffer's read side, shared by decode and the edge sampler |

### Connected components: speed and what's left

- **Speed.** At 1280 wide, the resolver takes about 920 cycles per row on a synthetic scene and 1,050 on noise (modelled means), and 1,195 and 1,203 for the longest rows (measured in RTL). A row lasts about 1,300 cycles at 120 FPS. Queues absorb busy rows, so only the average matters.
- **Worst case.** A 1-pixel checkerboard costs 2.5 cycles per pixel, so a frame with large areas of it falls behind and is cut and counted by the quad finder. A faster fabric clock than the pixel rate would cover it (2.5x) and gives margin everywhere else.
- **Memory.** Two copies of 4,096 records, about 490 bits each: 14 UltraRAM blocks by the first estimate. Synthetic scenes and noise used at most 909 labels, but 4,096 is UltraRAM's depth, so fewer wouldn't save any.
- **Timing.** A label find reads a FIFO, the label memory, `parent` and `size` in one cycle, and a record merge does 16 direction comparisons in one. Either may need a pipeline stage; neither has been through Vivado.
