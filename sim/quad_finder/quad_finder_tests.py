"""cocotb tests for rtl/quad_finder/quad_finder.sv: whole frames in, and the
quads out must be exactly the golden model's (threshold, components, scorecard
on black components of at least 16 px), in order, with the right frame ID and
per-frame counts. Overload must drop, count and recover."""

import os
import random
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # sim/, for gvsim

import cocotb  # noqa: E402
from cocotb.clock import Clock  # noqa: E402
from cocotb.triggers import RisingEdge  # noqa: E402
from gvsim import blocks, dump, gradient_with_squares, noise, read_pgm, scorecard_quads, synth_scene  # noqa: E402

CONFIG = os.environ.get("QF_CONFIG", "full")
SCENES = [(11, 256, 160, 2, (40, 90)), (100, 256, 160, 3, (24, 40)), (200, 480, 320, 2, (80, 150)),
          (500, 640, 400, 1, (250, 380))]


def golden(frame: tuple) -> dict:
    """Expected quads (id, corners, twice the area), counts, and all candidates."""
    rows = [r for r in dump(*frame)["components"] if int(r["color"]) == 0 and int(r["area"]) >= 16]
    recs = [(int(r["area"]), int(r["x_min"]), int(r["x_max"]), int(r["y_min"]), int(r["y_max"]),
             [(int(r[f"ex{k}"]), int(r[f"ey{k}"])) for k in range(16)]) for r in rows]
    res = scorecard_quads(recs) if recs else []
    small = sum((x1 - x0) ** 2 + (y1 - y0) ** 2 < 64 for _, x0, x1, y0, y1, _ in recs)
    return {"quads": [(int(r["id"]), q[1], q[2]) for r, q in zip(rows, res) if q[0] == 0],
            "candidates": len(rows), "small": small, "off_quad": sum(q[0] == 3 for q in res)}


class Bench:
    def __init__(self, dut):
        self.dut = dut
        self.quads: dict[int, list] = {}
        self.done: list[dict] = []

    async def reset(self):
        d = self.dut
        cocotb.start_soon(Clock(d.clk, 10, unit="ns").start())
        d.rst.value, d.i_valid.value, d.i_sof.value, d.min_white_black_diff.value = 1, 0, 0, 5
        for _ in range(3):
            await RisingEdge(d.clk)
        d.rst.value = 0
        await RisingEdge(d.clk)
        cocotb.start_soon(self._quads())
        cocotb.start_soon(self._done())

    async def _quads(self):
        d = self.dut
        while True:
            await RisingEdge(d.clk)
            if int(d.o_valid.value):
                corners = [(int(d.o_cx[k].value), int(d.o_cy[k].value)) for k in range(4)]
                f = int(d.o_frame.value)
                assert all(x["frame"] != f for x in self.done), f"quad of frame {f} after its o_frame_done"
                self.quads.setdefault(f, []).append((int(d.o_id.value), corners, d.o_area2.value.to_signed()))

    async def _done(self):
        d = self.dut
        while True:
            await RisingEdge(d.clk)
            if int(d.o_frame_done.value):
                self.done.append({k: int(getattr(d, f"o_fd_{k}").value) for k in (
                    "frame", "aborted", "candidates", "small", "dropped", "quads", "off_quad")})

    async def drive(self, frame: tuple, fid: int, h_blank: int, v_blank_rows: int = 16):
        d = self.dut
        w, h, pix = frame
        for y in range(h):
            for x in range(w):
                d.i_valid.value = 1
                d.i_sof.value = int(x == 0 and y == 0)
                d.i_frame.value, d.i_width.value, d.i_height.value = fid, w, h
                d.i_pix.value = pix[y * w + x]
                await RisingEdge(d.clk)
            d.i_valid.value = 0
            # The ID and size are only valid with i_sof.
            d.i_frame.value, d.i_width.value, d.i_height.value = 0xDEAD, 4, 4
            for _ in range(h_blank):
                await RisingEdge(d.clk)
        for _ in range(v_blank_rows * (w + h_blank)):
            await RisingEdge(d.clk)

    async def wait_done(self, n: int, limit: int = 3_000_000):
        for _ in range(limit // 1000):
            if len(self.done) >= n:
                return
            for _ in range(1000):
                await RisingEdge(self.dut.clk)
        raise AssertionError(f"{len(self.done)} of {n} frames done")

    def check(self, fid: int, want: dict):
        d = next(x for x in self.done if x["frame"] == fid)
        assert self.quads.get(fid, []) == want["quads"], f"frame {fid}: {self.quads.get(fid)} vs {want['quads']}"
        assert d == {"frame": fid, "aborted": 0, "candidates": want["candidates"], "small": want["small"],
                     "dropped": 0, "quads": len(want["quads"]), "off_quad": want["off_quad"]}, f"{d} vs {want}"

    def levels(self) -> str:
        d = self.dut
        return (f"pixel FIFO max {int(d.o_pix_level_max.value)}, record FIFO max {int(d.o_rec_level_max.value)}, "
                f"longest row {int(d.o_row_cycles_max.value)} cycles, labels max {int(d.o_labels_max.value)}")


def scene(seed, w, h, tags, px) -> tuple:
    with tempfile.TemporaryDirectory() as tmp:
        p = Path(tmp) / "s.pgm"
        synth_scene(p, seed, w, h, tags, px)
        return read_pgm(p)


@cocotb.test(skip=CONFIG != "full")
async def tag_scenes(dut):
    b = Bench(dut)
    await b.reset()
    for n, s in enumerate(SCENES):
        frame = scene(*s)
        want = golden(frame)
        assert want["quads"]
        await b.drive(frame, 1000 + n, h_blank=40)
        await b.wait_done(n + 1)
        b.check(1000 + n, want)
        dut._log.info(f"{frame[0]}x{frame[1]}: {len(want['quads'])} quads of {want['candidates']} candidates "
                      f"({want['small']} small); so far {b.levels()}")
    assert int(dut.o_error.value) == 0 and int(dut.o_frames_cut.value) == 0


@cocotb.test(skip=CONFIG != "full")
async def full_size_frame(dut):
    """1280x800 at one pixel per clock with 40 cycles of blanking per row."""
    b = Bench(dut)
    await b.reset()
    frame = scene(7, 1280, 800, 4, (60, 200))
    want = golden(frame)
    await b.drive(frame, 1, h_blank=40)
    await b.wait_done(1)
    b.check(1, want)
    assert int(dut.o_error.value) == 0 and int(dut.o_frames_cut.value) == 0
    dut._log.info(f"1280x800: {len(want['quads'])} quads of {want['candidates']} candidates; {b.levels()}")


@cocotb.test(skip=CONFIG != "full")
async def frames_back_to_back(dut):
    """Frames of mixed sizes and kinds, with only the vertical blanking the
    threshold needs, so later stages hold several frames at once."""
    b = Bench(dut)
    await b.reset()
    rng = random.Random(8)
    frames = [scene(*SCENES[0]), (97, 61, gradient_with_squares(rng, 97, 61)), (40, 30, bytes([100]) * 1200),
              (64, 48, blocks(rng, 64, 48)), (160, 100, noise(rng, 160, 100)), scene(*SCENES[1]),
              (161, 97, gradient_with_squares(rng, 161, 97))]
    wants = [golden(f) for f in frames]
    ids = [(0xFFFFFFFD + n) & 0xFFFFFFFF for n in range(len(frames))]  # wraps
    for n, f in enumerate(frames):
        if n >= 4:
            await b.wait_done(n - 3)  # at most 4 frames inside
        await b.drive(f, ids[n], h_blank=4)
    await b.wait_done(len(frames))
    for n, want in enumerate(wants):
        b.check(ids[n], want)
    assert [d["frame"] for d in b.done] == ids
    assert int(dut.o_error.value) == 0 and int(dut.o_frames_cut.value) == 0
    dut._log.info(b.levels())


@cocotb.test(skip=CONFIG != "tiny_pix")
async def pixel_overload_recovers(dut):
    """A checkerboard (the resolver's worst case, 2.5 cycles per pixel), with
    some noise, overflows the pixel FIFO mid-frame, repeatedly. Each
    slow, clean frame that follows must be exact, including those whose first
    pixel is 127 (a flat corner)."""
    b = Bench(dut)
    await b.reset()
    rng = random.Random(9)
    fid, cut, want = 0, 0, {}
    for n in range(8):
        # Cut in the middle of a row, with different geometry from the next frame.
        w, h = 120 + 17 * n, 60
        board = bytes(rng.randint(0, 255) if rng.random() < 0.03 else 250 * ((x + y) % 2)
                      for y in range(h) for x in range(w))
        await b.drive((w, h, board), fid, h_blank=4, v_blank_rows=40)
        fid += 1
        cut += 1
        w, h = 96, 64
        pix = bytearray(gradient_with_squares(rng, w, h))
        for y in range(12):
            pix[y * w : y * w + 16] = bytes([100]) * 16
        frame = (w, h, bytes(pix))
        want[fid] = golden(frame)
        await b.drive(frame, fid, h_blank=1500)
        fid += 1
    for _ in range(5000):
        await RisingEdge(dut.clk)
    assert int(dut.o_frames_cut.value) == cut, f"{int(dut.o_frames_cut.value)} cut, expected {cut}"
    for f, wa in want.items():
        b.check(f, wa)
    by_id = {d["frame"]: d for d in b.done}
    assert [d["frame"] for d in b.done] == list(range(fid))
    assert all(by_id[f]["aborted"] == int(f % 2 == 0) for f in by_id)
    assert int(dut.o_error.value) == 0
    dut._log.info(f"{cut} frames cut, {len(want)} clean frames exact after them")


@cocotb.test(skip=CONFIG != "tiny_rec")
async def record_overload_counts(dut):
    """A row of many candidates closing at once overflows a 4-record FIFO:
    they're counted, every quad that does come out is a golden one, in order,
    and the next frame is exact."""
    b = Bench(dut)
    await b.reset()
    w, h = 420, 24  # they close at frame end, so the marker finds the FIFO full
    img = bytearray([220]) * (w * h)
    for k in range(20):
        for y in range(12, 24):
            img[y * w + 4 + 20 * k : y * w + 16 + 20 * k] = bytes([30]) * 12
        for y in range(15, 21):
            img[y * w + 7 + 20 * k : y * w + 13 + 20 * k] = bytes([220]) * 6
    busy = (w, h, bytes(img))
    gold = golden(busy)
    assert len(gold["quads"]) == 20
    await b.drive(busy, 1, h_blank=40)
    await b.wait_done(1)
    d = b.done[0]
    assert d["candidates"] == gold["candidates"] and d["dropped"] > 0 and d["aborted"] == 0, d
    assert d["quads"] + d["dropped"] == 20 and len(b.quads[1]) == d["quads"], d
    rest = iter(gold["quads"])
    assert all(q in rest for q in b.quads[1]), "quads out of order or wrong"
    quiet = bytearray([220]) * (60 * 40)  # one candidate, then the counters must be clean
    for y in range(10, 22):
        quiet[y * 60 + 8 : y * 60 + 20] = bytes([30]) * 12
    for y in range(13, 19):
        quiet[y * 60 + 11 : y * 60 + 17] = bytes([220]) * 6
    frame = (60, 40, bytes(quiet))
    await b.drive(frame, 2, h_blank=40)
    await b.wait_done(2)
    b.check(2, golden(frame))
    assert int(dut.o_error.value) == 0
    dut._log.info(f"dropped {d['dropped']} of {d['candidates']} candidates")


@cocotb.test(skip=CONFIG != "few_labels")
async def label_overflow_is_flagged(dut):
    """Running out of labels mid-frame loses candidates, so it must set o_error."""
    b = Bench(dut)
    await b.reset()
    rng = random.Random(10)
    await b.drive((40, 30, bytes([100]) * 1200), 1, h_blank=40)
    await b.wait_done(1)
    assert int(dut.o_error.value) == 0
    await b.drive((160, 100, noise(rng, 160, 100)), 2, h_blank=40)
    await b.wait_done(2)
    assert int(dut.o_error.value) == 1
