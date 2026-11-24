"""The whole detector: frames into the RTL, the RPU's C++ (through its C
interface) driving the RTL's job ports, and finished tags out. They must be
exactly the golden detector's (Stage 4 widths, no 4C): IDs, Hamming distances,
margins, and corners to the last bit.

The RPU answers at once here; what the real one adds is measured on hardware."""

import ctypes
import math
import os
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # sim/, for gvsim

import cocotb  # noqa: E402
from cocotb.clock import Clock  # noqa: E402
from cocotb.triggers import RisingEdge  # noqa: E402
from gvsim import JOB_FIELDS, detect, read_pgm, synth_scene  # noqa: E402

H_BLANK = 40
COUNTERS = ["quads", "dropped_full", "dropped_stale", "dropped_detections", "points_lost", "bad_projector",
            "quick_fail", "refine_fail", "decode_fail", "duplicates"]


def i32(v: int) -> int:
    return ctypes.c_int32(v).value


class Rpu:
    def __init__(self, w: int, h: int):
        self.lib = ctypes.CDLL(os.environ["GV_RPU_HOST"])
        self.lib.gv_rpu_new.restype = ctypes.c_void_p
        self.f = ctypes.c_void_p(self.lib.gv_rpu_new(w, h))
        self.buf = (ctypes.c_int32 * 1024)()

    def call(self, name: str, *args) -> int:
        return getattr(self.lib, f"gv_rpu_{name}")(self.f, *args)

    def quad(self, frame: int, cx: list[int], cy: list[int]) -> None:
        self.call("quad", ctypes.c_uint32(frame), (ctypes.c_int32 * 4)(*cx), (ctypes.c_int32 * 4)(*cy))

    def next_job(self, kind: str, n: int) -> list[int] | None:
        return list(self.buf[:n]) if self.call(f"next_{kind}", self.buf) else None

    def pop_frame(self) -> dict | None:
        if not self.call("pop_frame", self.buf):
            return None
        frame, aborted, count = self.buf[0] & 0xFFFFFFFF, self.buf[1], self.buf[2]
        tags = []
        for i in range(count):
            v = self.buf[3 + 12 * i : 15 + 12 * i]
            tags.append((*v[:4], list(zip(v[4::2], v[5::2]))))
        return {"frame": frame, "aborted": aborted, "tags": tags}

    def counters(self) -> dict:
        out = (ctypes.c_uint32 * len(COUNTERS))()
        self.call("counters", out)
        return dict(zip(COUNTERS, out))


def lowest_row(h: list[int], height: int) -> int:
    """The lowest image row a read samples (its white ring and cells)."""
    rows = []
    for u in range(-1, 18, 2):
        for v in range(-1, 18, 2):
            if (u in (-1, 17)) and (v in (-1, 17)):
                continue  # the ring has no corner samples
            rows.append((h[3] * u + h[4] * v + 16 * h[5]) / 2**14 / ((h[6] * u + h[7] * v) / 2**24 + 16))
    return min(height - 1, max(0, math.floor(max(rows))))


class Bench:
    """Frames in, and the RPU between the PL's results and its job ports."""

    def __init__(self, dut, w: int, h: int):
        self.dut, self.rpu, self.frames, self.cycle = dut, Rpu(w, h), [], 0
        self.rows, self.in_blanking = 0, True
        self.last_pixel: dict[int, int] = {}
        self.h, self.waited = h, 0  # reads that were released just as their last row came in

    async def reset(self) -> None:
        d = self.dut
        cocotb.start_soon(Clock(d.clk, 10, unit="ns").start())
        d.rst.value, d.i_valid.value, d.i_sof.value = 1, 0, 0
        d.min_white_black_diff.value, d.max_bits.value = 5, 3
        d.rd_job_valid.value = d.ed_job_valid.value = 0
        for _ in range(4):
            await RisingEdge(d.clk)
        d.rst.value = 0
        await RisingEdge(d.clk)
        cocotb.start_soon(self._run())

    async def _run(self) -> None:
        d, r = self.dut, self.rpu
        rd_out = ed_out = False  # a job is on the port
        rd_h: list[int] = []
        progress = None
        while True:
            await RisingEdge(d.clk)
            self.cycle += 1
            if int(d.q_valid.value):
                r.quad(int(d.q_frame.value), [int(d.q_cx[k].value) for k in range(4)],
                       [int(d.q_cy[k].value) for k in range(4)])
            if int(d.q_frame_done.value):
                r.call("frame_done", ctypes.c_uint32(int(d.q_fd_frame.value)), int(d.q_fd_aborted.value))
                assert int(d.q_fd_dropped.value) == 0
            if int(d.rd_valid.value):
                r.call("read", ctypes.c_uint32(int(d.rd_tag.value)), int(d.rd_ok.value), d.rd_margin.value.to_signed(),
                       int(d.rd_found.value), int(d.rd_id.value), int(d.rd_rotation.value), int(d.rd_hamming.value))
            if int(d.pt_valid.value):
                r.call("point", ctypes.c_uint32(int(d.ed_tag.value)), d.pt_x.value.to_signed(), d.pt_y.value.to_signed())
            if int(d.ed_done.value):
                r.call("edge_done", ctypes.c_uint32(int(d.ed_tag.value)))
            now = (int(d.p_frame.value), int(d.p_rows.value))
            if now != progress:
                progress = now
                r.call("progress", ctypes.c_uint32(now[0]), now[1])
            if rd_out and int(d.rd_job_ready.value):
                rd_out = False
                # Rows counted by the bench itself (complete ones), not the RTL.
                low, rows = lowest_row(rd_h, self.h), self.rows
                assert low < rows or self.in_blanking, f"a read of row {low} was sent with {rows} rows in"
                self.waited += low + 4 >= rows and not self.in_blanking
                d.rd_job_valid.value = 0
            if ed_out and int(d.ed_job_ready.value):
                ed_out = False
                d.ed_job_valid.value = 0
            if not rd_out and (j := r.next_job("read", 9)):
                d.rd_job_tag.value, d.rd_job_h.value = j[0] & 0xFFFFFFFF, j[1:]
                d.rd_job_valid.value, rd_out, rd_h = 1, True, j[1:]
            if not ed_out and (j := r.next_job("edge", 13)):
                d.ed_job_tag.value = j[0] & 0xFFFFFFFF
                for k, v in zip(JOB_FIELDS, j[1:]):
                    getattr(d, k).value = v
                d.ed_job_valid.value, ed_out = 1, True
            if f := r.pop_frame():
                f["tail"] = self.cycle - self.last_pixel[f["frame"]]
                self.frames.append(f)

    async def drive(self, fid: int, w: int, h: int, pixels: bytes, v_blank_rows: int) -> None:
        d = self.dut
        d.i_frame.value, d.i_width.value, d.i_height.value = fid, w, h
        self.in_blanking = False
        for y in range(h):
            self.rows = y
            for x in range(w):
                d.i_valid.value, d.i_sof.value = 1, int(x == 0 and y == 0)
                d.i_pix.value = pixels[y * w + x]
                await RisingEdge(d.clk)
            d.i_valid.value = 0
            if y == h - 1:
                self.last_pixel[fid] = self.cycle
            for _ in range(H_BLANK):
                await RisingEdge(d.clk)
        self.in_blanking = True
        for _ in range(v_blank_rows * (w + H_BLANK)):
            await RisingEdge(d.clk)

    async def wait(self, n: int) -> None:
        for _ in range(300):
            if len(self.frames) >= n:
                return
            for _ in range(1000):
                await RisingEdge(self.dut.clk)
        raise AssertionError(f"{len(self.frames)} of {n} frames finished; {self.rpu.counters()}")

    def check_clean(self) -> None:
        d, c = self.dut, self.rpu.counters()
        assert int(d.o_error.value) == 0 and int(d.o_frames_cut.value) == 0
        assert not any(c[k] for k in ("dropped_full", "dropped_stale", "dropped_detections", "points_lost")), c


def scene(*s) -> tuple[tuple, list]:
    with tempfile.TemporaryDirectory() as tmp:
        p = Path(tmp) / "s.pgm"
        synth_scene(p, *s)
        return read_pgm(p), detect(p)


async def run_scenes(dut, scenes: list[tuple], v_blank_rows: int) -> Bench:
    """Scenes of one size, back to back, as consecutive frames."""
    w, h = scenes[0][1:3]
    b = Bench(dut, w, h)
    await b.reset()
    frames = [scene(*s) for s in scenes]
    for n, (f, _) in enumerate(frames):
        await b.drive((0xFFFFFFFE + n) & 0xFFFFFFFF, *f, v_blank_rows)  # IDs wrap
    await b.wait(len(frames))
    for n, (_, want) in enumerate(frames):
        got = b.frames[n]
        assert got["frame"] == (0xFFFFFFFE + n) & 0xFFFFFFFF and not got["aborted"]
        assert got["tags"] == want, f"scene {scenes[n]}: got {got['tags']}, golden {want}; {b.rpu.counters()}"
    b.check_clean()
    tags = sum(len(want) for _, want in frames)
    assert tags > 0
    dut._log.info(f"{b.waited} reads waited for their last rows")
    dut._log.info(f"{w}x{h}: {len(frames)} frames, {tags} tags, all bit-exact; tail after the last pixel "
                  f"{[f['tail'] for f in b.frames]} cycles; {b.rpu.counters()}")
    return b


@cocotb.test()
async def small_frames(dut):
    await run_scenes(dut, [(s, 256, 160, 3, (24, 60)) for s in range(100, 110)], 20)


@cocotb.test()
async def medium_frames(dut):
    """Larger tags, whose reads reach well below the tag."""
    b = await run_scenes(dut, [(s, 640, 400, 3, (60, 250)) for s in range(500, 504)], 20)
    assert b.waited > 0, "no read had to wait for its rows, so that wasn't tested"


@cocotb.test()
async def full_size_frame(dut):
    await run_scenes(dut, [(7, 1280, 800, 4, (60, 400))], 20)
