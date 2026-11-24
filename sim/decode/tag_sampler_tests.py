"""cocotb tests for rtl/decode/tag_sampler.sv against the golden model's
read_code() at the Stage 4 widths (`gv-model decodes`): every code read the
detector makes on synthetic tag scenes, tags and non-tag candidates alike."""

import random
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # sim/, for gvsim

import cocotb  # noqa: E402
from cocotb.clock import Clock  # noqa: E402
from cocotb.triggers import RisingEdge  # noqa: E402
from gvsim import code_reads, read_pgm, synth_scene  # noqa: E402



class Memory:
    """The 2x2 pixel window: pixels one cycle after the request. A neighbor
    outside the image reads as random."""

    def __init__(self, dut):
        self.dut, self.rng = dut, random.Random(3)
        self.set(1, 1, b"\0")
        cocotb.start_soon(self._run())

    def set(self, w: int, h: int, pixels: bytes) -> None:
        self.w, self.h, self.pixels = w, h, pixels
        self.dut.img_w.value, self.dut.img_h.value = w, h

    async def _run(self):
        d = self.dut
        while True:
            await RisingEdge(d.clk)
            if int(d.rd_en.value):
                x, y = int(d.rd_x.value), int(d.rd_y.value)
                d.rd_p.value = [
                    self.pixels[v * self.w + u] if u < self.w and v < self.h else self.rng.randrange(256)
                    for u, v in ((x, y), (x + 1, y), (x, y + 1), (x + 1, y + 1))]
            else:
                d.rd_p.value = [self.rng.randrange(256) for _ in range(4)]


async def reset(dut) -> None:
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1
    dut.pt_valid.value = 0
    for _ in range(3):
        await RisingEdge(dut.clk)
    dut.rst.value = 0
    await RisingEdge(dut.clk)


async def run_read(dut, points: list, gaps: random.Random | None) -> tuple[bool, int, int, int]:
    """Points back to back, or with random gaps. Cycles from the last point."""
    for i, (x, y) in enumerate(points):
        dut.pt_valid.value = 1
        dut.pt_idx.value = i
        dut.pt_x.value = x
        dut.pt_y.value = y
        await RisingEdge(dut.clk)
        dut.pt_valid.value = 0
        while gaps and i < len(points) - 1 and gaps.random() < 0.8:  # sometimes slower than the plane solve
            await RisingEdge(dut.clk)
    cycles = 0
    while True:
        await RisingEdge(dut.clk)
        cycles += 1
        if int(dut.done.value):
            assert int(dut.o_error.value) == 0
            return bool(int(dut.ok.value)), int(dut.code.value), dut.margin.value.to_signed(), cycles
        assert cycles < 10_000


@cocotb.test()
async def code_reads_on_tag_scenes(dut):
    await reset(dut)
    mem = Memory(dut)
    gaps = random.Random(4)
    total = oks = worst = 0
    for seed, (w, h, tags, tag_px) in [(11, (256, 160, 2, (40, 90))), (100, (256, 160, 3, (24, 40))),
                                       (200, (480, 320, 2, (80, 150))), (300, (256, 160, 3, (40, 90)))]:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "scene.pgm"
            for seed in range(seed, seed + 20):
                synth_scene(path, seed, w, h, tags, tag_px)
                reads = code_reads(path)
                if any(r["ok"] and r["margin"] > 400 for r in reads):  # at least one real tag
                    break
            _, _, pixels = read_pgm(path)
        mem.set(w, h, pixels)
        for n, r in enumerate(reads):
            ok, code, margin, cycles = await run_read(dut, r["points"], gaps if n % 4 == 3 else None)
            worst = max(worst, cycles)
            want = (r["ok"], r["code"] if r["ok"] else 0, r["margin"] if r["ok"] else 0)
            assert (ok, code if ok else 0, margin if ok else 0) == want, (
                f"seed {seed}, read {n}: RTL ok {ok} code {code:09x} margin {margin}; "
                f"golden ok {want[0]} code {want[1]:09x} margin {want[2]}")
            total += 1
            oks += ok
    dut._log.info(f"{total} code reads ({oks} ok), all bit-exact; worst {worst} cycles after the last point")

