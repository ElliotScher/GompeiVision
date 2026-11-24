"""cocotb tests for rtl/refine/edge_sampler.sv against the golden model.

`gv-model synth` renders tag scenes; `gv-model edges` gives every edge job the
detector hands the PL edge sampler at the Stage 4 fixed-point widths, with the
edge points it must return. The RTL must return exactly those points.
"""

import math
import random
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # sim/, for gvsim

import cocotb  # noqa: E402
from cocotb.clock import Clock  # noqa: E402
from cocotb.triggers import RisingEdge, Timer  # noqa: E402
from gvsim import JOB_FIELDS, edge_jobs, read_pgm, sample_edges, synth_scene, write_pgm  # noqa: E402



class Memory:
    """The 2x2 pixel window behind a port that is sometimes busy. Requests not
    granted must hold; a neighbor outside the image reads as random."""

    def __init__(self, dut):
        self.dut, self.rng, self.busy = dut, random.Random(3), 0.0
        self.set(1, 1, b"\0")
        dut.rd_ready.value = 1
        cocotb.start_soon(self._run())

    def set(self, w: int, h: int, pixels: bytes) -> None:
        self.w, self.h, self.pixels = w, h, pixels
        self.dut.img_w.value, self.dut.img_h.value = w, h

    async def _run(self):
        d = self.dut
        while True:
            await RisingEdge(d.clk)
            if int(d.rd_en.value) and int(d.rd_ready.value):
                x, y = int(d.rd_x.value), int(d.rd_y.value)
                assert x < self.w and y < self.h, f"read of ({x}, {y}) outside {self.w}x{self.h}"
                d.rd_p.value = [
                    self.pixels[v * self.w + u] if u < self.w and v < self.h else self.rng.randrange(256)
                    for u, v in ((x, y), (x + 1, y), (x, y + 1), (x + 1, y + 1))]
            else:
                d.rd_p.value = [self.rng.randrange(256) for _ in range(4)]
            d.rd_ready.value = int(self.rng.random() >= self.busy)


async def reset(dut) -> None:
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1
    dut.in_valid.value = 0
    for _ in range(3):
        await RisingEdge(dut.clk)
    dut.rst.value = 0
    await RisingEdge(dut.clk)


async def run_jobs(dut, jobs: list[dict]) -> tuple[list[list], int]:
    """All jobs back to back: each one's points, and the cycles taken."""
    results: list[list] = [[]]

    async def collect():
        n = 0
        while len(results) <= len(jobs):
            await RisingEdge(dut.clk)
            n += 1
            assert n < 200_000, f"{len(results) - 1} of {len(jobs)} jobs done"
            if int(dut.pt_valid.value) or int(dut.done.value):
                assert int(dut.o_tag.value) == 0xFFFF0000 + len(results) - 1, "wrong tag"
            if int(dut.pt_valid.value):
                results[-1].append((dut.pt_x.value.to_signed(), dut.pt_y.value.to_signed()))
            if int(dut.done.value):
                results.append([])
        return n

    collector = cocotb.start_soon(collect())
    for n, job in enumerate(jobs):
        for k in JOB_FIELDS:
            getattr(dut, k).value = job[k]
        dut.in_tag.value = 0xFFFF0000 + n
        dut.in_valid.value = 1
        while True:
            await Timer(1, unit="ns")
            ready = int(dut.in_ready.value)
            await RisingEdge(dut.clk)
            if ready:
                break
        dut.in_valid.value = 0
    cycles = await collector
    return results[:-1], cycles


def check(got: list[list], want: list[list], what: str) -> None:
    for n, (g, w) in enumerate(zip(got, want)):
        if g != w:
            first = next((i for i, (a, b) in enumerate(zip(g, w)) if a != b), min(len(g), len(w)))
            raise AssertionError(f"{what}, job {n}: {len(g)} points from RTL, {len(w)} golden; first difference at "
                                 f"point {first}: RTL {g[first:first + 1]}, golden {w[first:first + 1]}")


@cocotb.test()
async def tag_scenes(dut):
    """Every edge job the detector makes, with the pixel port free and busy."""
    await reset(dut)
    mem = Memory(dut)
    total_jobs = points = cycles = samples = 0
    for seed, (w, h, tags, tag_px) in [
        (11, (256, 160, 2, (40, 90))),
        (100, (256, 160, 3, (24, 40))),  # small tags: short windows
        (200, (480, 320, 2, (80, 150))),  # large tags: full 6 px windows, long edges
        (300, (256, 160, 3, (40, 90))),
    ]:
        # Random placement sometimes leaves no tag that reaches refinement: take
        # the first seed from `seed` whose scene has edge jobs.
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "scene.pgm"
            for seed in range(seed, seed + 20):
                synth_scene(path, seed, w, h, tags, tag_px)
                jobs = edge_jobs(path)
                if jobs:
                    break
            assert jobs, f"seed {seed}: no tags were refined, so nothing was tested"
            mem.set(*read_pgm(path))
        want = [j["points"] for j in jobs]
        mem.busy = 0.0
        got, c = await run_jobs(dut, jobs)
        check(got, want, f"seed {seed}")
        mem.busy = 0.5
        check((await run_jobs(dut, jobs))[0], want, f"seed {seed}, port half busy")
        total_jobs += len(jobs)
        points += sum(map(len, want))
        cycles += c
        samples += sum(j["count"] * (2 * j["half_steps"] + 1) for j in jobs)
    # One profile sample per cycle, plus the pipeline's latency per scene.
    assert cycles < 1.05 * samples + 500, f"{cycles} cycles for {samples} profile samples"
    dut._log.info(f"{total_jobs} edge jobs, {points} edge points, all bit-exact; {cycles / points:.1f} cycles per "
                  f"point ({samples} profile samples in {cycles} cycles)")


@cocotb.test()
async def arbitrary_jobs(dut):
    """Random jobs over noise, a gradient and flat areas: every half width,
    either direction rule, lines that leave the image or touch its edges
    exactly, empty jobs, and short jobs after long ones."""
    await reset(dut)
    mem = Memory(dut)
    rng = random.Random(8)
    npts = empty = 0
    for case in range(4):
        w, h = [(97, 61), (64, 48), (120, 80), (40, 40)][case]
        pix = [bytes(rng.randrange(256) for _ in range(w * h)), bytes(rng.choice([0, 255]) for _ in range(w * h)),
               bytes(min(255, 2 * x + (90 if y > 40 else 0)) for y in range(h) for x in range(w)),
               bytes(100 + (x // 13 + y // 17) % 2 * 80 for y in range(h) for x in range(w))][case]
        jobs = []
        for _ in range(150):
            th = rng.choice([0, math.pi / 2, math.pi, math.pi / 4, rng.uniform(0, 2 * math.pi)])
            dx, dy = round(4096 * math.cos(th)), round(4096 * math.sin(th))
            s = rng.choice([1, -1])
            half = rng.randint(1, 13)
            on_grid = rng.random() < 0.4
            jobs.append({
                "a_x": rng.randint(-10, 2 * w + 10) * 128 if on_grid else rng.randint(-2000, w * 256 + 2000),
                "a_y": rng.randint(-10, 2 * h + 10) * 128 if on_grid else rng.randint(-2000, h * 256 + 2000),
                "d_x": dx, "d_y": dy, "n_x": s * dy, "n_y": -s * dx,
                "t0": rng.randint(0, 1000), "spacing": 384, "count": rng.choice([0, 1, 3, 20]), "half_steps": half,
                "window_steps": rng.randint(0, half), "check_direction": rng.randint(0, 1)})
        for half in (2, 5, 13):  # search lines ending on, or 1/256 px past, each border
            for off in (-1, 0, 1):
                for far in (0, 1):
                    y = h * 256 - 128 - 128 * half + off if far else 128 + 128 * half + off
                    x = w * 256 - 128 - 128 * half + off if far else 128 + 128 * half + off
                    base = {"t0": 600, "spacing": 384, "count": 12, "half_steps": half, "window_steps": half - 1,
                            "check_direction": 0}
                    jobs.append({"a_x": 600, "a_y": y, "d_x": 4096, "d_y": 0, "n_x": 0, "n_y": 4096, **base})
                    jobs.append({"a_x": x, "a_y": 600, "d_x": 0, "d_y": 4096, "n_x": -4096, "n_y": 0, **base})
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "img.pgm"
            write_pgm(path, w, h, pix)
            want = sample_edges(path, jobs)
        mem.set(w, h, pix)
        mem.busy = 0.0 if case % 2 else 0.3
        check((await run_jobs(dut, jobs))[0], want, f"case {case}")
        npts += sum(map(len, want))
        empty += sum(not p for p in want)
    assert npts > 500 and empty > 20
    dut._log.info(f"{4 * (150 + 36)} jobs, {npts} points, {empty} jobs without points, all bit-exact")
