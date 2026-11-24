"""cocotb tests for rtl/decode/decode.sv with its pixel memory: homographies in
and exactly the golden reads out, in order, with back-to-back jobs (so reads
overlap) and with gaps."""

import random
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # sim/, for gvsim

import cocotb  # noqa: E402
from cocotb.clock import Clock  # noqa: E402
from cocotb.triggers import RisingEdge, Timer  # noqa: E402
from gvsim import code_reads, read_pgm, read_tags, synth_scene, write_pgm  # noqa: E402
from projector_tests import square_to_quad  # noqa: E402

SCENES = [(11, 256, 160, 2, (40, 90)), (100, 256, 160, 3, (24, 40)), (200, 480, 320, 2, (80, 150)),
          (300, 256, 160, 3, (40, 90))]


async def reset(dut) -> None:
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value, dut.wr_en.value, dut.in_valid.value, dut.max_bits.value = 1, 0, 0, 3
    for _ in range(3):
        await RisingEdge(dut.clk)
    dut.rst.value = 0
    await RisingEdge(dut.clk)


async def load_image(dut, w: int, h: int, pixels: bytes) -> None:
    dut.img_w.value, dut.img_h.value = w, h
    dut.wr_en.value = 1
    for y in range(h):
        for x in range(w):
            dut.wr_x.value, dut.wr_y.value, dut.wr_data.value = x, y, pixels[y * w + x]
            await RisingEdge(dut.clk)
    dut.wr_en.value = 0
    await RisingEdge(dut.clk)


async def run_jobs(dut, jobs: list[list[int]], gaps: random.Random | None = None) -> tuple[list, int]:
    """Results by job, and the cycles from the first job to the last result."""
    out: list = []

    async def collect():
        while len(out) < len(jobs):
            await RisingEdge(dut.clk)
            if int(dut.o_valid.value):
                assert int(dut.o_tag.value) == 0xFFFF0000 + len(out), "out of order"
                ok, found = int(dut.o_read_ok.value) == 1, int(dut.o_found.value) == 1
                out.append({
                    "ok": ok, "code": int(dut.o_code.value), "margin": dut.o_margin.value.to_signed(),
                    "match": (int(dut.o_id.value), int(dut.o_rotation.value), int(dut.o_hamming.value))
                    if found else None})

    async def count():
        n = 0
        while len(out) < len(jobs):
            await RisingEdge(dut.clk)
            n += 1
            assert n < 5000 + 2000 * len(jobs), f"{len(out)} of {len(jobs)} results"
        return n

    collector, counter = cocotb.start_soon(collect()), cocotb.start_soon(count())
    for n, h in enumerate(jobs):
        while gaps and gaps.random() < 0.99:
            await RisingEdge(dut.clk)
        dut.in_valid.value, dut.in_tag.value, dut.in_h.value = 1, 0xFFFF0000 + n, h
        while True:
            await Timer(1, unit="ns")
            ready = int(dut.in_ready.value)
            await RisingEdge(dut.clk)
            if ready:
                break
        dut.in_valid.value = 0
        # Coefficients need only hold while in_valid.
        dut.in_h.value = [0x55555555] * 8
    await collector
    cycles = await counter
    assert int(dut.o_error.value) == 0
    return out, cycles


def check(got: list, want: list, what: str) -> None:
    for n, (g, w) in enumerate(zip(got, want)):
        if not w["ok"]:
            assert not g["ok"] and g["match"] is None, f"{what}, read {n}: RTL {g}, golden failed read"
        else:
            assert g == w, f"{what}, read {n}: RTL {g}, golden {w}"


@cocotb.test()
async def reads_of_tag_scenes(dut):
    """Every code read the detector makes, back to back and with gaps."""
    await reset(dut)
    total = found = cycles = 0
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "scene.pgm"
        for s in SCENES:
            synth_scene(path, *s)
            jobs = [r["h"] for r in code_reads(path)]
            want = read_tags(path, jobs)
            assert all(want) and [w["ok"] for w in want] == [r["ok"] for r in code_reads(path)]
            await load_image(dut, *read_pgm(path))
            got, c = await run_jobs(dut, jobs)
            check(got, want, f"scene {s[0]}")
            got, _ = await run_jobs(dut, jobs, random.Random(5))
            check(got, want, f"scene {s[0]} with gaps")
            total, cycles, found = total + len(jobs), cycles + c, found + sum(w["match"] is not None for w in want)
    assert found >= 5
    dut._log.info(f"{total} reads, {found} IDs; {cycles / total:.0f} cycles per read back to back")


@cocotb.test()
async def arbitrary_quads(dut):
    """Random quads over noise and a tag scene: reversed borders, quads partly
    or mostly outside (too few border samples for a plane, cells with no
    value), tiny and huge ones, and points right on the image's edges."""
    await reset(dut)
    rng = random.Random(6)
    kinds = {"ok": 0, "failed": 0}
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "img.pgm"
        for case in range(3):
            if case == 0:
                synth_scene(path, *SCENES[0])
            else:
                w, h = (64, 48) if case == 1 else (97, 61)
                write_pgm(path, w, h, bytes(rng.choice([0, 255]) if case == 1 else rng.randrange(256)
                                            for _ in range(w * h)))
            w, h, pixels = read_pgm(path)
            jobs = []
            while len(jobs) < 100:
                size = rng.choice([3, 10, 40, 150, 2000])
                cx, cy = rng.uniform(-20, w + 20), rng.uniform(-20, h + 20)
                if rng.random() < 0.3:  # axis-aligned on whole and half pixels
                    x, y, d = rng.randint(-8, w), rng.randint(-8, h), rng.choice([8, 16, 32])
                    quad = [(x, y), (x + d, y), (x + d, y + d), (x, y + d)]
                else:
                    quad = [(cx + sx * size * rng.uniform(0.5, 1), cy + sy * size * rng.uniform(0.5, 1))
                            for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
                hf = square_to_quad(quad)
                c = [round(v * (1 << 14)) for v in hf[:6]] + [round(v * (1 << 24)) for v in hf[6:]]
                if all(abs(v) < 1 << 31 for v in c):
                    jobs.append(c)
            want = read_tags(path, jobs)
            jobs = [j for j, r in zip(jobs, want) if r]
            want = [r for r in want if r]
            await load_image(dut, w, h, pixels)
            got, _ = await run_jobs(dut, jobs)
            check(got, want, f"case {case}")
            for r in want:
                kinds["ok" if r["ok"] else "failed"] += 1
    assert min(kinds.values()) >= 20, kinds
    dut._log.info(f"reads: {kinds}")
