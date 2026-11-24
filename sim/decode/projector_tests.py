"""cocotb tests for rtl/decode/projector.sv against the golden model's
FixedProjector at the Stage 4 widths (`gv-model decodes`): the 100 sample
points of every code read the detector makes on synthetic tag scenes."""

import random
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # sim/, for gvsim

import cocotb  # noqa: E402
from cocotb.clock import Clock  # noqa: E402
from cocotb.triggers import RisingEdge  # noqa: E402
from gvsim import code_reads, gv_model, synth_scene  # noqa: E402

SCENES = [(11, 256, 160, 2, (40, 90)), (100, 256, 160, 3, (24, 40)), (200, 480, 320, 2, (80, 150)),
          (500, 640, 400, 1, (250, 380))]


async def reset(dut) -> None:
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1
    dut.start.value = 0
    for _ in range(3):
        await RisingEdge(dut.clk)
    dut.rst.value = 0
    await RisingEdge(dut.clk)


async def project(dut, h: list[int]) -> tuple[list, int]:
    for name, v in zip(["h0", "h1", "h2", "h3", "h4", "h5", "h6", "h7"], h):
        getattr(dut, name).value = v
    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0
    pts: dict[int, tuple[int, int]] = {}
    cycles = 0
    while True:
        await RisingEdge(dut.clk)
        cycles += 1
        if int(dut.pt_valid.value):
            pts[int(dut.pt_idx.value)] = (dut.pt_x.value.to_signed(), dut.pt_y.value.to_signed())
        if int(dut.done.value):
            return [pts.get(i) for i in range(100)], cycles
        assert cycles < 10_000


@cocotb.test()
async def sample_points_on_tag_scenes(dut):
    await reset(dut)
    total = worst = 0
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "scene.pgm"
        for seed, w, h, tags, tag_px in SCENES:
            synth_scene(path, seed, w, h, tags, tag_px)
            for n, r in enumerate(code_reads(path)):
                got, cycles = await project(dut, r["h"])
                worst = max(worst, cycles)
                for i, (g, want) in enumerate(zip(got, r["points"])):
                    assert g == tuple(want), f"seed {seed}, read {n}, point {i}: RTL {g}, golden {tuple(want)}"
                total += 1
    assert total > 20, f"only {total} reads"
    dut._log.info(f"{total} reads x 100 points, all bit-exact; {worst} cycles per read")


def square_to_quad(q):
    """Heckbert's square-to-quad homography (the model's square_to_quad)."""
    (x0, y0), (x1, y1), (x2, y2), (x3, y3) = q
    sx, sy = x0 - x1 + x2 - x3, y0 - y1 + y2 - y3
    dx1, dx2, dy1, dy2 = x1 - x2, x3 - x2, y1 - y2, y3 - y2
    den = dx1 * dy2 - dx2 * dy1
    g = (sx * dy2 - dx2 * sy) / den
    h = (dx1 * sy - sx * dy1) / den
    return [x1 - x0 + g * x1, x3 - x0 + h * x3, x0, y1 - y0 + g * y1, y3 - y0 + h * y3, y0, g, h]


def golden_project(coeffs: list[int]):
    out = subprocess.run([gv_model(), "project", *map(str, coeffs)], check=True, capture_output=True,
                         text=True).stdout.split()
    if out[0] != "valid":
        return None
    return [tuple(int(c) for c in p.split(":")) for p in out[1:]]


@cocotb.test()
async def extreme_projectors(dut):
    """Valid projectors at the format's limits: quads up to +-32000 px, tags
    tens of thousands of pixels wide, strong perspective."""
    await reset(dut)
    rng = random.Random(7)
    checked = 0
    while checked < 60:
        span = rng.choice([50, 2_000, 30_000])
        cx, cy = rng.uniform(-30_000, 30_000), rng.uniform(-30_000, 30_000)
        quad = [(cx + rng.uniform(-span, span), cy + rng.uniform(-span, span)) for _ in range(4)]
        try:
            hf = square_to_quad(quad)
        except ZeroDivisionError:
            continue
        coeffs = [round(v * (1 << 14)) for v in hf[:6]] + [round(v * (1 << 24)) for v in hf[6:]]
        if any(abs(c) >= 1 << 31 for c in coeffs):
            continue
        want = golden_project(coeffs)
        if want is None:
            continue
        got, _ = await project(dut, coeffs)
        for i, (g, w) in enumerate(zip(got, want)):
            assert g == w, f"quad {quad}, point {i}: RTL {g}, golden {w}"
        checked += 1
    dut._log.info(f"{checked} extreme projectors, all bit-exact")
