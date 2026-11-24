"""cocotb tests for rtl/quad/scorecard.sv against the golden model's
scorecard_quad() (`gv-model quads`): the verdict, rough corners and twice the
area of real components from synthetic tag scenes, and of made-up records that
cover every verdict, the decision limits and the format's extremes."""

import math
import random
import sys
import tempfile
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # sim/, for gvsim

import cocotb  # noqa: E402
from cocotb.clock import Clock  # noqa: E402
from cocotb.triggers import RisingEdge, with_timeout  # noqa: E402
from gvsim import dump, noise, read_pgm, scorecard_quads, synth_scene  # noqa: E402

SCENES = [(11, 256, 160, 2, (40, 90)), (100, 256, 160, 3, (24, 40)), (200, 480, 320, 2, (80, 150)),
          (500, 640, 400, 1, (250, 380))]
DIRS = [(64, 0), (59, 24), (45, 45), (24, 59), (0, 64), (-24, 59), (-45, 45), (-59, 24),
        (-64, 0), (-59, -24), (-45, -45), (-24, -59), (0, -64), (24, -59), (45, -45), (59, -24)]
MAX = 4095


async def reset(dut) -> None:
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1
    dut.in_valid.value = 0
    for _ in range(3):
        await RisingEdge(dut.clk)
    dut.rst.value = 0
    await RisingEdge(dut.clk)


async def run(dut, rec: tuple, ident: int) -> tuple[tuple, int]:
    area, xmin, xmax, ymin, ymax, ex = rec
    assert int(dut.in_ready.value) == 1
    dut.in_id.value, dut.in_area.value = ident, area
    dut.in_xmin.value, dut.in_xmax.value = xmin, xmax
    dut.in_ymin.value, dut.in_ymax.value = ymin, ymax
    dut.in_ex.value = [p[0] for p in ex]
    dut.in_ey.value = [p[1] for p in ex]
    dut.in_valid.value = 1
    await RisingEdge(dut.clk)
    dut.in_valid.value = 0
    await with_timeout(RisingEdge(dut.o_valid), 50, "us")
    await RisingEdge(dut.clk)
    assert int(dut.o_id.value) == ident
    verdict = int(dut.o_verdict.value)
    cycles = int(dut.o_cycles.value)
    if verdict == 1:
        return (1, None, None), cycles
    corners = [(int(dut.o_cx[k].value), int(dut.o_cy[k].value)) for k in range(4)]
    return (verdict, corners, int(dut.o_area2.value.to_signed())), cycles


async def check(dut, records: list[tuple]) -> tuple[Counter, int]:
    verdicts: Counter = Counter()
    worst = 0
    for n, (rec, want) in enumerate(zip(records, scorecard_quads(records))):
        got, cycles = await run(dut, rec, (n * 2654435761) & 0xFFFFFFFF)
        assert got == want, f"record {n} {rec}: RTL {got}, golden {want}"
        verdicts[want[0]] += 1
        worst = max(worst, cycles)
    return verdicts, worst


def from_csv(r: dict) -> tuple:
    return (int(r["area"]), int(r["x_min"]), int(r["x_max"]), int(r["y_min"]), int(r["y_max"]),
            [(int(r[f"ex{k}"]), int(r[f"ey{k}"])) for k in range(16)])


def from_points(pts: list[tuple[int, int]], area: int) -> tuple:
    """The record of a blob whose hull is `pts` (extremes as the labeler picks
    them: the largest score, ties to the first in raster order)."""
    raster = sorted(pts, key=lambda p: (p[1], p[0]))
    ex = [max(raster, key=lambda p: dx * p[0] + dy * p[1]) for dx, dy in DIRS]
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    return (area, min(xs), max(xs), min(ys), max(ys), ex)


def convex_quad(rng: random.Random, size: float, cx: float, cy: float) -> list[tuple[int, int]]:
    """A rotated, slightly irregular quad, clipped to the coordinate range."""
    base = rng.uniform(0, 2 * math.pi)
    out = []
    for k in range(4):
        th = base + k * math.pi / 2 + rng.uniform(-0.25, 0.25)
        rad = size * rng.uniform(0.8, 1.2)
        out.append((min(MAX, max(0, round(cx + rad * math.cos(th)))), min(MAX, max(0, round(cy + rad * math.sin(th))))))
    return out


@cocotb.test()
async def components_of_tag_scenes(dut):
    """Every component of synthetic tag scenes and noise (either color: the
    scorecard doesn't look at it)."""
    await reset(dut)
    records = []
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "scene.pgm"
        for seed, w, h, tags, tag_px in SCENES:
            synth_scene(path, seed, w, h, tags, tag_px)
            records += [from_csv(r) for r in dump(*read_pgm(path))["components"]]
    records += [from_csv(r) for r in dump(96, 64, noise(random.Random(5), 96, 64))["components"]]
    verdicts, worst = await check(dut, records)
    assert verdicts[0] >= 8, f"only {verdicts[0]} quads accepted"
    dut._log.info(f"{len(records)} components, verdicts {dict(verdicts)}; at most {worst} cycles")


@cocotb.test()
async def quads_at_the_limits(dut):
    """Quads with extra hull points near the off-quad limit, sides near the
    minimum, and fill areas on both sides of each bound."""
    await reset(dut)
    rng = random.Random(21)
    shapes = []
    for _ in range(500):
        size = rng.choice([6, 7, 9, 20, 60, 250, 900, 2000])
        c = rng.uniform(size, MAX - size) if size < 2000 else 2048
        quad = convex_quad(rng, size, c, rng.uniform(size, MAX - size) if size < 2000 else 2048)
        pts = list(quad)
        for _ in range(rng.randint(0, 3)):  # push a point near a side outward
            k, t = rng.randrange(4), rng.uniform(0.1, 0.9)
            (x0, y0), (x1, y1) = quad[k], quad[(k + 1) % 4]
            side = math.hypot(x1 - x0, y1 - y0) or 1
            out = max(2.0, side / 20) * rng.uniform(0.5, 1.5)
            x, y = x0 + t * (x1 - x0) + out * (y1 - y0) / side, y0 + t * (y1 - y0) - out * (x1 - x0) / side
            pts.append((min(MAX, max(0, round(x))), min(MAX, max(0, round(y)))))
        shapes.append(pts)
    first = scorecard_quads([from_points(p, 1) for p in shapes])
    records = []
    for pts, (verdict, corners, area2) in zip(shapes, first):
        records.append(from_points(pts, (area2 or 100) // 3))
        if verdict != 4:
            continue
        sides = [math.dist(corners[k], corners[(k + 1) % 4]) for k in range(4)]
        den = 256 * area2 + sum(int(256 * s) for s in sides) + 512
        for pct in (25, 95):
            edge = pct * den // 51200
            records += [from_points(pts, edge + d) for d in (0, 1)]
    verdicts, worst = await check(dut, records)
    assert all(verdicts[v] >= 20 for v in (0, 2, 3, 4)), f"verdicts {dict(verdicts)}"
    dut._log.info(f"{len(records)} records, verdicts {dict(verdicts)}; at most {worst} cycles")


@cocotb.test()
async def exact_boundaries(dut):
    """Axis-aligned squares and octagons, where distances are whole pixels, so
    the limits are hit exactly: side 7/8/9, bulge 1..3 px and 5% of the side,
    and clusters of extremes at merge distance."""
    await reset(dut)
    records = []
    for s in (7, 8, 9, 10, 39, 40, 41, 60, 80, 100, 200):
        sq = [(50, 50), (50 + s, 50), (50 + s, 50 + s), (50, 50 + s)]
        records.append(from_points(sq, s * s // 2))
        for bulge in (1, 2, 3, s // 20, s // 20 + 1):
            records.append(from_points(sq + [(50 + s // 2, 50 - bulge)], s * s // 2))
            records.append(from_points(sq + [(50 + s + bulge, 50 + s // 2), (50 + s // 2, 50 + s + bulge)], s * s // 2))
        # With x = (s + 1)^2 the fill bounds are exactly 25% and 95% of x.
        records += [from_points(sq, a + d) for a in ((s + 1) ** 2 // 4, (s + 1) ** 2 * 95 // 100) for d in (-1, 0, 1)]
        for cut in (1, 2, 3):  # chopped corners: merge distance sqrt(2), then sqrt(8), ...
            c = [(50 + cut, 50), (50 + s - cut, 50), (50 + s, 50 + cut), (50 + s, 50 + s - cut),
                 (50 + s - cut, 50 + s), (50 + cut, 50 + s), (50, 50 + s - cut), (50, 50 + cut)]
            records.append(from_points(c, s * s // 2))
    verdicts, worst = await check(dut, records)
    dut._log.info(f"{len(records)} records, verdicts {dict(verdicts)}; at most {worst} cycles")


@cocotb.test()
async def off_quad_limit_to_the_last_bit(dut):
    """A point outside a side by the largest cross product the mean-side limit
    allows, or one more, on sides where the length's last bit (1/256 px) moves
    that limit."""
    await reset(dut)
    rng = random.Random(44)
    records = []
    while len(records) < 120:
        size = rng.choice([60, 150, 400])
        quad = convex_quad(rng, size, 1000, 1000)
        lens = [math.isqrt(round(math.dist(quad[k], quad[(k + 1) % 4]) ** 2) << 16) for k in range(4)]
        m = max(512 * 80, sum(lens))
        for k in range(4):
            (x0, y0), (x1, y1) = quad[k], quad[(k + 1) % 4]
            limit = m * lens[k] // (80 * 65536)
            if limit == m * (lens[k] - 1) // (80 * 65536) or lens[k] % 2 == 0:
                continue
            for c in (limit, limit + 1):
                # Pixels near the middle of the side with cross(u, v, p) == -c.
                hit = None
                if x1 != x0:
                    for x in range(min(x0, x1), max(x0, x1) + 1):
                        num = (y1 - y0) * (x - x0) - c  # = (x1 - x0) * (y - y0)
                        t = (x - x0) / (x1 - x0)
                        if 0.3 < t < 0.7 and num % (x1 - x0) == 0:
                            hit = (x, y0 + num // (x1 - x0))
                if hit:
                    records.append(from_points(quad + [hit], size * size))
    verdicts, worst = await check(dut, records)
    assert verdicts[0] >= 10 and verdicts[3] >= 10, f"verdicts {dict(verdicts)}"
    dut._log.info(f"{len(records)} records, verdicts {dict(verdicts)}; at most {worst} cycles")


@cocotb.test()
async def format_extremes(dut):
    """Any 16 points over the whole 12-bit range (not necessarily convex), dots,
    the full-range square, and the search's worst case."""
    await reset(dut)
    rng = random.Random(33)
    records = []
    for _ in range(150):
        span = rng.choice([1, 2, 4, 12, 100, MAX])
        x0, y0 = rng.randint(0, MAX - span), rng.randint(0, MAX - span)
        ex = [(x0 + rng.randint(0, span), y0 + rng.randint(0, span)) for _ in range(16)]
        for k in range(1, 16):  # some runs of near-duplicates
            if rng.random() < 0.4:
                ex[k] = tuple(min(MAX, max(0, v + rng.randint(-1, 1))) for v in ex[k - 1])
        xs, ys = [p[0] for p in ex], [p[1] for p in ex]
        records.append((rng.randint(1, (span + 1) ** 2), min(xs), max(xs), min(ys), max(ys), ex))
    records.append(from_points([(0, 0)], 1))
    records.append(from_points([(0, 0), (MAX, 0), (MAX, MAX), (0, MAX)], 4096 * 4096 // 2))
    records.append(from_points([(0, 0), (MAX, 0), (MAX, MAX), (0, MAX)], 4096 * 4096))
    ring = [(2048 + round(2000 * math.cos(k * math.pi / 8)), 2048 + round(2000 * math.sin(k * math.pi / 8)))
            for k in range(16)]
    records.append(from_points(ring, 12_000_000))
    verdicts, worst = await check(dut, records)
    assert verdicts[1] >= 5 and worst > 1820
    dut._log.info(f"{len(records)} records, verdicts {dict(verdicts)}; at most {worst} cycles")
