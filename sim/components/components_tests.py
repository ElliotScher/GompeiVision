"""cocotb tests for rtl/components/components.sv against the golden model.

Each test image is thresholded by `gv-model dump`; the RTL gets the
thresholded image and must emit exactly the golden components.csv: the same
records in the same order (id, color, area, bounding box, closing row and all
16 extreme pixels).
"""

import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # sim/, for gvsim

import cocotb  # noqa: E402
from cocotb.clock import Clock  # noqa: E402
from cocotb.triggers import RisingEdge  # noqa: E402
from gvsim import blocks, dump, gradient_with_squares, noise  # noqa: E402

FIELDS = ["id", "white", "area", "xmin", "xmax", "ymin", "ymax", "closed_after_row"]


def expected(rows: list[dict]) -> list[tuple]:
    out = []
    for r in rows:
        ex = tuple((int(r[f"ex{k}"]), int(r[f"ey{k}"])) for k in range(16))
        out.append(
            (int(r["id"]), int(r["color"]) == 255, int(r["area"]), int(r["x_min"]), int(r["x_max"]),
             int(r["y_min"]), int(r["y_max"]), int(r["closed_after_row"]), ex)
        )
    return out


async def reset(dut) -> None:
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1
    dut.in_valid.value = 0
    dut.in_sof.value = 0
    dut.in_pix.value = 0
    dut.i_width.value = 4
    dut.i_height.value = 4
    for _ in range(3):
        await RisingEdge(dut.clk)
    dut.rst.value = 0
    await RisingEdge(dut.clk)


async def drive(dut, w: int, h: int, pixels: bytes) -> None:
    """Ready/valid: each pixel is held until the block takes it."""
    for i in range(w * h):
        dut.in_valid.value = 1
        dut.in_sof.value = int(i == 0)
        dut.i_width.value = w
        dut.i_height.value = h
        dut.in_pix.value = pixels[i]
        await RisingEdge(dut.clk)
        while not int(dut.in_ready.value):
            await RisingEdge(dut.clk)
    dut.in_valid.value = 0
    dut.in_sof.value = 0


async def collect(dut, records: list) -> None:
    while True:
        await RisingEdge(dut.clk)
        if int(dut.o_valid.value):
            ex = tuple((int(dut.o_ex[k].value), int(dut.o_ey[k].value)) for k in range(16))
            records.append(
                (int(dut.o_id.value), bool(int(dut.o_white.value)), int(dut.o_area.value), int(dut.o_xmin.value),
                 int(dut.o_xmax.value), int(dut.o_ymin.value), int(dut.o_ymax.value),
                 int(dut.o_closed_after_row.value), ex)
            )
        if int(dut.o_frame_done.value):
            return


async def run_frame(dut, w: int, h: int, image: bytes) -> int:
    d = dump(w, h, image)
    want = expected(d["components"])
    got: list = []
    collector = cocotb.start_soon(collect(dut, got))
    await drive(dut, w, h, d["threshold"])
    await collector
    for n, (g, e) in enumerate(zip(got, want)):
        if g != e:
            names = FIELDS + ["extremes"]
            diffs = [f"{names[k]}: RTL {g[k]} vs golden {e[k]}" for k in range(len(names)) if g[k] != e[k]]
            raise AssertionError(f"{w}x{h}: record {n} (golden id {e[0]}) differs: " + "; ".join(diffs))
    assert len(got) == len(want), f"{w}x{h}: RTL emitted {len(got)} components, golden {len(want)}"
    assert int(dut.o_overflow.value) == 0
    return len(want)


@cocotb.test()
async def small_and_odd_sizes(dut):
    """Edge-column join quirks show up most at small and odd sizes."""
    await reset(dut)
    rng = random.Random(1)
    for w, h in [(4, 4), (5, 4), (4, 7), (7, 5), (8, 8), (17, 9), (33, 17), (64, 48)]:
        await run_frame(dut, w, h, noise(rng, w, h))
        await run_frame(dut, w, h, blocks(rng, w, h))


@cocotb.test()
async def tag_like_scenes(dut):
    await reset(dut)
    rng = random.Random(2)
    for w, h in [(96, 64), (160, 100), (97, 61)]:
        n = await run_frame(dut, w, h, gradient_with_squares(rng, w, h))
        dut._log.info(f"tag-like {w}x{h}: {n} components; longest row processing so far {int(dut.o_row_cycles_max.value)} cycles")


@cocotb.test()
async def noise_heavy(dut):
    """Noise thresholds into many tiny components and long merge chains."""
    await reset(dut)
    rng = random.Random(3)
    n = await run_frame(dut, 160, 120, noise(rng, 160, 120))
    dut._log.info(f"noise 160x120: {n} components; longest row processing {int(dut.o_row_cycles_max.value)} cycles")


def shapes(w: int, h: int) -> list[tuple[str, bytes]]:
    """Patterns that stress merging: a row's worth of joins, long merge chains,
    and labels that merge right before they close."""
    def img(f) -> bytes:
        return bytes(240 if f(x, y) else 10 for y in range(h) for x in range(w))
    cx, cy = w // 2, h // 2
    return [
        ("checkerboard", img(lambda x, y: (x + y) % 2)),
        ("columns", img(lambda x, y: x % 2)),
        ("rows", img(lambda x, y: y % 2)),
        ("comb, joined below", img(lambda x, y: x % 2 == 0 or y == h - 2)),
        ("comb, joined in pairs, then pairs of pairs", img(lambda x, y: x % 2 == 0 or (y % 4 == 3 and (x >> (1 + y // 4)) % 2 == 0))),
        ("nested Us", img(lambda x, y: max(abs(x - cx), y - 2) % 4 < 2)),
        ("nested Us, upside down", img(lambda x, y: max(abs(x - cx), h - y) % 4 < 2)),
        ("rings", img(lambda x, y: max(abs(x - cx), abs(y - cy)) % 2)),
        ("diagonals", img(lambda x, y: (x + y) % 4 < 2)),
        ("diagonals, other way", img(lambda x, y: (x - y) % 4 < 2)),
        ("staircase merges", img(lambda x, y: x % 3 == 0 or x // 3 == y)),
        ("staircase merges, leftward", img(lambda x, y: x % 3 == 0 or (w - x) // 3 == y)),
        ("diamonds", img(lambda x, y: (abs(x % 16 - 8) + abs(y % 16 - 8)) % 4 < 2)),
    ]


@cocotb.test()
async def adversarial_shapes(dut):
    await reset(dut)
    rng = random.Random(4)
    for w, h in [(64, 40), (101, 33)]:
        for name, image in shapes(w, h):
            dut._log.info(f"{name} {w}x{h}")
            await run_frame(dut, w, h, image)
            noisy = bytes(rng.randint(0, 255) if rng.random() < 0.02 else v for v in image)
            await run_frame(dut, w, h, noisy)


@cocotb.test()
async def wide_noise(dut):
    """The full 1280 px width: labels must recycle."""
    await reset(dut)
    rng = random.Random(5)
    n = await run_frame(dut, 1280, 48, noise(rng, 1280, 48))
    # Real-time at 1280 wide needs about 1,300 (120 FPS at the pixel clock).
    assert int(dut.o_row_cycles_max.value) <= 1250, "resolver slower than it was"
    dut._log.info(f"noise 1280x48: {n} components; longest row {int(dut.o_row_cycles_max.value)} cycles, "
                  f"labels max {int(dut.o_labels_max.value)}")


async def collect_last(dut, records: list, starts: int) -> None:
    """Records after the `starts`th o_frame_start, up to its o_frame_done."""
    seen = 0
    while True:
        await RisingEdge(dut.clk)
        if int(dut.o_frame_start.value):
            seen += 1
            records.clear()
        if int(dut.o_valid.value):
            ex = tuple((int(dut.o_ex[k].value), int(dut.o_ey[k].value)) for k in range(16))
            records.append(
                (int(dut.o_id.value), bool(int(dut.o_white.value)), int(dut.o_area.value), int(dut.o_xmin.value),
                 int(dut.o_xmax.value), int(dut.o_ymin.value), int(dut.o_ymax.value),
                 int(dut.o_closed_after_row.value), ex)
            )
        if int(dut.o_frame_done.value):
            assert seen == starts, f"o_frame_done of frame {seen}, expected only {starts} to finish"
            return


@cocotb.test()
async def restarts_mid_frame(dut):
    """Frames cut at any pixel (one or two in a row) leave nothing behind."""
    await reset(dut)
    rng = random.Random(6)
    for n in range(40):
        cuts = rng.randint(1, 2)
        w, h = rng.randint(6, 60), rng.randint(5, 20)
        image = noise(rng, w, h) if n % 2 else blocks(rng, w, h)
        d = dump(w, h, image)
        got: list = []
        collector = cocotb.start_soon(collect_last(dut, got, cuts + 1))
        for c in range(cuts):
            cw, ch = rng.randint(6, 60), rng.randint(5, 20)
            t = dump(cw, ch, noise(rng, cw, ch))["threshold"]
            stop = rng.randint(1, cw * ch - 1)
            if c == cuts - 1 and n % 3 == 0:
                # Cut inside a run that has already joined several above it.
                t = bytes(255 * (x % 2) for x in range(cw)) + b"\xff" * cw
                stop = cw + rng.randint(5, cw - 1)
            for i in range(stop):
                dut.in_valid.value, dut.in_sof.value = 1, int(i == 0)
                dut.i_width.value, dut.i_height.value, dut.in_pix.value = cw, ch, t[i]
                await RisingEdge(dut.clk)
                while not int(dut.in_ready.value):
                    await RisingEdge(dut.clk)
        await drive(dut, w, h, d["threshold"])
        await collector
        assert got == expected(d["components"]), f"case {n}: {w}x{h} after {cuts} cut frame(s)"
    # Cut frames leak no labels: 3 rows' worth is the most one frame can hold.
    assert int(dut.o_labels_max.value) <= 3 * 60
    assert int(dut.o_overflow.value) == 0
