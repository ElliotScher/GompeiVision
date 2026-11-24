"""cocotb tests for rtl/threshold/threshold.sv against the golden model.

Every expected image comes from `gv-model dump` (model/tools/gv_model.cpp),
the same code the model tests check bit for bit against AprilTag.
"""

import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # sim/, for gvsim

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge
from gvsim import blocks, dump, gradient_with_squares, noise  # noqa: E402

H_BLANK = 4  # idle cycles after each input row (the block needs >= 3 on average)


def golden(w: int, h: int, pixels: bytes, diff: int) -> bytes:
    return dump(w, h, pixels, "--min-white-black-diff", str(diff))["threshold"]


# --- driving -----------------------------------------------------------------------


async def reset(dut) -> None:
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1
    dut.i_valid.value = 0
    dut.i_sof.value = 0
    dut.i_pix.value = 0
    dut.i_width.value = 4
    dut.i_height.value = 4
    dut.min_white_black_diff.value = 5
    for _ in range(3):
        await RisingEdge(dut.clk)
    dut.rst.value = 0
    await RisingEdge(dut.clk)


async def drive(dut, w: int, h: int, pixels: bytes, h_blank: int = H_BLANK) -> None:
    for y in range(h):
        for x in range(w):
            dut.i_valid.value = 1
            dut.i_sof.value = int(x == 0 and y == 0)
            dut.i_width.value = w
            dut.i_height.value = h
            dut.i_pix.value = pixels[y * w + x]
            await RisingEdge(dut.clk)
        dut.i_valid.value = 0
        dut.i_sof.value = 0
        for _ in range(h_blank):
            await RisingEdge(dut.clk)


async def collect(dut, w: int, h: int) -> bytes:
    """One frame of output, checking coordinates and the SOF/EOL markers."""
    out = bytearray(w * h)
    got = 0
    idle = 0
    while got < w * h:
        await RisingEdge(dut.clk)
        if int(dut.o_valid.value):
            idle = 0
            x, y = int(dut.o_x.value), int(dut.o_y.value)
            assert (x, y) == (got % w, got // w), f"pixel {got} came out as ({x}, {y})"
            assert int(dut.o_sof.value) == int(got == 0)
            assert int(dut.o_eol.value) == int(x == w - 1)
            out[got] = int(dut.o_pix.value)
            got += 1
        else:
            idle += 1
            assert idle < 50 * (w + H_BLANK) + 1000, f"output stalled after {got} of {w * h} pixels"
    return bytes(out)


async def run_frame(dut, w: int, h: int, pixels: bytes, diff: int = 5) -> None:
    dut.min_white_black_diff.value = diff
    want = golden(w, h, pixels, diff)
    collector = cocotb.start_soon(collect(dut, w, h))
    await drive(dut, w, h, pixels)
    got = await collector
    if got != want:
        bad = next(i for i in range(w * h) if got[i] != want[i])
        mism = sum(1 for i in range(w * h) if got[i] != want[i])
        raise AssertionError(
            f"{w}x{h}: {mism} pixels differ; first at ({bad % w}, {bad // w}): RTL {got[bad]}, golden {want[bad]}"
        )
    assert int(dut.o_overrun.value) == 0


# --- tests -------------------------------------------------------------------------


@cocotb.test()
async def odd_sizes(dut):
    """Sizes that aren't multiples of 4 exercise the partial-tile rule."""
    await reset(dut)
    rng = random.Random(1)
    for w, h in [(4, 4), (5, 4), (4, 7), (7, 5), (8, 8), (33, 17), (17, 33), (64, 48)]:
        await run_frame(dut, w, h, noise(rng, w, h))
        await run_frame(dut, w, h, blocks(rng, w, h))


@cocotb.test()
async def low_contrast_and_flat(dut):
    """The 127 ("skip") path, including the partial edge tiles never skipping."""
    await reset(dut)
    rng = random.Random(2)
    for w, h in [(10, 9), (40, 30), (37, 23)]:
        await run_frame(dut, w, h, bytes([100]) * (w * h))
        await run_frame(dut, w, h, noise(rng, w, h, 100, 107))


@cocotb.test()
async def min_white_black_diff_values(dut):
    await reset(dut)
    rng = random.Random(3)
    img = gradient_with_squares(rng, 96, 64)
    for diff in [0, 1, 5, 20, 60, 255]:
        await run_frame(dut, 96, 64, img, diff)


@cocotb.test()
async def tag_like_scene(dut):
    """A larger gradient-and-squares frame, then a second frame back to back."""
    await reset(dut)
    rng = random.Random(4)
    for w, h in [(320, 200), (161, 97)]:
        await run_frame(dut, w, h, gradient_with_squares(rng, w, h))


@cocotb.test()
async def full_size_frame(dut):
    """1280x800, the sensor's full frame: the 16-row ring wraps 50 times."""
    await reset(dut)
    rng = random.Random(5)
    await run_frame(dut, 1280, 800, blocks(rng, 1280, 800, 80))


@cocotb.test()
async def overrun_without_blanking(dut):
    """With no horizontal blanking the output (3 extra cycles per row) falls
    behind until the input would overwrite an unread row: o_overrun must rise."""
    await reset(dut)
    rng = random.Random(6)
    w, h = 64, 300
    cocotb.start_soon(collect(dut, w, h))
    await drive(dut, w, h, noise(rng, w, h), h_blank=0)
    assert int(dut.o_overrun.value) == 1, "overrun not flagged"
