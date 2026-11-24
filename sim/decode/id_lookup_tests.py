"""cocotb tests for rtl/decode/id_lookup.sv against the golden model's
Tag36h11Lookup::decode() (`gv-model lookup`), using the ROMs from
`gv-model lookup-tables`."""

import os
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # sim/, for gvsim

import cocotb  # noqa: E402
from cocotb.clock import Clock  # noqa: E402
from cocotb.triggers import RisingEdge  # noqa: E402
from gvsim import lookup  # noqa: E402

TABLES = Path(os.environ["LOOKUP_TABLES"])


def table(name: str) -> list[int]:
    return [int(line, 16) for line in (TABLES / name).read_text().split()]


CODES = table("codes.mem")
NEAR = table("near.mem")  # NEAR[k]: cells touching cell k; cell k is bit 35 - k


def touching_triple(rng: random.Random) -> int:
    """Three wrong cells that form one 8-connected group."""
    while True:
        k = rng.randrange(36)
        n1 = [35 - j for j in range(36) if NEAR[k] >> (35 - j) & 1]
        if not n1:
            continue
        a = rng.choice(n1)
        group = (1 << (35 - k)) | (1 << a)
        # A third cell touching either of the two.
        cand = [35 - j for j in range(36) if (NEAR[k] | NEAR[35 - a]) >> (35 - j) & 1 and not group >> (35 - j) & 1]
        if cand:
            return group | (1 << rng.choice(cand))


def flips(rng: random.Random, n: int) -> int:
    m = 0
    while bin(m).count("1") < n:
        m |= 1 << rng.randrange(36)
    return m


async def reset(dut) -> None:
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1
    dut.in_valid.value = 0
    for _ in range(3):
        await RisingEdge(dut.clk)
    dut.rst.value = 0
    await RisingEdge(dut.clk)


async def rtl_lookup(dut, code: int, max_bits: int) -> tuple[tuple | None, int]:
    dut.in_code.value = code
    dut.max_bits.value = max_bits
    dut.in_valid.value = 1
    await RisingEdge(dut.clk)
    dut.in_valid.value = 0
    cycles = 0
    while True:
        await RisingEdge(dut.clk)
        cycles += 1
        if int(dut.out_valid.value):
            if not int(dut.out_found.value):
                return None, cycles
            return (int(dut.out_id.value), int(dut.out_rotation.value), int(dut.out_hamming.value)), cycles
        assert cycles < 1000


async def check(dut, reads: list[int], max_bits: int) -> int:
    want = lookup(reads, max_bits)
    worst = 0
    for code, w in zip(reads, want):
        got, cycles = await rtl_lookup(dut, code, max_bits)
        worst = max(worst, cycles)
        assert got == w, f"read {code:09x} (max_bits {max_bits}): RTL {got}, golden {w}"
    return worst


@cocotb.test()
async def corrections(dut):
    await reset(dut)
    rng = random.Random(1)
    reads = []
    for _ in range(150):
        c = CODES[rng.randrange(len(CODES))]
        reads += [c, c ^ flips(rng, 1), c ^ flips(rng, 2), c ^ flips(rng, 3), c ^ touching_triple(rng),
                  c ^ flips(rng, 4), c ^ flips(rng, 5 + rng.randrange(6))]
    reads += [rng.getrandbits(36) for _ in range(300)]
    worst = 0
    for max_bits in [3, 2, 1, 0]:
        worst = max(worst, await check(dut, reads, max_bits))
    found3 = sum(1 for w in lookup(reads, 3) if w is not None)
    dut._log.info(f"{len(reads)} reads x 4 max_bits settings, all match ({found3} decode at max_bits 3); "
                  f"worst {worst} cycles per lookup")


@cocotb.test()
async def every_code_every_rotation(dut):
    """All 2,348 rotated codes decode exactly."""
    await reset(dut)
    await check(dut, CODES, 3)
