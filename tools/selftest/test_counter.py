"""cocotb testbench for counter.sv (toolchain self-test).

Run through scripts/doctor.sh --selftest, or directly with:
    python tools/selftest/run.py
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, ReadOnly


async def reset(dut):
    dut.rst.value = 1
    dut.en.value = 0
    await ClockCycles(dut.clk, 2)
    dut.rst.value = 0


@cocotb.test()
async def counts_when_enabled(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset(dut)

    dut.en.value = 1
    await ClockCycles(dut.clk, 10)
    await ReadOnly()
    assert dut.count.value == 10, f"expected 10, got {int(dut.count.value)}"


@cocotb.test()
async def holds_when_disabled(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset(dut)

    dut.en.value = 1
    await ClockCycles(dut.clk, 3)
    dut.en.value = 0
    await ClockCycles(dut.clk, 5)
    await ReadOnly()
    assert dut.count.value == 3, f"expected 3, got {int(dut.count.value)}"


@cocotb.test()
async def wraps_around(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset(dut)

    dut.en.value = 1
    await ClockCycles(dut.clk, 256)
    await ReadOnly()
    assert dut.count.value == 0, f"expected wrap to 0, got {int(dut.count.value)}"

    await RisingEdge(dut.clk)
    await ReadOnly()
    assert dut.count.value == 1
