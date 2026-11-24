"""cocotb test for rtl/tag_finish/tag_finish.sv: every code read and every edge
job of synthetic tag scenes, both streams at once on the shared pixel store,
must give exactly the golden results."""

import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # sim/, for gvsim

import cocotb  # noqa: E402
from cocotb.clock import Clock  # noqa: E402
from cocotb.triggers import Combine, RisingEdge, Timer  # noqa: E402
from gvsim import JOB_FIELDS, code_reads, edge_jobs, read_pgm, read_tags, synth_scene  # noqa: E402

SCENES = [(11, 256, 160, 2, (40, 90)), (100, 256, 160, 3, (24, 40)), (200, 480, 320, 2, (80, 150)),
          (300, 256, 160, 3, (40, 90))]


async def send(dut, prefix: str, n: int, values: dict) -> None:
    for k, v in values.items():
        getattr(dut, k).value = v
    getattr(dut, f"{prefix}_job_tag").value = n
    getattr(dut, f"{prefix}_job_valid").value = 1
    while True:
        await Timer(1, unit="ns")
        ready = int(getattr(dut, f"{prefix}_job_ready").value)
        await RisingEdge(dut.clk)
        if ready:
            break
    getattr(dut, f"{prefix}_job_valid").value = 0


@cocotb.test()
async def reads_and_edges_together(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value, dut.wr_en.value, dut.max_bits.value = 1, 0, 3
    dut.rd_job_valid.value = dut.ed_job_valid.value = 0
    for _ in range(3):
        await RisingEdge(dut.clk)
    dut.rst.value = 0
    total_reads = total_points = total_cycles = 0
    for s in SCENES:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "scene.pgm"
            synth_scene(path, *s)
            w, h, pixels = read_pgm(path)
            reads = [r["h"] for r in code_reads(path)]
            want_reads = read_tags(path, reads)
            # Repeated so that the edge stream outlasts the reads.
            jobs = edge_jobs(path) * 4
        assert reads and jobs
        dut.img_w.value, dut.img_h.value, dut.wr_en.value = w, h, 1
        for y in range(h):
            for x in range(w):
                dut.wr_x.value, dut.wr_y.value, dut.wr_data.value = x, y, pixels[y * w + x]
                await RisingEdge(dut.clk)
        dut.wr_en.value = 0
        got_reads: list = []
        points: list[list] = [[]]
        cycles = 0

        async def collect():
            nonlocal cycles
            while len(got_reads) < len(reads) or len(points) <= len(jobs):
                await RisingEdge(dut.clk)
                cycles += 1
                assert cycles < 400_000, f"{len(got_reads)} reads, {len(points) - 1} edge jobs"
                if int(dut.rd_valid.value):
                    assert int(dut.rd_tag.value) == len(got_reads)
                    found = int(dut.rd_found.value)
                    got_reads.append({
                        "ok": int(dut.rd_ok.value) == 1, "code": int(dut.rd_code.value),
                        "margin": dut.rd_margin.value.to_signed(),
                        "match": (int(dut.rd_id.value), int(dut.rd_rotation.value), int(dut.rd_hamming.value))
                        if found else None})
                if int(dut.pt_valid.value):
                    points[-1].append((dut.pt_x.value.to_signed(), dut.pt_y.value.to_signed()))
                if int(dut.ed_done.value):
                    assert int(dut.ed_tag.value) == len(points) - 1
                    points.append([])

        async def send_reads():
            for n, c in enumerate(reads):
                await send(dut, "rd", n, {"rd_job_h": c})

        async def send_edges():
            for n, j in enumerate(jobs):
                await send(dut, "ed", n, {k: j[k] for k in JOB_FIELDS})

        await Combine(cocotb.start_soon(collect()), cocotb.start_soon(send_reads()),
                      cocotb.start_soon(send_edges()))
        for n, (g, e) in enumerate(zip(got_reads, want_reads)):
            assert (g == e) if e["ok"] else not g["ok"], f"scene {s[0]}, read {n}: RTL {g}, golden {e}"
        for n, (g, j) in enumerate(zip(points, jobs)):
            assert g == j["points"], f"scene {s[0]}, edge job {n} differs"
        total_reads += len(reads)
        total_points += sum(map(len, points))
        total_cycles += cycles
    assert int(dut.o_error.value) == 0
    dut._log.info(f"{total_reads} reads and {total_points} edge points together, all bit-exact, in {total_cycles} "
                  "cycles")
