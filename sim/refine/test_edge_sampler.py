"""pytest entry: build rtl/refine/edge_sampler.sv with Verilator and run the
cocotb tests in edge_sampler_tests.py against `gv-model edges`."""

import os
from pathlib import Path

from cocotb_tools.runner import get_results, get_runner

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def test_edge_sampler():
    gv_model = os.environ.get("GV_MODEL", str(ROOT / "build" / "model-tool" / "gv-model"))
    assert Path(gv_model).is_file(), f"{gv_model} not found: build model/ first (scripts/run-sims.sh does)"
    runner = get_runner("verilator")
    build = HERE / "sim_build"
    runner.build(
        sources=[ROOT / "rtl" / "refine" / "edge_sampler.sv"],
        hdl_toplevel="edge_sampler",
        build_dir=build,
        always=True,
        build_args=["-Wall"],
    )
    results = runner.test(
        hdl_toplevel="edge_sampler",
        test_module="edge_sampler_tests",
        test_dir=HERE,
        build_dir=build,
        extra_env={"GV_MODEL": gv_model},
    )
    total, failed = get_results(results)
    assert failed == 0, f"{failed} of {total} cocotb tests failed"
