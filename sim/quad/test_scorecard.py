"""pytest entry: build rtl/quad/scorecard.sv with Verilator and run
scorecard_tests.py against `gv-model quads`."""

import os
from pathlib import Path

from cocotb_tools.runner import get_results, get_runner

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def test_scorecard():
    gv_model = os.environ.get("GV_MODEL", str(ROOT / "build" / "model-tool" / "gv-model"))
    assert Path(gv_model).is_file(), f"{gv_model} not found: build model/ first (scripts/run-sims.sh does)"
    build = HERE / "sim_build"
    runner = get_runner("verilator")
    runner.build(
        sources=[ROOT / "rtl" / "quad" / "scorecard.sv"],
        hdl_toplevel="scorecard",
        build_dir=build,
        always=True,
        build_args=["-Wall"],
    )
    results = runner.test(
        hdl_toplevel="scorecard",
        test_module="scorecard_tests",
        test_dir=HERE,
        build_dir=build,
        extra_env={"GV_MODEL": gv_model},
        results_xml=str(HERE / "test_scorecard.result.xml"),
    )
    total, failed = get_results(results)
    assert failed == 0, f"{failed} of {total} cocotb tests failed"
