"""pytest entry: build rtl/components/components.sv with Verilator and run the
cocotb tests in components_tests.py against `gv-model dump`."""

import os
from pathlib import Path

from cocotb_tools.runner import get_results, get_runner

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def test_components():
    gv_model = os.environ.get("GV_MODEL", str(ROOT / "build" / "model-tool" / "gv-model"))
    assert Path(gv_model).is_file(), f"{gv_model} not found: build model/ first (scripts/run-sims.sh does)"
    runner = get_runner("verilator")
    build = HERE / "sim_build"
    runner.build(
        sources=[ROOT / "rtl" / "components" / "components.sv"],
        hdl_toplevel="components",
        build_dir=build,
        always=True,
        build_args=["-Wall"],
    )
    results = runner.test(
        hdl_toplevel="components",
        test_module="components_tests",
        test_dir=HERE,
        build_dir=build,
        extra_env={"GV_MODEL": gv_model},
    )
    total, failed = get_results(results)
    assert failed == 0, f"{failed} of {total} cocotb tests failed"
