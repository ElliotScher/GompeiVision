"""pytest entry: build rtl/quad_finder/quad_finder.sv (and the blocks in it)
with Verilator and run quad_finder_tests.py against the golden model. One
build has room for everything (no drops allowed); the others are short of one
resource each, for the overload tests."""

import os
from pathlib import Path

import pytest
from cocotb_tools.runner import get_results, get_runner

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
RTL = ROOT / "rtl"

CONFIGS = {
    "full": {},
    "tiny_pix": {"MAX_W": 640, "PIX_DEPTH": 1024, "REC_DEPTH": 64},
    "tiny_rec": {"MAX_W": 640, "PIX_DEPTH": 262144, "REC_DEPTH": 8},
    "few_labels": {"MAX_W": 640, "PIX_DEPTH": 262144, "REC_DEPTH": 64, "MAX_LABELS": 64},
}


@pytest.mark.parametrize("config", CONFIGS)
def test_quad_finder(config):
    gv_model = os.environ.get("GV_MODEL", str(ROOT / "build" / "model-tool" / "gv-model"))
    assert Path(gv_model).is_file(), f"{gv_model} not found: build model/ first (scripts/run-sims.sh does)"
    build = HERE / f"sim_build_{config}"
    runner = get_runner("verilator")
    runner.build(
        sources=[RTL / "threshold" / "threshold.sv", RTL / "components" / "components.sv",
                 RTL / "quad" / "scorecard.sv", RTL / "quad_finder" / "quad_finder.sv"],
        hdl_toplevel="quad_finder",
        build_dir=build,
        parameters=CONFIGS[config],
        always=True,
        build_args=["-Wall"],
    )
    results = runner.test(
        hdl_toplevel="quad_finder",
        test_module="quad_finder_tests",
        test_dir=HERE,
        build_dir=build,
        extra_env={"GV_MODEL": gv_model, "QF_CONFIG": config},
        results_xml=str(HERE / f"test_quad_finder_{config}.result.xml"),
    )
    total, failed = get_results(results)
    assert total > 0 and failed == 0, f"{failed} of {total} cocotb tests failed"
