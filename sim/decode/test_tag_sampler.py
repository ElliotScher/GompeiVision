"""pytest entry: build rtl/decode/tag_sampler.sv with Verilator and run
tag_sampler_tests.py against `gv-model decodes`."""

import os
import subprocess
from pathlib import Path

from cocotb_tools.runner import get_results, get_runner

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def test_tag_sampler():
    gv_model = os.environ.get("GV_MODEL", str(ROOT / "build" / "model-tool" / "gv-model"))
    assert Path(gv_model).is_file(), f"{gv_model} not found: build model/ first (scripts/run-sims.sh does)"
    build = HERE / "sim_build_tag_sampler"
    tables = build / "lookup_tables"
    build.mkdir(exist_ok=True)
    subprocess.run([gv_model, "lookup-tables", str(tables)], check=True, capture_output=True)
    runner = get_runner("verilator")
    runner.build(
        sources=[ROOT / "rtl" / "decode" / "tag_sampler.sv"],
        hdl_toplevel="tag_sampler",
        build_dir=build,
        always=True,
        build_args=["-Wall"],
    )
    results = runner.test(
        hdl_toplevel="tag_sampler",
        test_module="tag_sampler_tests",
        test_dir=HERE,
        build_dir=build,
        plusargs=[f"+LOOKUP_TABLES={tables}"],
        extra_env={"GV_MODEL": gv_model},
        results_xml=str(HERE / "test_tag_sampler.result.xml"),
    )
    total, failed = get_results(results)
    assert failed == 0, f"{failed} of {total} cocotb tests failed"
