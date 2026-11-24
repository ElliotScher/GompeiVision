"""pytest entry: build rtl/tag_finish/tag_finish.sv (decode, edge sampler and
pixel window) and run tag_finish_tests.py against the golden model."""

import os
import subprocess
from pathlib import Path

from cocotb_tools.runner import get_results, get_runner

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def test_tag_finish():
    gv_model = os.environ.get("GV_MODEL", str(ROOT / "build" / "model-tool" / "gv-model"))
    assert Path(gv_model).is_file(), f"{gv_model} not found: build model/ first (scripts/run-sims.sh does)"
    build = HERE / "sim_build"
    tables = build / "lookup_tables"
    build.mkdir(exist_ok=True)
    subprocess.run([gv_model, "lookup-tables", str(tables)], check=True, capture_output=True)
    runner = get_runner("verilator")
    runner.build(
        sources=[ROOT / "rtl" / f for f in (
            "decode/projector.sv", "decode/tag_sampler.sv", "decode/id_lookup.sv", "decode/decode.sv",
            "refine/edge_sampler.sv", "pixels/pixel_window.sv", "tag_finish/tag_finish.sv")],
        hdl_toplevel="tag_finish",
        build_dir=build,
        always=True,
        build_args=["-Wall"],
    )
    results = runner.test(
        hdl_toplevel="tag_finish",
        test_module="tag_finish_tests",
        test_dir=HERE,
        build_dir=build,
        plusargs=[f"+LOOKUP_TABLES={tables}"],
        extra_env={"GV_MODEL": gv_model},
        results_xml=str(HERE / "test_tag_finish.result.xml"),
    )
    total, failed = get_results(results)
    assert failed == 0, f"{failed} of {total} cocotb tests failed"
