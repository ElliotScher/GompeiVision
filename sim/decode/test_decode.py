"""pytest entry: build rtl/decode/decode.sv (and its blocks) with the pixel
window and run decode_tests.py against `gv-model decodes` and `read`."""

import os
import subprocess
from pathlib import Path

from cocotb_tools.runner import get_results, get_runner

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def test_decode():
    gv_model = os.environ.get("GV_MODEL", str(ROOT / "build" / "model-tool" / "gv-model"))
    assert Path(gv_model).is_file(), f"{gv_model} not found: build model/ first (scripts/run-sims.sh does)"
    build = HERE / "sim_build_decode"
    tables = build / "lookup_tables"
    build.mkdir(exist_ok=True)
    subprocess.run([gv_model, "lookup-tables", str(tables)], check=True, capture_output=True)
    runner = get_runner("verilator")
    runner.build(
        sources=[ROOT / "rtl" / "decode" / f"{m}.sv" for m in ("projector", "tag_sampler", "id_lookup", "decode")] + [
            ROOT / "rtl" / "pixels" / "pixel_window.sv", HERE / "decode_tb.sv"],
        hdl_toplevel="decode_tb",
        build_dir=build,
        always=True,
        build_args=["-Wall"],
    )
    results = runner.test(
        hdl_toplevel="decode_tb",
        test_module="decode_tests",
        test_dir=HERE,
        build_dir=build,
        plusargs=[f"+LOOKUP_TABLES={tables}"],
        extra_env={"GV_MODEL": gv_model},
        results_xml=str(HERE / "test_decode.result.xml"),
    )
    total, failed = get_results(results)
    assert failed == 0, f"{failed} of {total} cocotb tests failed"
