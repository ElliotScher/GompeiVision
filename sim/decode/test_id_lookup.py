"""pytest entry: build rtl/decode/id_lookup.sv with Verilator, generate its
ROMs with `gv-model lookup-tables`, and run id_lookup_tests.py against
`gv-model lookup`."""

import os
import subprocess
from pathlib import Path

from cocotb_tools.runner import get_results, get_runner

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def test_id_lookup():
    gv_model = os.environ.get("GV_MODEL", str(ROOT / "build" / "model-tool" / "gv-model"))
    assert Path(gv_model).is_file(), f"{gv_model} not found: build model/ first (scripts/run-sims.sh does)"
    build = HERE / "sim_build"
    tables = build / "lookup_tables"
    build.mkdir(exist_ok=True)
    subprocess.run([gv_model, "lookup-tables", str(tables)], check=True, capture_output=True)
    runner = get_runner("verilator")
    runner.build(
        sources=[ROOT / "rtl" / "decode" / "id_lookup.sv"],
        hdl_toplevel="id_lookup",
        build_dir=build,
        always=True,
        build_args=["-Wall"],
    )
    results = runner.test(
        hdl_toplevel="id_lookup",
        test_module="id_lookup_tests",
        test_dir=HERE,
        build_dir=build,
        plusargs=[f"+LOOKUP_TABLES={tables}"],
        extra_env={"GV_MODEL": gv_model, "LOOKUP_TABLES": str(tables)},
    )
    total, failed = get_results(results)
    assert failed == 0, f"{failed} of {total} cocotb tests failed"
