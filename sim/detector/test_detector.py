"""pytest entry: build the whole PL detector (rtl/detector/detector.sv) and run
detector_tests.py, with the RPU's code (rpu/, built for the host) in the loop."""

import os
import subprocess
from pathlib import Path

from cocotb_tools.runner import get_results, get_runner

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def test_detector():
    gv_model = os.environ.get("GV_MODEL", str(ROOT / "build" / "model-tool" / "gv-model"))
    assert Path(gv_model).is_file(), f"{gv_model} not found: build model/ first (scripts/run-sims.sh does)"
    rpu_lib = os.environ.get("GV_RPU_HOST", str(ROOT / "build" / "rpu-host" / "libgv_rpu_host.so"))
    assert Path(rpu_lib).is_file(), f"{rpu_lib} not found: build rpu/ first (scripts/run-sims.sh does)"
    build = HERE / "sim_build"
    tables = build / "lookup_tables"
    build.mkdir(exist_ok=True)
    subprocess.run([gv_model, "lookup-tables", str(tables)], check=True, capture_output=True)
    runner = get_runner("verilator")
    runner.build(
        sources=[ROOT / "rtl" / f for f in (
            "decode/projector.sv", "decode/tag_sampler.sv", "decode/id_lookup.sv", "decode/decode.sv",
            "refine/edge_sampler.sv", "pixels/pixel_window.sv", "tag_finish/tag_finish.sv",
            "threshold/threshold.sv", "components/components.sv", "quad/scorecard.sv",
            "quad_finder/quad_finder.sv", "detector/detector.sv")],
        hdl_toplevel="detector",
        build_dir=build,
        always=True,
        build_args=["-Wall"],
    )
    results = runner.test(
        hdl_toplevel="detector",
        test_module="detector_tests",
        test_dir=HERE,
        build_dir=build,
        plusargs=[f"+LOOKUP_TABLES={tables}"],
        extra_env={"GV_MODEL": gv_model, "GV_RPU_HOST": rpu_lib},
        results_xml=str(HERE / "test_detector.result.xml"),
    )
    total, failed = get_results(results)
    assert failed == 0, f"{failed} of {total} cocotb tests failed"
