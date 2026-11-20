"""Build counter.sv with Verilator and run the cocotb tests in test_counter.py.

Exits non-zero if the build or any test fails. Build output goes to
tools/selftest/sim_build/ (git-ignored).
"""

import sys
from pathlib import Path

from cocotb_tools.runner import get_runner

HERE = Path(__file__).resolve().parent


def main() -> int:
    runner = get_runner("verilator")
    runner.build(
        sources=[HERE / "counter.sv"],
        hdl_toplevel="counter",
        build_dir=HERE / "sim_build",
        always=True,
        build_args=["-Wall"],
    )
    results = runner.test(
        hdl_toplevel="counter",
        test_module="test_counter",
        test_dir=HERE,
        build_dir=HERE / "sim_build",
    )
    # runner.test returns the path to a JUnit results.xml; failures are
    # reported there rather than raised, so parse it.
    from cocotb_tools.runner import get_results

    num_tests, num_failed = get_results(results)
    print(f"selftest: {num_tests - num_failed}/{num_tests} cocotb tests passed")
    return 1 if num_failed else 0


if __name__ == "__main__":
    sys.exit(main())
