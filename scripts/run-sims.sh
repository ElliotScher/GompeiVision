#!/usr/bin/env bash
# Run the Verilator + cocotb simulations: the toolchain self-test, then every
# testbench under sim/ (files named test_*.py, collected by pytest). Builds
# the golden model's gv-model tool first (the testbenches compare against it)
# unless GV_MODEL points at one.
#
#   scripts/run-sims.sh [PYTEST_ARGS...]
#
# Extra arguments go to pytest, e.g. -k framebus or -x. Uses the repo's .venv
# (from bootstrap.sh) when it exists. Exits non-zero if anything fails.
set -euo pipefail

# shellcheck source-path=SCRIPTDIR source=lib/common.sh
. "$(dirname "${BASH_SOURCE[0]}")/lib/common.sh"

case "${1:-}" in -h | --help) sed -n '2,8s/^# \{0,1\}//p' "${BASH_SOURCE[0]}"; exit 0 ;; esac

[[ -x "$GOMPEI_VENV/bin/python" ]] && PATH="$GOMPEI_VENV/bin:$PATH"
[[ -d "$HOME/.local/bin" ]] && PATH="$PATH:$HOME/.local/bin"
have verilator || die "verilator not found (run scripts/bootstrap.sh --hardware)"

cd "$GOMPEI_ROOT"

# The testbenches compare against the golden model's dump tool.
if [[ -z "${GV_MODEL:-}" ]]; then
  have cmake || die "cmake not found (run scripts/bootstrap.sh --rpu or --apu)"
  info "== building the golden model's gv-model tool"
  # Its own build directory, without the model's tests (no GoogleTest).
  [[ -f build/model-tool/CMakeCache.txt ]] ||
    cmake -S model -B build/model-tool -DCMAKE_BUILD_TYPE=Release -DGV_MODEL_TESTS=OFF >/dev/null
  cmake --build build/model-tool --target gv-model --parallel >/dev/null
  export GV_MODEL="$GOMPEI_ROOT/build/model-tool/gv-model"
fi

# The detector testbench runs the RPU's code, built for the host.
if [[ -z "${GV_RPU_HOST:-}" ]]; then
  [[ -f build/rpu-host/CMakeCache.txt ]] ||
    cmake -S rpu -B build/rpu-host -DCMAKE_BUILD_TYPE=Release -DGV_RPU_TESTS=OFF >/dev/null
  cmake --build build/rpu-host --target gv_rpu_host --parallel >/dev/null
  export GV_RPU_HOST="$GOMPEI_ROOT/build/rpu-host/libgv_rpu_host.so"
fi

info "== toolchain self-test"
python3 tools/selftest/run.py

if [[ -z "$(find sim -name 'test_*.py' -print -quit)" ]]; then
  info "== sim/: no testbenches yet"
  exit 0
fi
info "== sim/ testbenches"
python3 -m pytest sim "$@"
