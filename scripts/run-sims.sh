#!/usr/bin/env bash
# Run the Verilator + cocotb simulations: the toolchain self-test, then every
# testbench under sim/ (files named test_*.py, collected by pytest).
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
info "== toolchain self-test"
python3 tools/selftest/run.py

if [[ -z "$(find sim -name 'test_*.py' -print -quit)" ]]; then
  info "== sim/: no testbenches yet"
  exit 0
fi
info "== sim/ testbenches"
python3 -m pytest sim "$@"
