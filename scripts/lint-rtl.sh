#!/usr/bin/env bash
# Lint and format-check the SystemVerilog in rtl/, sim/ and tools/selftest/.
#
#   scripts/lint-rtl.sh [--fix]
#
# Runs verible-verilog-lint and verible-verilog-format --verify on every .sv
# and .svh file. --fix rewrites the files with verible-verilog-format instead
# of failing on formatting. Exits non-zero on any lint or format problem.
set -euo pipefail

# shellcheck source-path=SCRIPTDIR source=lib/common.sh
. "$(dirname "${BASH_SOURCE[0]}")/lib/common.sh"

FIX=0
for arg in "$@"; do
  case "$arg" in
    --fix) FIX=1 ;;
    -h | --help) sed -n '2,8s/^# \{0,1\}//p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) die "unknown argument: $arg (see --help)" ;;
  esac
done

[[ -d "$HOME/.local/bin" ]] && PATH="$PATH:$HOME/.local/bin"
have verible-verilog-lint || die "verible-verilog-lint not found (run scripts/bootstrap.sh --hardware)"

cd "$GOMPEI_ROOT"
mapfile -t files < <(find rtl sim tools/selftest -type f \( -name '*.sv' -o -name '*.svh' \) \
  -not -path '*/sim_build/*' 2>/dev/null | sort)
((${#files[@]})) || { info "lint-rtl: no SystemVerilog files"; exit 0; }

status=0
verible-verilog-lint "${files[@]}" || status=1
if ((FIX)); then
  verible-verilog-format --inplace "${files[@]}"
elif ! verible-verilog-format --verify "${files[@]}" >/dev/null 2>&1; then
  err "formatting differs from verible-verilog-format in:"
  for f in "${files[@]}"; do
    verible-verilog-format --verify "$f" >/dev/null 2>&1 || printf '  %s\n' "$f" >&2
  done
  err "fix with: scripts/lint-rtl.sh --fix"
  status=1
fi
((status)) || info "lint-rtl: ${#files[@]} file(s) clean"
exit "$status"
