#!/usr/bin/env bash
# Check the dev toolchain against tools/versions.env.
#
#   scripts/doctor.sh [--quiet] [--selftest] [TRACKS...]
#
# TRACKS are --hardware, --rpu, --apu, --robot, --ml or --all (the default).
# Prints tool / required / found / status and exits non-zero if a required
# tool in those tracks is missing or too old. Optional tools never fail the
# run. --quiet prints only the problems. --selftest also builds and runs the
# Verilator + cocotb counter test in tools/selftest/ (and checks the hardware
# track). Uses the repo's .venv (from bootstrap.sh) when it exists.
set -euo pipefail

# shellcheck source-path=SCRIPTDIR source=lib/common.sh
. "$(dirname "${BASH_SOURCE[0]}")/lib/common.sh"

QUIET=0 SELFTEST=0
parse_tracks "$@"
for arg in ${REST_ARGS[@]+"${REST_ARGS[@]}"}; do
  case "$arg" in
    --quiet | -q) QUIET=1 ;;
    --selftest) SELFTEST=1 ;;
    -h | --help) sed -n '2,11s/^# \{0,1\}//p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) die "unknown argument: $arg (see --help)" ;;
  esac
done
((!SELFTEST)) || want_track hardware || TRACKS+=(hardware)

load_versions
[[ -x "$GOMPEI_VENV/bin/python" ]] && PATH="$GOMPEI_VENV/bin:$PATH"
[[ -d "$HOME/.local/bin" ]] && PATH="$PATH:$HOME/.local/bin"

# Each row: section|tool|required|found|status|note. Status is one of
#   OK, MISSING, OLD, WRONG (fail the run), WARN (reported, doesn't fail).
ROWS=()
FAILED=0
SECTION=""
row() {
  ROWS+=("$SECTION|$1|$2|$3|$4|${5:-}")
  [[ "$4" == MISSING || "$4" == OLD || "$4" == WRONG ]] && FAILED=1
  return 0
}

# check_min NAME MIN FOUND [optional [NOTE]]: OK / OLD / MISSING row.
check_min() {
  local name="$1" min="$2" found="$3" optional="${4:-}" note="${5:-}"
  if [[ -z "$found" ]]; then
    row "$name" ">= $min" "-" "$([[ -n "$optional" ]] && echo WARN || echo MISSING)" "$note"
  elif version_ge "$found" "$min"; then
    row "$name" ">= $min" "$found" OK
  else
    row "$name" ">= $min" "$found" "$([[ -n "$optional" ]] && echo WARN || echo OLD)" "$note"
  fi
}

# check_exact NAME WANT FOUND [optional [NOTE]]: OK / WRONG / MISSING row.
check_exact() {
  local name="$1" want="$2" found="$3" optional="${4:-}" note="${5:-}"
  if [[ "$found" == "$want" ]]; then
    row "$name" "$want" "$found" OK
  elif [[ -n "$optional" ]]; then
    row "$name" "$want" "${found:--}" WARN "$note"
  else
    row "$name" "$want" "${found:--}" "$([[ -z "$found" ]] && echo MISSING || echo WRONG)" "$note"
  fi
}

# tool_version CMD [ARGS...]: first version number in the command's output.
tool_version() {
  have "$1" || return 0
  extract_version "$("$@" 2>&1 | head -n 5 || true)"
}

# py_version MODULE: the module's __version__, else its package metadata.
py_version() {
  have python3 || return 0
  python3 - "$1" 2>/dev/null <<'PY' || true
import importlib, importlib.metadata, sys
name = sys.argv[1]
print(getattr(importlib.import_module(name), "__version__", None) or importlib.metadata.version(name))
PY
}

# --- Common -----------------------------------------------------------------
SECTION=common
check_min git 2.34 "$(tool_version git --version)"
check_min python3 "$PYTHON_MIN" "$(tool_version python3 --version)"
check_min make 4.0 "$(tool_version make --version)"
check_min "g++ (host)" 13 "$(have g++ && g++ -dumpfullversion 2>/dev/null || true)"

# --- Hardware track ---------------------------------------------------------
if want_track hardware; then
  SECTION=hardware
  check_min verilator "$VERILATOR_VERSION" "$(tool_version verilator --version)" "" "apt's verilator is too old; run scripts/bootstrap.sh --hardware"
  # Verible tags look like v0.0-<build>-g<hash>; compare the build number.
  want_build="${VERIBLE_VERSION#v0.0-}" want_build="${want_build%%-*}"
  found_build=""
  if have verible-verilog-lint; then
    [[ "$(verible-verilog-lint --version 2>&1 || true)" =~ v0\.0-([0-9]+) ]] && found_build="${BASH_REMATCH[1]}"
  fi
  check_min "verible (build)" "$want_build" "$found_build"
  check_min cocotb "$COCOTB_VERSION" "$(py_version cocotb)"
  check_min pytest "$PYTEST_VERSION" "$(py_version pytest)"
  # gtkwave --version needs a display (it fails over SSH, in WSL without
  # WSLg, or in CI), so only check that it's installed.
  if have gtkwave; then row gtkwave "any" "installed" OK; else row gtkwave "any" "-" WARN "optional waveform viewer"; fi
  check_exact vivado "$AMD_TOOLS_VERSION" \
    "$(have vivado && extract_version "$(vivado -version 2>/dev/null | head -n 1 || true)" || true)" \
    optional "optional: bitstreams build on the shared build machine"
fi

# --- RPU track --------------------------------------------------------------
if want_track rpu; then
  SECTION=rpu
  check_min arm-none-eabi-gcc "$ARM_NONE_EABI_GCC_MIN" "$(have arm-none-eabi-gcc && arm-none-eabi-gcc -dumpfullversion || true)"
  check_min cmake "$CMAKE_MIN" "$(tool_version cmake --version)"
  check_exact vitis "$AMD_TOOLS_VERSION" \
    "$(have vitis && extract_version "$(vitis -v 2>/dev/null | head -n 5 || true)" || true)" \
    optional "optional until the RPU firmware needs the AMD BSP (Stage 2)"
fi

# --- APU track --------------------------------------------------------------
if want_track apu; then
  SECTION=apu
  check_min aarch64-linux-gnu-g++ "$AARCH64_GCC_MIN" "$(have aarch64-linux-gnu-g++ && aarch64-linux-gnu-g++ -dumpfullversion || true)"
  check_min cmake "$CMAKE_MIN" "$(tool_version cmake --version)"
  check_min ninja "$NINJA_MIN" "$(tool_version ninja --version)"
  check_min "docker (legacy pipeline)" "$DOCKER_MIN" "$(tool_version docker --version)" optional \
    "the legacy CPU pipeline builds in Dockerfile.X86 / Dockerfile.ARM"
fi

# --- Robot track ------------------------------------------------------------
if want_track robot; then
  SECTION=robot
  java_major=""
  if have java; then
    java_major="$(java -XshowSettings:properties -version 2>&1 | sed -n 's/^ *java\.specification\.version = //p' || true)"
  fi
  check_exact "java (JDK)" "$JDK_VERSION" "$java_major" "" "install openjdk-$JDK_VERSION-jdk-headless, or select it with update-alternatives"
  javac_major=""
  have javac && [[ "$(javac -version 2>&1 || true)" =~ ([0-9]+) ]] && javac_major="${BASH_REMATCH[1]}"
  check_exact javac "$JDK_VERSION" "$javac_major"
fi

# --- ML track ---------------------------------------------------------------
if want_track ml; then
  SECTION=ml
  check_min docker "$DOCKER_MIN" "$(tool_version docker --version)"
  daemon=""
  have docker && daemon="$(docker info --format '{{.ServerVersion}}' 2>/dev/null || true)"
  check_min "docker daemon" "$DOCKER_MIN" "$daemon" "" "start Docker, and make sure you're in the docker group"
  image=""
  [[ -n "$daemon" ]] && docker image inspect "$VITIS_AI_DOCKER_IMAGE" >/dev/null 2>&1 && image="$VITIS_AI_VERSION"
  check_exact "Vitis AI image" "$VITIS_AI_VERSION" "$image" optional "docker pull $VITIS_AI_DOCKER_IMAGE"
fi

# --- Self-test --------------------------------------------------------------
if ((SELFTEST)); then
  SECTION=selftest
  log="$GOMPEI_ROOT/tools/selftest/sim_build/selftest.log"
  mkdir -p "$(dirname "$log")"
  if have python3 && python3 "$GOMPEI_ROOT/tools/selftest/run.py" >"$log" 2>&1; then
    result="$(grep -oE '[0-9]+/[0-9]+ cocotb tests passed' "$log" || true)"
    row "counter test" "pass" "${result%% *}" OK
  else
    row "counter test" "pass" "fail" MISSING "log: ${log#"$GOMPEI_ROOT"/}"
  fi
fi

# --- Report -----------------------------------------------------------------
colour() {
  case "$1" in
    OK) printf '%s' "$C_GREEN" ;;
    MISSING | OLD | WRONG) printf '%s' "$C_RED" ;;
    WARN) printf '%s' "$C_YELLOW" ;;
    *) printf '%s' "$C_DIM" ;;
  esac
}

w0=7 w1=4 w2=8 w3=5
for r in "${ROWS[@]}"; do
  IFS='|' read -r s t req found _ _ <<<"$r"
  ((${#s} > w0)) && w0=${#s}
  ((${#t} > w1)) && w1=${#t}
  ((${#req} > w2)) && w2=${#req}
  ((${#found} > w3)) && w3=${#found}
done

printed=0
for r in "${ROWS[@]}"; do
  IFS='|' read -r s t req found st note <<<"$r"
  if ((QUIET)) && [[ "$st" == OK ]]; then continue; fi
  if ((!printed)); then
    printf "%-${w0}s  %-${w1}s  %-${w2}s  %-${w3}s  %s\n" Track Tool Required Found Status
    printed=1
  fi
  printf "%-${w0}s  %-${w1}s  %-${w2}s  %-${w3}s  %s%-7s%s %s\n" "$s" "$t" "$req" "$found" \
    "$(colour "$st")" "$st" "$C_RESET" "${note:+${C_DIM}${note}${C_RESET}}"
done

if ((SELFTEST)) && ((FAILED)) && [[ -f "${log:-}" ]] && ! grep -q 'cocotb tests passed' "$log"; then
  info ""
  info "Self-test log (last 15 lines):"
  tail -n 15 "$log" | sed 's/^/  /'
fi

if ((FAILED)); then
  err "required tools are missing or too old (pins: tools/versions.env; install: scripts/bootstrap.sh; help: docs/dev-setup.md)"
  exit 1
fi
((QUIET)) || info "${C_GREEN}All required tools found${C_RESET} (tracks: ${TRACKS[*]})."
