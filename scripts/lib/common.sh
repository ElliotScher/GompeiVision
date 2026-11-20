# shellcheck shell=bash source-path=SCRIPTDIR
# Shared helpers for scripts/*.sh. Source it; don't execute it.
# Safe under `set -euo pipefail`. Don't end a pipeline with `grep -q`: it exits
# at the first match, the writer gets SIGPIPE, and pipefail reports failure.

GOMPEI_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export GOMPEI_ROOT

# The repo-local Python environment bootstrap.sh creates (cocotb, pytest,
# CMake). doctor.sh and the self-test put its bin/ first on PATH when it exists.
GOMPEI_VENV="$GOMPEI_ROOT/.venv"
export GOMPEI_VENV

# All development tracks, in the order doctor.sh reports them.
ALL_TRACKS=(hardware rpu apu robot ml)

# Colour only when writing to a terminal and NO_COLOR is unset.
if [[ -t 1 && -z "${NO_COLOR:-}" ]]; then
  C_RED=$'\e[31m' C_GREEN=$'\e[32m' C_YELLOW=$'\e[33m' C_DIM=$'\e[2m' C_BOLD=$'\e[1m' C_RESET=$'\e[0m'
else
  C_RED='' C_GREEN='' C_YELLOW='' C_DIM='' C_BOLD='' C_RESET=''
fi
export C_RED C_GREEN C_YELLOW C_DIM C_BOLD C_RESET

info() { printf '%s\n' "$*"; }
warn() { printf '%swarning:%s %s\n' "$C_YELLOW" "$C_RESET" "$*" >&2; }
err()  { printf '%serror:%s %s\n' "$C_RED" "$C_RESET" "$*" >&2; }
die()  { err "$*"; exit 1; }

have() { command -v "$1" >/dev/null 2>&1; }

# version_ge A B: true if dotted version A >= B. Missing parts count as 0,
# and any non-numeric suffix on a part ("1.0.1-rc1", "18git") is ignored.
version_ge() {
  local -a a b
  local i x y
  IFS=. read -ra a <<<"$1"
  IFS=. read -ra b <<<"$2"
  for ((i = 0; i < ${#a[@]} || i < ${#b[@]}; i++)); do
    x="${a[i]:-0}" y="${b[i]:-0}"
    x="${x%%[!0-9]*}" y="${y%%[!0-9]*}"
    x=$((10#${x:-0})) y=$((10#${y:-0}))
    ((x > y)) && return 0
    ((x < y)) && return 1
  done
  return 0
}

# extract_version TEXT: print the first N.N[.N...] in TEXT, or nothing.
extract_version() {
  [[ "$1" =~ ([0-9]+(\.[0-9]+)+) ]] && printf '%s\n' "${BASH_REMATCH[1]}"
  return 0
}

# confirm PROMPT: yes/no prompt. Always yes when ASSUME_YES=1 (--yes);
# always no when stdin isn't a terminal and --yes wasn't given.
confirm() {
  [[ "${ASSUME_YES:-0}" == 1 ]] && return 0
  [[ -t 0 ]] || return 1
  local reply
  read -rp "$1 [y/N] " reply
  [[ "$reply" == [yY] || "$reply" == [yY][eE][sS] ]]
}

# load_versions: source tools/versions.env into the environment.
load_versions() {
  local f="$GOMPEI_ROOT/tools/versions.env"
  [[ -f "$f" ]] || die "missing $f"
  set -a
  # shellcheck source=../../tools/versions.env
  . "$f"
  set +a
}

# parse_tracks ARGS...: set TRACKS from --hardware/--rpu/--apu/--robot/--ml/--all
# flags (all tracks if none given). Other arguments are left in REST_ARGS.
parse_tracks() {
  TRACKS=()
  REST_ARGS=()
  local a
  for a in "$@"; do
    case "$a" in
      --hardware | --rpu | --apu | --robot | --ml) TRACKS+=("${a#--}") ;;
      --all) TRACKS=("${ALL_TRACKS[@]}") ;;
      *) REST_ARGS+=("$a") ;;
    esac
  done
  ((${#TRACKS[@]})) || TRACKS=("${ALL_TRACKS[@]}")
  return 0
}

# want_track NAME: true if NAME is in TRACKS.
want_track() {
  local t
  for t in "${TRACKS[@]}"; do [[ "$t" == "$1" ]] && return 0; done
  return 1
}
