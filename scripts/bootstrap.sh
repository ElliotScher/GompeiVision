#!/usr/bin/env bash
# Install GompeiVision's development tools on Ubuntu 24.04+ or Debian 13+,
# on bare metal or in WSL2 on Windows.
#
#   scripts/bootstrap.sh [--yes] [--dry-run] [TRACKS...]
#
#   --hardware  Verilator, Verible, cocotb, GTKWave (RTL and simulation)
#   --rpu       ARM bare-metal GCC (Cortex-R5F firmware)
#   --apu       AArch64 Linux cross-compiler, CMake, Ninja (module Linux apps)
#   --robot     JDK for the SystemCore robot library
#   --ml        Docker and the Vitis AI image (object detection)
#   --all       every track (the default when none is given)
#   --yes       don't ask before installing
#   --dry-run   print the plan and exit
#
# Prints what it will do and asks first. Steps that are already done are left
# out, so re-running is safe. Shell rc files are never edited: lines to add
# are printed at the end. The AMD tools (Vivado, Vitis, PetaLinux) can't be
# redistributed, so it prints how to get them instead. Finishes by running
# scripts/doctor.sh for the same tracks.
set -euo pipefail

# shellcheck source-path=SCRIPTDIR source=lib/common.sh
. "$(dirname "${BASH_SOURCE[0]}")/lib/common.sh"

ASSUME_YES=0 DRY_RUN=0
parse_tracks "$@"
for arg in ${REST_ARGS[@]+"${REST_ARGS[@]}"}; do
  case "$arg" in
    --yes | -y) ASSUME_YES=1 ;;
    --dry-run) DRY_RUN=1 ;;
    -h | --help) sed -n '2,21s/^# \{0,1\}//p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) die "unknown argument: $arg (see --help)" ;;
  esac
done
export ASSUME_YES

load_versions
LOCAL_BIN="$HOME/.local/bin"
LOCAL_OPT="$HOME/.local/opt"

# --- Platform ---------------------------------------------------------------
[[ "$(uname -s)" == Linux ]] || die "bootstrap runs on Ubuntu (on Windows, inside WSL2); see docs/dev-setup.md."
[[ -e /etc/NIXOS ]] && die "NixOS isn't supported by bootstrap; set up manually (docs/dev-setup.md)."
ids="$(. /etc/os-release && echo " ${ID:-} ${ID_LIKE:-} ")"
case "$ids" in
  *" ubuntu "* | *" debian "*) ;;
  *) die "unsupported distribution ($ids): bootstrap needs Ubuntu 24.04+ or Debian 13+ (WSL2 works); see docs/dev-setup.md." ;;
esac
if [[ "$(. /etc/os-release && echo "${ID:-}")" == ubuntu ]] &&
  ! version_ge "$(. /etc/os-release && echo "${VERSION_ID:-0}")" 24.04; then
  die "Ubuntu $(. /etc/os-release && echo "$VERSION_ID") is too old; use 24.04 or newer."
fi
if [[ "$(. /etc/os-release && echo "${ID:-}")" == debian ]] &&
  ! version_ge "$(. /etc/os-release && echo "${VERSION_ID:-0}")" 13; then
  die "Debian $(. /etc/os-release && echo "${VERSION_ID:-testing}") is too old (or unversioned); use 13 or newer."
fi
IS_WSL=0
grep -qi microsoft /proc/version 2>/dev/null && IS_WSL=1

case "$(uname -m)" in
  x86_64) ARCH=x86_64 ;;
  aarch64) ARCH=arm64 ;;
  *) die "unsupported CPU architecture: $(uname -m)" ;;
esac

# Root comes from sudo, or nothing when already root (containers).
as_root() {
  if [[ "$(id -u)" == 0 ]]; then
    "$@"
  elif have sudo; then
    sudo "$@"
  else
    die "this step needs root and sudo isn't installed: $*"
  fi
}
TARGET_USER="${SUDO_USER:-$(id -un)}"

# --- Packages ---------------------------------------------------------------
apt_common=(git curl ca-certificates xz-utils make g++ python3 python3-venv)
# Build dependencies for Verilator (built from source: apt's is too old for
# cocotb 2), plus libpython, which cocotb embeds in the simulator.
apt_hardware=(autoconf flex bison help2man perl libfl2 libfl-dev zlib1g-dev ccache libpython3-dev gtkwave)
apt_rpu=(gcc-arm-none-eabi libnewlib-arm-none-eabi)
apt_apu=(g++-aarch64-linux-gnu ninja-build)
apt_robot=("openjdk-$JDK_VERSION-jdk-headless")
apt_ml=(docker.io)

pkg_installed() { dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep 'install ok installed' >/dev/null; }

APT_UPDATED=0
pm_install() {
  if ((!APT_UPDATED)); then as_root apt-get -o Acquire::Retries=3 update; APT_UPDATED=1; fi
  as_root env DEBIAN_FRONTEND=noninteractive apt-get -o Acquire::Retries=3 install -y --no-install-recommends "$@"
}

# --- Plan -------------------------------------------------------------------
PLAN_DESC=()
PLAN_FN=()
step() { PLAN_DESC+=("$1"); PLAN_FN+=("$2"); }

NOTES=() # printed at the end: PATH lines and other follow-ups
note() { NOTES+=("$1"); }

wanted=("${apt_common[@]}")
for t in "${TRACKS[@]}"; do
  case "$t" in
    hardware) wanted+=("${apt_hardware[@]}") ;;
    rpu) wanted+=("${apt_rpu[@]}") ;;
    apu) wanted+=("${apt_apu[@]}") ;;
    robot) wanted+=("${apt_robot[@]}") ;;
    ml) wanted+=("${apt_ml[@]}") ;;
  esac
done
MISSING_PKGS=()
for p in "${wanted[@]}"; do
  pkg_installed "$p" || MISSING_PKGS+=("$p")
done
install_packages() { pm_install "${MISSING_PKGS[@]}"; }
((${#MISSING_PKGS[@]} == 0)) || step "Install apt packages: ${MISSING_PKGS[*]}" install_packages

# Python tools in the repo's .venv, pinned from versions.env.
venv_pkgs=()
if want_track hardware; then venv_pkgs+=("cocotb==$COCOTB_VERSION" "pytest==$PYTEST_VERSION"); fi
if want_track rpu || want_track apu; then venv_pkgs+=("cmake==$CMAKE_VERSION"); fi
venv_has() {
  local spec name want
  for spec in "$@"; do
    name="${spec%%==*}" want="${spec##*==}"
    [[ "$("$GOMPEI_VENV/bin/python" -c "import importlib.metadata as m; print(m.version('$name'))" 2>/dev/null)" == "$want" ]] || return 1
  done
}
install_venv() {
  # Recreate a .venv whose Python doesn't run here (made on another machine
  # or by another tool), and add pip to one made without it (e.g. by uv).
  if ! "$GOMPEI_VENV/bin/python" -c '' 2>/dev/null; then python3 -m venv --clear "$GOMPEI_VENV"; fi
  "$GOMPEI_VENV/bin/python" -m pip --version >/dev/null 2>&1 || "$GOMPEI_VENV/bin/python" -m ensurepip --upgrade
  "$GOMPEI_VENV/bin/python" -m pip install --upgrade --quiet pip
  "$GOMPEI_VENV/bin/python" -m pip install "${venv_pkgs[@]}"
}
if ((${#venv_pkgs[@]})) && ! { [[ -x "$GOMPEI_VENV/bin/python" ]] && venv_has "${venv_pkgs[@]}"; }; then
  step "Install ${venv_pkgs[*]} into .venv" install_venv
fi

# Verilator, built from source at the pinned tag into ~/.local.
verilator_version() {
  local v="$LOCAL_BIN/verilator"
  [[ -x "$v" ]] || v="$(command -v verilator || true)"
  [[ -n "$v" ]] && extract_version "$("$v" --version 2>/dev/null || true)"
  return 0
}
install_verilator() {
  local src
  src="$(mktemp -d)"
  git clone --depth 1 --branch "v$VERILATOR_VERSION" https://github.com/verilator/verilator.git "$src"
  (
    cd "$src"
    unset VERILATOR_ROOT
    autoconf
    ./configure --prefix="$HOME/.local"
    make -j"$(nproc)"
    make install
  )
  rm -rf "$src"
}
if want_track hardware; then
  found="$(verilator_version)"
  if [[ -z "$found" ]] || ! version_ge "$found" "$VERILATOR_VERSION"; then
    step "Build Verilator $VERILATOR_VERSION from source into ~/.local (several minutes)" install_verilator
  fi
fi

# Verible, from its release tarball into ~/.local/opt, linked into ~/.local/bin.
VERIBLE_DIR="$LOCAL_OPT/verible-$VERIBLE_VERSION"
install_verible() {
  local tmp name
  name="verible-$VERIBLE_VERSION-linux-static-$ARCH.tar.gz"
  tmp="$(mktemp -d)"
  curl -fL --progress-bar -o "$tmp/$name" \
    "https://github.com/chipsalliance/verible/releases/download/$VERIBLE_VERSION/$name"
  mkdir -p "$LOCAL_OPT" "$LOCAL_BIN"
  rm -rf "$VERIBLE_DIR"
  tar -xzf "$tmp/$name" -C "$LOCAL_OPT"
  rm -rf "$tmp"
  [[ -d "$VERIBLE_DIR/bin" ]] || die "unexpected Verible archive layout (no $VERIBLE_DIR/bin)"
  ln -sf "$VERIBLE_DIR"/bin/* "$LOCAL_BIN/"
}
if want_track hardware && [[ ! -x "$VERIBLE_DIR/bin/verible-verilog-lint" ]]; then
  step "Download Verible $VERIBLE_VERSION into $LOCAL_OPT" install_verible
fi

# Docker group and the Vitis AI image (ML track).
add_to_docker_group() {
  if confirm "Add $TARGET_USER to the docker group (run Docker without sudo)?"; then
    as_root usermod -aG docker "$TARGET_USER"
    note "Log out and back in so the docker group membership takes effect."
  else
    note "To run Docker without sudo later: sudo usermod -aG docker $TARGET_USER"
  fi
}
pull_vitis_ai() {
  if confirm "Pull $VITIS_AI_DOCKER_IMAGE now (about 10 GB)?"; then
    as_root docker pull "$VITIS_AI_DOCKER_IMAGE"
  else
    note "Pull the Vitis AI image when you need it: docker pull $VITIS_AI_DOCKER_IMAGE"
  fi
}
if want_track ml; then
  if [[ "$TARGET_USER" != root ]] && ! id -nG "$TARGET_USER" | tr ' ' '\n' | grep -x docker >/dev/null; then
    step "Add $TARGET_USER to the docker group (asks first)" add_to_docker_group
  fi
  if ! { have docker && as_root docker image inspect "$VITIS_AI_DOCKER_IMAGE" >/dev/null 2>&1; }; then
    step "Pull the Vitis AI $VITIS_AI_VERSION Docker image (asks first; large)" pull_vitis_ai
  fi
fi

# --- Show, confirm, run -----------------------------------------------------
if ((${#PLAN_DESC[@]} == 0)); then
  info "Nothing to install for tracks: ${TRACKS[*]}."
else
  info "bootstrap will (tracks: ${TRACKS[*]}):"
  for ((i = 0; i < ${#PLAN_DESC[@]}; i++)); do
    info "  $((i + 1)). ${PLAN_DESC[i]}"
  done
  if ((DRY_RUN)); then
    info "(dry run: nothing was changed)"
    exit 0
  fi
  confirm "Continue?" || die "nothing changed. Re-run with --yes to skip this question."
  # Steps run as plain commands so `set -e` stops them at the first failing
  # command; the trap names the step that failed.
  CURRENT_STEP=""
  trap 'rc=$?; if ((rc != 0)) && [[ -n "$CURRENT_STEP" ]]; then err "step failed: $CURRENT_STEP"; fi' EXIT
  for ((i = 0; i < ${#PLAN_DESC[@]}; i++)); do
    info ""
    info "${C_GREEN}==> ${PLAN_DESC[i]}${C_RESET}"
    CURRENT_STEP="${PLAN_DESC[i]}"
    "${PLAN_FN[i]}"
    CURRENT_STEP=""
  done
fi
((!DRY_RUN)) || exit 0

# --- Follow-ups and doctor --------------------------------------------------
extra_path=""
case ":$PATH:" in
  *":$LOCAL_BIN:"*) ;;
  *)
    if [[ -d "$LOCAL_BIN" ]]; then
      extra_path="$LOCAL_BIN"
      note "Add to your shell profile: export PATH=\"$LOCAL_BIN:\$PATH\""
    fi
    ;;
esac
[[ -x "$GOMPEI_VENV/bin/python" ]] &&
  note "Activate the repo's Python tools (cocotb, CMake) with: source .venv/bin/activate"

if want_track hardware || want_track rpu || want_track ml; then
  note "AMD tools ($AMD_TOOLS_VERSION) can't be installed automatically: see docs/dev-setup.md#amd-tools. They're optional on student machines; bitstreams build on the shared build machine."
fi
if want_track ml && ((IS_WSL)); then
  note "WSL2: if Docker Desktop for Windows is installed, enable its WSL integration instead of running docker.io inside WSL."
fi

if ((${#NOTES[@]})); then
  info ""
  info "Next steps:"
  for n in "${NOTES[@]}"; do info "  - $n"; done
fi

info ""
info "Running scripts/doctor.sh:"
doctor_args=()
for t in "${TRACKS[@]}"; do doctor_args+=("--$t"); done
PATH="${extra_path:+$extra_path:}$PATH" "$GOMPEI_ROOT/scripts/doctor.sh" "${doctor_args[@]}"
