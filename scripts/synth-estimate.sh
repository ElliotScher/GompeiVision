#!/usr/bin/env bash
# Rough resource estimate of the PL detector with open-source tools: sv2v, then
# Yosys for UltraScale+. It needs both on the PATH; on NixOS:
#
#   nix shell nixpkgs#yosys nixpkgs#haskellPackages.sv2v -c scripts/synth-estimate.sh [TOP...]
#
# TOP is one or more module names (default: detector, the whole PL half). It
# shows that the RTL synthesizes and what it roughly costs. It says nothing
# about timing, and Vivado's numbers will differ (usually lower LUT counts).
set -euo pipefail

# shellcheck source-path=SCRIPTDIR source=lib/common.sh
. "$(dirname "${BASH_SOURCE[0]}")/lib/common.sh"

case "${1:-}" in -h | --help) sed -n '2,10s/^# \{0,1\}//p' "${BASH_SOURCE[0]}"; exit 0 ;; esac
have yosys || die "yosys not found (see --help)"
have sv2v || die "sv2v not found (see --help)"

cd "$GOMPEI_ROOT"
gv_model="${GV_MODEL:-build/model-tool/gv-model}"
[[ -x "$gv_model" ]] || die "$gv_model not found (scripts/run-sims.sh builds it)"
out=build/synth
mkdir -p "$out"
"$gv_model" lookup-tables "$out/tables" >/dev/null
mapfile -t files < <(find rtl -name '*.sv' | sort)

for top in "${@:-detector}"; do
  sv2v -DSYNTHESIS --top="$top" -w "$out/$top.v" "${files[@]}"
  # The ROMs' $readmemh needs a path at elaboration.
  sed -i "s#parameter TABLE_DIR = \"\"#parameter TABLE_DIR = \"$PWD/$out/tables\"#" "$out/$top.v"
  yosys -q -q -l "$out/$top.log" -p "read_verilog -sv $out/$top.v
    synth_xilinx -family xcup -uram -noiopad -top $top -flatten
    tee -q -o $out/$top.stat stat" >"$out/$top.out" 2>&1 || { tail -5 "$out/$top.out" >&2; die "yosys failed on $top"; }
  info "== $top"
  awk '$2 ~ /^LUT[1-6]$/ { lut += $1 } $2 ~ /^FD/ { ff += $1 } $2 ~ /^DSP/ { dsp += $1 }
       $2 ~ /^RAM64M8$/ { lutram += 8 * $1 } $2 ~ /^RAM32M16$/ { lutram += 8 * $1 }
       $2 ~ /^RAM64X1/ { lutram += $1 } $2 ~ /^SRL/ { lutram += $1 }
       $2 == "RAMB36E2" { bram += $1 } $2 == "RAMB18E2" { bram += $1 / 2 } $2 == "URAM288" { uram += $1 }
       END { printf "LUT %d (+ %d as memory)  FF %d  DSP %d  BRAM36 %g  URAM %d\n",
                    lut, lutram, ff, dsp, bram, uram }' "$out/$top.stat"
done
