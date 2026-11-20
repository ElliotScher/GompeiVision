#!/usr/bin/env bash
# Run doctor and check that it *worked*, not that the machine passes: CI
# runners lack most of GompeiVision's toolchain, so doctor is expected to
# exit 1 there.
#
#   check-doctor.sh [EXPECT_REGEX...]
#
# Fails unless doctor exits 0 or 1, printed its table header and its closing
# verdict (so it ran to the end), hit no shell errors, and every EXPECT_REGEX
# matches a line of its output. Also checks that --quiet prints no OK rows.
set -uo pipefail

out="$(mktemp)"

bash scripts/doctor.sh >"$out" 2>&1
code=$?
cat "$out"
echo "--- doctor exited $code"

fail() { echo "::error::$*"; exit 1; }

[[ $code == 0 || $code == 1 ]] || fail "doctor exited $code (expected 0 or 1)"
grep -qE '^Track +Tool +Required +Found +Status' "$out" || fail "no table header"
grep -qE 'All required tools found|required tools are missing or too old' "$out" ||
  fail "no closing verdict: doctor stopped early"
if grep -nE 'line [0-9]+: |unbound variable|syntax error|command not found|bad substitution' "$out"; then
  fail "shell errors in doctor output"
fi
for re in "$@"; do
  grep -qE "$re" "$out" || fail "expected a line matching: $re"
done

# --quiet must print no OK rows.
bash scripts/doctor.sh --quiet >"$out" 2>&1
if grep -E ' OK +' "$out"; then fail "--quiet printed OK rows"; fi
echo "doctor check passed"
