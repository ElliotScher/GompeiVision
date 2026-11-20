# Stage 0a: Dev environment and toolchain checks

## Goal

Any team member can set up the toolchains for their track (hardware, real-time firmware, Linux apps, robot library, ML) on Linux (and Windows via WSL2) in one command, and check it with another.

## Deliverables

- `tools/versions.env` pinning Vivado/Vitis, the Linux image tooling, Vitis AI, Verilator, cocotb, Python, the AArch64 and ARM bare-metal cross-compilers, CMake and WPILib library versions.
- `scripts/bootstrap.sh`: installs the open-source tools, with `--hardware`, `--rpu`, `--apu`, `--robot` and `--ml` flags so students only install what they need. Prints instructions for the AMD tools (which can't be redistributed).
- `scripts/doctor.sh`: checks each tool and version, and reports optional ones.
- `docs/dev-setup.md`: per-OS instructions and troubleshooting.
- A "hello" cocotb test of a trivial counter, run by `doctor.sh --selftest`.

## Exit criteria

- [x] ~~Two students set up from scratch on different machines using only the docs.~~ Waived. Clean-image runs of bootstrap (below, and in CI) stand in for it.
- [x] `scripts/doctor.sh` passes, and the counter test passes.
- [x] The legacy CPU pipeline still builds in its Docker containers.

## Results (2026-11-20)

**What was built**
- `tools/versions.env` is the single source of pins.
  - **Open-source tools:** bootstrap installs exactly the pinned version. Doctor accepts that version or newer.
  - **AMD tools:** must match exactly.
  - **Main pins:** Verilator 5.048, Verible v0.0-4023, cocotb 2.1.0, JDK 25 (WPILib 2027) and the AMD tools at 2023.1.
- `scripts/bootstrap.sh` and `scripts/doctor.sh`, with shared helpers in `scripts/lib/common.sh`. Both take `--hardware`, `--rpu`, `--apu`, `--robot` and `--ml`, and default to all tracks.
  - **bootstrap.sh:** supports Ubuntu 24.04+ and Debian 13+, bare metal or WSL2. It prints its plan, asks before installing, and skips steps that are already done. It never edits shell profiles. It ends by running doctor.
  - **doctor.sh:** prints a table of required and found versions. `--quiet` prints only problems, and `--selftest` runs the counter test.
- `tools/selftest/`: `counter.sv` plus three cocotb tests (count, hold, wrap), built with Verilator `-Wall` through cocotb's Python runner.
- `docs/dev-setup.md`: setup instructions, including WSL2, the AMD tools, manual setup and troubleshooting.
- `.github/workflows/devenv.yml`, which runs on pushes and weekly:
  - shellcheck on the scripts.
  - Doctor on a hosted runner and in a bare container.
  - Bootstrap from bare `ubuntu:24.04` and `debian:13` images, followed by the self-test and a check that a second run installs nothing.

**Decisions and findings**
- **AMD tools 2023.1:** Vitis AI 3.5 and its DPUCZDX8G 4.1 IP (the K26 DPU) are verified only with the 2023.1 tools, so that pin is provisional until Stage 0b confirms the DPU path.
- **Verilator from source:** Ubuntu 24.04's apt Verilator is 5.020, but cocotb 2 needs at least 5.036, so bootstrap builds the pinned tag into `~/.local`. This takes a few minutes.
- **CMake from PyPI:** Ubuntu 24.04's CMake is 3.28, but the repo needs 3.30, so bootstrap installs 3.31.10 (the version the legacy Dockerfiles use) into `.venv`.
- **cocotb 2.1.0, not 2.0.1:** 2.0.1 refuses to install on Python 3.14.
- **libpython:** cocotb embeds Python in the simulator, so it needs `libpython3-dev`, which a bare Ubuntu install lacks. The first clean-image run caught this.
- **Doctor's checks:**
  - Doctor reads `-dumpfullversion` from the cross-compilers, because Ubuntu's `-dumpversion` prints only `13`.
  - GTKWave is only checked for presence, because `--version` needs a display.
  - Vivado, Vitis and the Vitis AI image are optional warnings. Bitstreams build on the shared build machine.

**Verification**
- **Clean `ubuntu:24.04` container, as a non-root sudo user:** `bootstrap.sh --yes --hardware --rpu --apu --robot` installed everything, and its closing doctor run passed. A second run planned nothing. `doctor.sh --selftest` passed 3/3.
- **Clean `debian:13` container, as root (as CI runs it):** the same sequence passed, starting from a stale `.venv` copied from another machine, which bootstrap rebuilt.
- **Legacy pipeline:** `Dockerfile.X86` builds, and the pipeline compiles and packages `GompeiVision-1.0.3-amd64.deb` inside it. `Dockerfile.ARM` wasn't built locally, because the dev machine has no arm64 emulation. The existing `build.yml` builds it on an arm64 runner on every push.
- **ML track:** not exercised in containers, since they have no Docker daemon. Doctor's Docker checks passed on the dev machine.

## Risks

- Vivado and Vitis AI are huge downloads and slow on student laptops. Mitigation: a shared build machine with the AMD tools installed; software-track students only need the cross-compilers.
