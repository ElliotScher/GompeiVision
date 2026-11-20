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

- [ ] Two students set up from scratch on different machines using only the docs.
- [ ] `scripts/doctor.sh` passes on both, and the counter test passes.
- [ ] The legacy CPU pipeline still builds in its Docker containers.

## Risks

- Vivado and Vitis AI are huge downloads and slow on student laptops. Mitigation: a shared build machine with the AMD tools installed; software-track students only need the cross-compilers.
