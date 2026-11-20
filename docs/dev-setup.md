# Development setup

GompeiVision has five development tracks, and each one needs its own toolchain. Install only the tracks you work on. Every version is pinned in [`tools/versions.env`](../tools/versions.env). This page doesn't repeat the numbers, so they can't drift.

| Track | Flag | You work on | Tools |
|---|---|---|---|
| Hardware | `--hardware` | FPGA logic (SystemVerilog) and its simulations | Verilator, Verible, cocotb + pytest, GTKWave; Vivado (optional, see [AMD tools](#amd-tools)) |
| RPU | `--rpu` | Real-time firmware on the Cortex-R5F cores | `arm-none-eabi-gcc`, CMake; Vitis (optional) |
| APU | `--apu` | Linux apps on the module's Cortex-A53 cores | `aarch64-linux-gnu-g++`, CMake, Ninja; Docker for the legacy CPU pipeline |
| Robot | `--robot` | The SystemCore robot library | JDK (WPILib 2027) |
| ML | `--ml` | Object detection models | Docker and the Vitis AI image |

See [Team and skills](PLAN.md#team-and-skills) for who owns what.

## Quick start

Supported: **Ubuntu 24.04 or newer** (or **Debian 13**), on bare metal or in **WSL2** on Windows 10/11. CI checks both from bare images every week.

```sh
scripts/bootstrap.sh --hardware        # one track
scripts/bootstrap.sh --apu --robot     # several
scripts/bootstrap.sh                   # every track
```

Bootstrap prints what it will install and asks before doing anything. Steps that are already done are skipped, so it's safe to re-run after a pin changes. It ends by running doctor for the same tracks.

| Option | Effect |
|---|---|
| `--yes` | Don't ask |
| `--dry-run` | Print the plan and exit |

What bootstrap installs, and where:

- **apt packages:** compilers, the ARM cross-compilers, Ninja, GTKWave, the JDK, Docker, and Verilator's build dependencies.
- **Verilator:** built from source at the pinned tag into `~/.local`, because Ubuntu's package is too old for cocotb 2. Takes a few minutes.
- **Verible:** the release tarball goes in `~/.local/opt`, linked into `~/.local/bin`.
- **`.venv/` in the repo:** cocotb, pytest and CMake from PyPI. Ubuntu 24.04's CMake is older than the repo needs.
- **Docker group, Vitis AI image (`--ml`):** both ask first. The image is about 10 GB.

Bootstrap never edits your shell profile. At the end it prints the lines to add, usually `~/.local/bin` on `PATH`. Activate the Python tools with `source .venv/bin/activate`. Doctor and the self-test find `.venv` on their own.

### Windows (WSL2)

1. In PowerShell: `wsl --install -d Ubuntu-24.04`, then reboot.
2. Clone the repo **inside** WSL (e.g. `~/src/GompeiVision`), not under `/mnt/c`. Builds on the Windows filesystem are many times slower.
3. Run `scripts/bootstrap.sh` from the WSL shell.
4. ML track: if Docker Desktop is installed on Windows, turn on its WSL integration instead of installing `docker.io` inside WSL.

### Checking a setup: doctor

```sh
scripts/doctor.sh                       # every track
scripts/doctor.sh --hardware --robot    # only these tracks
scripts/doctor.sh --hardware --selftest # also build and run the cocotb counter test
scripts/doctor.sh --quiet               # only print problems
```

Doctor prints each tool's required and found version. It exits non-zero if a required tool in the chosen tracks is missing or too old. It installs nothing. For open-source tools, any version at or above the pin passes. AMD tools must match exactly. Optional items (`WARN`) never fail the run.

`--selftest` builds [`tools/selftest/counter.sv`](../tools/selftest/counter.sv) with Verilator and runs its cocotb tests ([`test_counter.py`](../tools/selftest/test_counter.py)). If it passes, the simulation toolchain works end to end. You can also run it directly with `python tools/selftest/run.py`.

## AMD tools

Vivado, Vitis and PetaLinux can't be redistributed, so bootstrap doesn't install them. They're **optional on student machines**: bitstreams build on the shared build machine, and most RTL work only needs Verilator and cocotb.

To install them yourself, use the version in `AMD_TOOLS_VERSION`. It is tied to the Vitis AI release whose DPU IP supports the K26, and Stage 0b confirms it.

1. Create a free AMD account and download the **Unified Installer** for that version from the [AMD downloads page](https://www.xilinx.com/support/download.html) (Linux self-extracting web installer).
2. Install Vivado ML **Standard** (the free tier covers the K26). Select Vitis if you work on the RPU track. You need about 100 GB of disk space.
3. Run `source <install dir>/Vivado/<version>/settings64.sh` in your shell profile, then check with `scripts/doctor.sh --hardware`.

The **Vitis AI** toolchain (training and quantization) runs in its Docker image, `VITIS_AI_DOCKER_IMAGE`. `scripts/bootstrap.sh --ml` pulls it.

## Manual setup

Use this on systems bootstrap doesn't support (other distros, older releases), or if you'd rather install things yourself. Doctor tells you when you're done.

- **Verilator** at or above `VERILATOR_VERSION`. Build it from source if your distro's package is older: `git clone -b v<version> https://github.com/verilator/verilator && cd verilator && autoconf && ./configure --prefix=$HOME/.local && make -j$(nproc) && make install`.
- **Verible:** a `linux-static` tarball from [its releases](https://github.com/chipsalliance/verible/releases), on `PATH`.
- **Python tools:** `python3 -m venv .venv && .venv/bin/pip install cocotb==<COCOTB_VERSION> pytest==<PYTEST_VERSION> cmake==<CMAKE_VERSION>`.
- **Cross-compilers:** `arm-none-eabi-gcc` (with newlib) and `aarch64-linux-gnu-g++` at or above the minimums. Doctor looks for those exact names.
- **JDK** `JDK_VERSION`, as `java` and `javac` on `PATH`.
- **Docker** at or above `DOCKER_MIN`, with your user able to reach the daemon.

## The legacy CPU pipeline

The current competition system in [`src/`](../src/) (see [legacy-cpu-pipeline.md](legacy-cpu-pipeline.md)) still builds in its Docker images, independent of the setup above:

```sh
docker build -f Dockerfile.X86 -t gompeivision-legacy .
docker run --rm -u "$(id -u):$(id -g)" -v "$PWD":/work -w /work gompeivision-legacy \
  bash -c 'cmake -B build && cmake --build build'
```

## Troubleshooting

| Symptom | Fix |
|---|---|
| `verilator` too old after bootstrap | `~/.local/bin` isn't on `PATH`, or comes after `/usr/bin` (where an apt `verilator` lives). Put `~/.local/bin` first, or `sudo apt remove verilator` |
| cocotb: `only supports a maximum Python version of …` | Your Python is newer than that cocotb release supports. Use a Python the pinned cocotb has wheels for, or bump `COCOTB_VERSION` |
| Self-test: `Unable to find libpython` | cocotb loads Python into the simulator and needs the shared library: `sudo apt install libpython3-dev` |
| Self-test fails at `make` / `g++` | Verilator compiles the design to C++: install `make` and `g++` |
| `docker: permission denied` | Add yourself to the `docker` group (`sudo usermod -aG docker $USER`), then log out and back in |
| `java` reports the wrong version | Several JDKs are installed: `sudo update-alternatives --config java` (and `javac`) |
| `vivado` shows `WARN` with a version | You have a different Vivado version on `PATH`. Source the matching `settings64.sh` |
