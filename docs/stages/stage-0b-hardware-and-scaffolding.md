# Stage 0b: Hardware confirmation, cost check, scaffolding, simulation CI

## Goal

Confirm the SoC FPGA module, dev kit, sensors and per-module cost with real numbers, and set up the repo layout and CI so every later stage has a place to put code and tests.

## Deliverables

- **Hardware report** in this doc: the Kria K26 dev kits (KV260, KR260) against the [selection criteria](../PLAN.md#selection-criteria-stage-0b): MIPI camera ports, logic/DSP/on-chip RAM, DPU support in the current Vitis AI release, free-tier toolchain coverage, price, availability.
- A first resource estimate per PL block (capture, AprilTag pipeline, downscaler, DPU, CAN-FD, infrastructure) against the K26, with ≥30% headroom, and the largest DPU configuration that fits next to the tag pipeline (see [PLAN.md](../PLAN.md#per-module-resource-estimate)).
- On-chip RAM plan: line buffers, label tables, feature accumulators, the decode strip buffer (UltraRAM) and the DPU, against the part's block RAM and UltraRAM.
- CAN-FD check: SystemCore's CAN-FD bus count and bitrates, the WPILib API for custom FD frames (receive queue or newest-per-ID), and the CAN-FD controller (PL IP and its license, or MCP2518FD over SPI) and transceiver.
- Sensor check: OV9281 and OV9782 boards (and AR0234 as the upgrade), whether one carrier connector and one driver can take either, trigger and strobe pins, and a few M12 lens focal lengths tested for tag range and field of view.
- **Cost report:** real quotes for every line of the [module cost table](../PLAN.md#module-cost) (SOM, camera sensor and lens, carrier board fabrication and assembly, housing stock, connectors), compared with the ~$400–580 target.
- Module-size check: the K26 footprint against a Limelight-sized housing.
- Recording check: VCU support in the current tools, recording to eMMC, and a USB SSD on the dev kit for raw datasets.
- Whether a newer SoC (e.g. AMD Versal AI Edge Gen 2) is available in a small module yet.
- Purchase list: dev kit, OV9281 and OV9782 camera boards (two of each, to test multiple modules), M12 lenses, cables/adapters, a CAN-FD transceiver breakout, a USB 3.0 SSD for dev-kit datasets.
- Repo scaffolding: `rtl/`, `sim/`, `platform/`, `rpu/`, `apu/`, `robot/`, `ml/`, `model/`, `tools/tagslam/`, `hardware/`, with a README stub in each.
- GitHub Actions: Verible lint, Verilator + cocotb sims, `model/`, `apu/` and `robot/` unit tests. Bitstream builds on a nightly or self-hosted job.

## Exit criteria

- [ ] Module, dev kit and sensors chosen, with the report filled in and all numbers checked against datasheets.
- [ ] Confirmed that the current Vitis AI release supports a DPU on the K26 (or the report names the object-detection fallback).
- [ ] Per-module cost estimate from real quotes recorded, and within the target or with the gap explained.
- [ ] CI runs on every PR and is green.
- [ ] A "blinky" bitstream loads from Linux on the dev kit.

## Hardware report (draft, desk research 2026-11-20)

Everything here comes from datasheets and web listings, with sources linked. Nothing has been checked against hardware yet. Prices are list prices from the date above, not quotes.

### K26 SOM (DS987)

| Item | Value |
|---|---|
| Device | XCK26-SFVC784-2LV (custom Zynq UltraScale+ EV) |
| CLB LUTs / flip-flops | 117,120 / 234,240 |
| DSP slices | 1,248 |
| Block RAM | 144 × 36 Kb (5.1 Mb) |
| UltraRAM | 64 × 288 Kb (≈18 Mb) |
| VCU | 1, H.264/H.265, simultaneous encode/decode |
| A53 / R5F | 4 × 1333 MHz / 2 × 533 MHz |
| Memory | 4 GB 64-bit DDR4-2400, 16 GB eMMC, 512 Mb QSPI |
| PS-GTR | 4 lanes, up to 6 Gb/s: PCIe Gen2 x1/x2/x4, USB 3.0, SATA, SGMII, DisplayPort |
| GTH | 4 lanes, up to 12.5 Gb/s |
| PL I/O | 116 HPIO pairs, 69 HDIO |
| MIPI | HPA, HPB and HPC support 3 + 4 + 4 four-lane MIPI links |
| PS CAN | 2 × CAN 2.0A/B only (no FD) |
| Size / mass | 77 × 60 × 10.9 mm (without thermal solution) / 58 g |
| Connectors | 2 × Samtec ADF6-60-03.5-L-4-2-A on the SOM; mating ADM6-60-01.5-L-4-2-A on the carrier |
| Temperature | K26C 0–85 °C, K26I −40–100 °C (junction) |

What this means:
- **Headroom:** the 30% rule leaves about 82k LUTs and 870 DSPs. The PLAN's "~117k LUTs, 1,250 DSPs (check)" is now checked.
- **On-chip RAM:** the 320 KB decode strip buffer is about 2.6 Mb, roughly 14% of UltraRAM. A full on-chip double buffer (about 16.4 Mb) would take about 89%, so the PLAN is right to rule it out.
- **Storage:** the production SOM's 16 GB eMMC holds the OS and match recordings. That was chosen over an M.2 SSD on 2026-11-20 (see the revised estimate below). PCIe Gen2 on the PS-GTR lanes stays available if an SSD is ever needed.
- **CAN-FD:** PS CAN can't do FD, so the controller goes in the PL or on SPI, as PLAN.md already says.

Sources: [DS987 on docs.amd.com](https://docs.amd.com/r/en-US/ds987-k26-som) (rev 1.6) and the [DS987 v1.2 PDF](https://www.sundance.com/wp-content/uploads/docs/k26-som-Data-Sheet.pdf).

### Dev kit: KV260 vs KR260

| | KV260 | KR260 |
|---|---|---|
| MIPI camera connectors | 3: IAS0 (4-lane, through the onsemi AP1302 ISP), IAS1 (4-lane, direct to HPA), Raspberry Pi 15-pin (2-lane, direct to HPA) | **None** (SLVS-EC and USB 3.0 only) |
| Storage | microSD; no M.2/PCIe on the carrier | microSD; no M.2/PCIe on the carrier |
| USB 3.0 | 4 | 4 |
| Ethernet | 1 × GbE | 4 × GbE + SFP+ |
| Expansion | 1 Pmod | 4 Pmod + 40-pin Pi HAT header |
| CAN | none | none |
| Power | 12 V 3 A barrel (adapter not included) | 12 V 3 A (included) |
| MSRP | $249 | $349 |

**Recommendation: the KV260.** The KR260 has no MIPI CSI-2 input, so it fails selection criterion 2.
- **Camera:** prototype on the 2-lane Raspberry Pi connector, which goes straight to the PL with no ISP in the way. IAS1 is a 4-lane option for the AR0234 later.
- **CAN-FD:** put the MCP2518FD or CAN-FD transceiver breakout on the Pmod.
- **Recording:** match recording goes to eMMC (on the kit, to the SD card, because the kit's non-production SOM has no eMMC). Raw datasets go to a USB 3.0 SSD.

Sources: [UG1089 KV260 interfaces](https://docs.amd.com/r/en-US/ug1089-kv260-starter-kit/Interfaces), [UG1092 KR260 interfaces](https://docs.amd.com/r/en-US/ug1092-kr260-starter-kit/Interfaces), [KV260 2025 refresh](https://amd.com/en/blogs/2025/kria-kv260-vision-ai-starter-kit-refresh.html) (UG1089 now covers carrier rev 1.0 and rev 2.0). MSRPs are from AMD's product pages via search; those pages timed out when fetched directly.

### Vitis AI and the DPU

- **Latest release:** Vitis AI 6.3 (2026-09-24). It supports only Versal AI Edge and Versal AI Edge Gen 2, and no release after 3.5 mentions MPSoC, the K26 or DPUCZDX8G ([releases](https://github.com/amd/Vitis-AI/releases), [README](https://github.com/amd/Vitis-AI)).
- **Vitis AI 3.5** is verified with Vivado, Vitis and PetaLinux 2023.1, using DPUCZDX8G 4.1 tested on Linux 6.1. The IP is marked "not updated" (mature): no IP, reference design or board image updates in 3.5 ([release notes](https://xilinx.github.io/Vitis-AI/3.5/html/docs/reference/release_notes.html), [compatibility](https://xilinx.github.io/Vitis-AI/3.5/html/docs/reference/version_compatibility.html)).
- **Exit criterion met:** the current Vitis AI release does not cover the K26, but 3.5 + DPUCZDX8G 4.1 + 2023.1 is a supported, frozen stack. We stay on it. Moving to newer AMD tools would mean carrying the 4.1 IP forward ourselves.
- **Still to check:** whether the DPU IP packages in newer Vivado, and which host OS the 2023.1 tools install on (Ubuntu 22.04 is the safe choice for the build machine).

### VCU and recording

- **VCU:** the K26 has one VCU, and the 2023.1 tools support it ([PG252 2023.1](https://docs.amd.com/r/2023.1-English/pg252-vcu/VCU-Encoder-Features)). PetaLinux images include the encode apps.
- **Ubuntu is less ready:** one guide says Kria Ubuntu's default design doesn't include the VCU and needs AMD's patched GStreamer/V4L2 packages ([RidgeRun](https://developer.ridgerun.com/wiki/index.php/Xilinx_ZYNQ_UltraScale%2B_MPSoC/Xilinx_Kria/Enable_the_VCU_on_Ubuntu_2022.4)). That favors a PetaLinux/Yocto image in Stage 2.
- **Dev kit:** raw datasets go to a USB 3.0 SSD. Modules record match video to eMMC.

### Sensors

| | OV9281 | OV9782 |
|---|---|---|
| Type | Mono global shutter, 3 µm, 1/4" | Color (RGB Bayer) global shutter, 3 µm, 1/4" |
| Rate | 1280×800 at 120 FPS over **2 lanes** | 1280×800 at 120 FPS over 2 lanes |
| Output | RAW8/RAW10, MIPI or DVP | RAW8/RAW10, MIPI or DVP |
| Sync | Frame sync (FSIN), built-in strobe | Frame sync, built-in strobe |
| Mainline Linux driver | `ov9282.c` (`ovti,ov9281`, `ovti,ov9282`), 2-lane only, no trigger/strobe controls | none |

- **Watch the lane count when buying.** Arducam's Raspberry Pi OV9281 board (2019 datasheet) wires 1 lane and is limited to 1280×800 at 60 FPS. Other listings say 2-lane/120 FPS, so check the specific SKU. The same board uses the standard Pi 15-pin pinout, which fits the KV260's J9, and brings STROBE and FSIN out on a separate 5-pin header.
- **One carrier connector and driver for both sensors:** the sensors share pixel, format and lane count. But there is no mainline OV9782 driver, and the trigger controls aren't in the mainline OV9281 driver either. We configure the sensor over I2C from our own code anyway (PLAN: "sensor I2C config from [APU]"), so we'd write one small driver with a register table per sensor, not depend on V4L2. Off-the-shelf boards use different connectors (Pi 15-pin, OAK 22-pin), so our own camera board is what makes the two pin-compatible.
- **Prices found:**
  - Arducam OV9782 22-pin 2-lane (OAK form factor): $54.99.
  - Luxonis OAK-FFC OV9782 M12: $84.99.
  - Several Arducam MIPI OV9281/OV9782 SKUs are marked discontinued.
  - Other 2-lane OV9281 MIPI boards: Vision Components and Leopard Imaging, both quote-only.

Sources: [OV9281 product brief](https://www.ovt.com/wp-content/uploads/2024/05/OV9281-PB-v1.4-WEB.pdf), [OV9782](https://www.ovt.com/products/ov9782/), [Arducam OV9281 datasheet](https://download.kamami.pl/p576986-OV9281_MIPI_Camera_Module_DS_v2.pdf), [mainline ov9282.c](https://gbmc.googlesource.com/linux/+/refs/heads/linux-6.11.y/drivers/media/i2c/ov9282.c), [Arducam OV9782 B0335](https://www.arducam.com/?p=71701), [Luxonis OV9782 M12](https://shop.luxonis.com/products/oak-ffc-ov9782-22-pin), [Vision Components OV9281](https://www.mouser.com/new/vision-components/vision-components-ov9281-module/).

### CAN-FD hardware

The controller has three options:
- **AMD CAN FD IP (PG223):** supports Zynq UltraScale+. Two pages of its guide disagree on whether it's included with Vivado or a paid core ("check the License column in the IP catalog"), so check in Vivado 2023.1. Selling devices that contain a CAN FD controller may also need a Bosch protocol license; check whether that applies to an FRC team.
- **Open-source PL core:** not yet surveyed.
- **MCP2518FD over SPI:** about $1.50–2.75 per chip ([DigiKey](https://www.digikey.co.uk/en/products/detail/microchip-technology/MCP2518FDT-E-SL/10231313), [LCSC](https://lcsc.com/product-detail/image/MCP2518FDT-E-SL_C626758.html)). It is the cheapest and avoids the license question, at the cost of the SPI latency the PLAN already notes.

The transceiver is a TI TCAN1462-Q1 (CAN FD up to 8 Mbps, SOIC-8 or 3 × 3 mm VSON) or similar ([TI](https://www.ti.com/product/TCAN1462-Q1)).

**SystemCore side:** five buses, `can_s0`–`can_s4`. The hardware is 8 Mbit FD-capable, but the buses run CAN 2.0 in current alpha software; FD is expected later ([discussion #46](https://github.com/wpilibsuite/SystemCoreTesting/discussions/46)). Tracked as a dependency for Stage 5. The HAL has a "CAN Streams" API (allwpilib PR #9412) to check once FD lands.

### Module size

The K26 is 77 × 60 mm (3.03 × 2.36 in). In the PLAN's 3.5 × 2.5 in (88.9 × 63.5 mm) envelope, that leaves about 6 mm per side on the long axis but only about 1.75 mm per side on the short axis, before clearance and fasteners. So **the 2.5 in dimension doesn't fit** a milled wall. The module needs to be about 2.8 in (71 mm) wide, which gives about 5.5 mm per side for a 3 mm wall plus clearance, or the SOM has to stand on edge. Depth is fine: the stack is roughly camera board + flex + carrier (~1.6 mm + parts) + 10.9 mm SOM + heatsink. Stage 9 sets the final size, but the PLAN's form-factor estimate should become about 3.5 × 2.8 × 1.5 in.

### Cost check (list prices, 2026-11-20)

| Part | PLAN estimate | Found | Source |
|---|---|---|---|
| K26C SOM (SM-K26-XCL2GC) | $250–330 | **$377–406** (Avnet $377 qty 1, 182-week factory lead time, 0 stock; DigiKey ~$406; €359 in the EU) | [DigiKey](https://www.digikey.com/en/product-highlight/x/xilinx/kria-k26-som), [Avnet](https://www.avnet.com/americas/product/amd/sm-k26-xcl2gc/evolve-52912586) |
| Camera board + M12 lens | $25–60 | $55–85 off the shelf; own board TBD | above |
| Carrier PCB + assembly | $60–100 | not quoted yet | — |
| SOM mating connectors (2 × ADM6-60) | (not in the table) | **~$50** ($24–30 each) | [Octopart](https://octopart.com/part/samtec/ADM6-60-01.5-L-4-2-A-TR), [Farnell](https://ie.farnell.com/samtec/adm6-60-01-5-l-4-2-a-tr/hd-array-male-conn-240pos-4row/dp/3462775) |
| M.2 2230 NVMe SSD | $25–40 | not checked | — |
| Housing stock and hardware | $15–25 | not checked | — |
| Other connectors, CAN-FD parts, regulator | $25 | MCP2518FD + transceiver ≈ $3–5 | above |
| **Per module** | **$400–580** | **≈ $580–710**, using the SOM and connector prices above and the PLAN's numbers for the other lines | |

**The gap:** the SOM is about $50–150 above the estimate, and the two Samtec connectors add another ~$50 that the table didn't list. Together that puts the module about $0–130 above the top of the target. Ways to close it:
- AMD's academic or university program pricing for the K26 (to check).
- Buying SOMs in volume or through a distributor promotion.
- Cheaper Samtec sourcing (RS lists about £13 indicative).

**Revised plan (decided 2026-11-20):** stay on the production bare K26C, and keep object detection (the DPU adds no hardware cost). To close the gap:
- **No M.2 SSD:** record to a data partition on the SOM's 16 GB eMMC.
- **Camera on the carrier:** the sensor goes on the carrier itself, with no camera board, flex cable or camera connector.
- **Free samples:** ask Samtec for samples of the two ADM6 connectors.
- **Simpler power:** a discrete buck-boost regulator, and no PoE.

That gives about **$470–580 per module**: SOM $377–406, camera and lens $20–30, carrier $45–70, Samtec connectors $0–50, housing $15–25, other parts $15–25. PLAN.md's [Module cost](../PLAN.md#module-cost) table is updated to match. The carrier, housing and connector lines still need real quotes.

The 182-week Avnet lead time means stock at DigiKey is what makes the K26 buyable now; check stock before committing to a module count.

### Newer SoCs

Versal AI Edge Gen 2 SOMs exist, but none is a small, cheap option yet:
- **iWave iG-G77M:** 70 × 80 mm, with samples for early-access customers only ([iWave](https://iwave-global.com/press-release/iwave-launches-versal-ai-edge-gen2-system-on-module/)).
- **New Wave V3211:** VITA 93, starting around $5,000 ([Military Embedded](https://militaryembedded.com/radar-ew/rugged-computing/new-wave-design-unveils-the-v3211-small-form-factor-sff-som-powered-by-amds-versal-ai-edge-gen-2-adaptive-soc)).

Both are bigger than the K26 and far more expensive. **Verdict:** keep watching, and stay on the K26.

### Still open (needs hardware, quotes or the AMD tools)

- Real quotes: carrier fabrication and assembly, SSD, aluminum stock, connectors.
- Whether the CAN FD IP is free in the Vivado 2023.1 IP catalog.
- A first DPU size: build DPUCZDX8G configurations (B512–B4096) in Vivado for real LUT and DSP counts, then pick the largest that fits after the tag pipeline.
- Lens tests for tag range and field of view.
- A blinky bitstream loaded from Linux on the KV260.

### Purchase list (draft)

| Item | Qty | Approx. each |
|---|---|---|
| KV260 Vision AI Starter Kit (+ 12 V 3 A adapter, microSD) | 1 (2 for multi-module tests) | $249 |
| 2-lane OV9281 MIPI board with Pi 15-pin connector, FSIN/STROBE exposed (check the SKU is 2-lane) | 2 | ~$50 |
| OV9782 2-lane MIPI board (needs an adapter from 22-pin OAK to Pi 15-pin, or the IAS connector) | 2 | ~$55–85 |
| M12 lenses, a few focal lengths (e.g. 2.8, 3.6, 6 mm) | 3–4 | ~$10 |
| MCP2518FD click/breakout (Pmod or SPI) + CAN-FD transceiver | 2 | ~$20 |
| USB 3.0 SSD (dev kit recording) | 1 | ~$40 |

## Risks

- Vitis AI's focus has been shifting to newer AMD chips; check its support for Zynq UltraScale+ before buying.
- SOM prices change; the cost report records the date of each quote.
