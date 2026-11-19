# Stage 10: Module hardware: carrier board, enclosure and competition hardening

## Goal

Turn the dev-kit prototype into a robot-ready, Limelight-sized stereo module (a few inches on each side): our own carrier board for the SoC module, two camera boards, an M.2 SSD, an enclosure and mounting, safe on robot power and through a full season.

## Deliverables

- KiCad carrier board for the K26 SOM (or the module chosen in Stage 0b): two MIPI camera connectors, Gigabit Ethernet PHY + magnetics, an M.2 2230 NVMe slot on the Zynq's PCIe lanes, a 12 V buck-boost regulator that holds through brownouts, trigger lines for stereo, status LEDs, debug header, a CAN-FD transceiver, and the IMU near the housing's mounting points. The module carries the chip and DDR, so we never lay out DDR ourselves.
- FreeCAD aluminum enclosure that is also the heatsink (SoC thermally bonded to it), with the two lenses at the far ends of the front plate, closed against metal shavings, with filtered airflow. Chip temperature reported on the status topic. Frame grounding and ESD per the rules.
- Camera links: short, enclosed MIPI flex cables from the camera boards to the carrier, with locking connectors.
- Read-only root filesystem, brownout-tolerant boot, hardware watchdog, firmware/bitstream update over Ethernet with a fallback image.
- FRC compliance check against the [checklist](../PLAN.md#frc-rules-checklist) and the cost accounting.
- Spares and a field-swap procedure.

## Exit criteria

- [ ] Rev A boards boot, pass MIPI and Ethernet tests, and match Stage 5 numbers.
- [ ] Survives a brownout test, vibration on the robot, and a full practice day.
- [ ] Power-cut recovery measured: tag path producing measurements again within a stated time (target: a few seconds).
- [ ] Thermal: a closed module on a hot robot, with every feature and recording running for a full practice day, stays within the chip's temperature limits (measured).
- [ ] Module dimensions within the Limelight-sized target.
- [ ] Unit swap in the pits takes under 5 minutes, including config.
- [ ] Inspection-ready: power, cost and construction rules checked.

## Risks

- MIPI routing still needs care. Follow the module's carrier design guide closely, and plan for a Rev B.
