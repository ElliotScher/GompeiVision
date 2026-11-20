# Stage 9: Module hardware: carrier and camera boards, machined enclosure and competition hardening

## Goal

Turn the dev-kit prototype into a robot-ready, Limelight-sized single-lens module (a few inches on each side): our own carrier board for the K26, our own camera board, an M.2 SSD and a housing machined on our 3-axis mill, safe on robot power and through a full season, at the [target cost](../PLAN.md#module-cost).

## Deliverables

- KiCad carrier board for the K26 SOM: one MIPI camera connector, Gigabit Ethernet PHY + magnetics, an M.2 2230 NVMe slot on the Zynq's PCIe lanes, a 12 V buck-boost regulator that holds through brownouts, a CAN-FD transceiver, trigger in/out, status LEDs and a debug header, all external signals on one locking back-panel connector plus Ethernet. An unpopulated IMU footprint for later. The module carries the chip and DDR, so we never lay out DDR ourselves.
- KiCad camera boards for the OV9281 and OV9782 (one layout if they're pin-compatible) with an M12 lens mount.
- FreeCAD aluminum enclosure, machined in-house on the 3-axis mill (pocketed shell and back plate), that is also the heatsink (SoC thermally bonded to it), closed against metal shavings, with mounting holes matching common camera mounts. A small filtered fan only if the thermal test needs one. Frame grounding and ESD per the rules.
- Camera link: a short, enclosed MIPI flex cable from the camera board to the carrier, with locking connectors and strain relief.
- Read-only root filesystem, brownout-tolerant boot, hardware watchdog, firmware/bitstream update over Ethernet with a fallback image.
- FRC compliance check against the [checklist](../PLAN.md#frc-rules-checklist) and the cost accounting, with the final per-module cost recorded here.
- Spares and a field-swap procedure.

## Exit criteria

- [ ] Rev A boards boot, pass MIPI, Ethernet and CAN-FD tests, and match Stage 5 numbers.
- [ ] Survives a brownout test, vibration on the robot, and a full practice day.
- [ ] Power-cut recovery measured: CAN-FD corners flowing again within a stated time (target: a few seconds).
- [ ] Thermal: a closed module on a hot robot, with tags, object detection and recording running for a full practice day, stays within the chip's temperature limits (measured).
- [ ] Module dimensions within the Limelight-sized target.
- [ ] Per-module cost recorded and within the target, or the gap explained.
- [ ] Unit swap in the pits takes under 5 minutes, including config.
- [ ] Inspection-ready: power, cost and construction rules checked.

## Risks

- MIPI routing still needs care. Follow the module's carrier design guide closely, and plan for a Rev B.
- Small-batch board assembly costs can dominate. Compare assemblers, and panelize camera boards.
