# Stage 11: Custom carrier board, enclosure and competition hardening

## Goal

Turn the dev-kit prototype into a robot-ready unit: our own carrier board for the SoC module, an enclosure and mounting, safe on robot power and through a full season.

## Deliverables

- KiCad carrier board for the K26 SOM (or the module chosen in Stage 0b): MIPI camera connectors, Gigabit Ethernet PHY + magnetics, storage, 12 V wide-input regulator, trigger lines for stereo, status LEDs, debug header. The module carries the chip and DDR, so we never lay out DDR ourselves.
- FreeCAD enclosure with heat sinking and camera mounts.
- Read-only root filesystem, brownout-tolerant boot, hardware watchdog, firmware/bitstream update over Ethernet with a fallback image.
- FRC compliance check against the [checklist](../PLAN.md#frc-rules-checklist) and the cost accounting.
- Spares and a field-swap procedure.

## Exit criteria

- [ ] Rev A boards boot, pass MIPI and Ethernet tests, and match Stage 5 numbers.
- [ ] Survives a brownout test, vibration on the robot, and a full practice day.
- [ ] Unit swap in the pits takes under 5 minutes, including config.
- [ ] Inspection-ready: power, cost and construction rules checked.

## Risks

- MIPI routing still needs care. Follow the module's carrier design guide closely, and plan for a Rev B.
