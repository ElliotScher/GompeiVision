# Stage 6: Multiple modules: fusion and joint solves on the robot

## Goal

Several modules working together on one robot, with the robot library using every module's corners each loop.

## Deliverables

- Module identity: the board ID sets NT topics and the CAN device number, so a module can be swapped in the pits with config following its mounting position.
- Mounting-position calibration for each module (camera-to-robot transform), stored in robot code.
- [PL] trigger in/out on the module connector: one module configured as the master drives a shared trigger wire, the others expose on it, and the CAN-FD flag marks shared-trigger exposures.
- [Robot] the robot library's two multi-module modes (see [Robot-side PnP](../PLAN.md#robot-side-pnp)):
  - per-frame solves for each module, fused by the pose estimator with their own timestamps and standard deviations;
  - one joint solve over all modules' corners from the same shared-trigger exposure.
- All modules on one dedicated CAN-FD vision bus. See [Multiple modules](../PLAN.md#multiple-modules).

## Exit criteria

- [ ] Two or more modules on one robot at target FPS, with no drops for 1 hour.
- [ ] Shared-trigger exposure timestamps within 10 µs across modules.
- [ ] On recorded datasets, the fused pose is more accurate than any single module, and the joint solve is compared with per-frame fusion (especially with one or two tags per module).
- [ ] CAN-FD bus load measured with all modules at 120 FPS in a tag-heavy scene.
- [ ] Robot-side time per loop measured with all modules at the worst-case tag count.

## Risks

- Mounting-position errors show up as disagreement between modules. Calibrate with all modules seeing the same tags, and check before each event.
- If the shared trigger isn't worth the wiring, per-frame fusion is the default and the trigger stays unused.
