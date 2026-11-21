# robot/

The robot library for SystemCore (Java first, with an optional C++ core over JNI if measurement says it's needed). It reads every module's CAN-FD corner frames each loop, recovers capture times from the age field, and solves PnP: IPPE-square per tag, multi-tag refinement across modules, and the optional heading-constrained solve. See [Robot-side PnP](../docs/PLAN.md#robot-side-pnp).

- **Toolchain:** JDK and WPILib at `JDK_VERSION` / `WPILIB_ROBOT_VERSION` in [`tools/versions.env`](../tools/versions.env), built with Gradle.
- **Filled in by:** [Stage 5](../docs/stages/stage-5-pose-and-mvp.md). It can start after Stage 3, against NAPA's corners.
- **CI:** once `robot/build.gradle` exists, CI runs `./gradlew test` here.
