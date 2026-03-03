# Team 5176 Robot Code (2026) - Detailed Subsystem Guide

This repository is the FRC 2026 robot code for Team 5176.  
This README focuses on the three game-piece movement subsystems:

- Shooter (`ShooterSubsystem`)
- Transport (`TransportSubsystem`)
- Spindexer (`SpindexerSubsystem`)

It also covers:

- how these subsystems are commanded,
- where to tune behavior,
- and how to change driver/operator bindings safely.

## Table of Contents

1. [Code Structure](#code-structure)
2. [Execution Flow](#execution-flow)
3. [Shooter Subsystem](#shooter-subsystem)
4. [Transport Subsystem](#transport-subsystem)
5. [Spindexer Subsystem](#spindexer-subsystem)
6. [How Subsystems Are Triggered by Commands](#how-subsystems-are-triggered-by-commands)
7. [Control Bindings (Current)](#control-bindings-current)
8. [How to Change Bindings](#how-to-change-bindings)
9. [Detailed Tuning Guide](#detailed-tuning-guide)
10. [Recommended Validation Procedure](#recommended-validation-procedure)
11. [Known Behaviors and Caveats](#known-behaviors-and-caveats)
12. [Quick Reference: What to Edit](#quick-reference-what-to-edit)

---

## Code Structure

Primary files for this README topic:

- `src/main/java/frc/robot/Constants.java`
- `src/main/java/frc/robot/subsystems/ShooterSubsystem.java`
- `src/main/java/frc/robot/subsystems/TransportSubsystem.java`
- `src/main/java/frc/robot/subsystems/SpindexerSubsystem.java`
- `src/main/java/frc/robot/commands/RebuiltCommands.java`
- `src/main/java/frc/robot/commands/IO.java`
- `src/main/java/frc/robot/Robot.java`

### Ownership of behavior

- `Constants.java` defines gains, limits, CAN IDs, and setpoint constants.
- Each subsystem class defines motor controller configuration + runtime control method(s).
- `RebuiltCommands.java` defines command-level actions and toggles.
- `IO.java` defines trigger thresholds and button-to-command mappings.
- `Robot.java` instantiates the subsystems and `IO` so bindings become active.

---

## Execution Flow

For shooter/transport/spindexer, the command path is:

1. Driver input is read in `IO` triggers.
2. Trigger event runs a command from `RebuiltCommands`.
3. Command calls subsystem methods on `Robot.<subsystem>` singletons.
4. Subsystem sends velocity setpoints to REV closed-loop controllers.

### Startup behavior

At robot startup (`Robot` static initialization + constructor path):

- `spindexerSubsystem`, `shooterSubsystem`, and `transportSubsystem` are constructed.
- Each subsystem configures its motor(s) with:
  - idle mode,
  - voltage compensation,
  - current limits,
  - encoder feedback sensor,
  - PID gains,
  - optional feedforward object application (present but mostly commented out).
- `IO` is constructed, registering trigger callbacks.

---

## Shooter Subsystem

File: `src/main/java/frc/robot/subsystems/ShooterSubsystem.java`

### Hardware model in code

- Motor controllers: 2x `SparkFlex` brushless
- CAN IDs:
  - Leader: `Constants.ShooterConstants.LEADERSHOOTERID` (11)
  - Follower: `Constants.ShooterConstants.FOLLOWERSHOOTERID` (12)
- Follower is configured to follow ID `12` inverted (`follow(12, true)`).

### Control mode

- Closed-loop velocity control (`ControlType.kVelocity`) on leader.
- Follower mirrors leader behavior.

### Runtime methods

- `setShooterVelocity(double velocityRPM)`
  - Sends velocity setpoint to leader controller.
- `isShooting()`
  - Returns true when absolute encoder velocity is greater than `2000` RPM.
- `atTargetSpeed()`
  - Compares current velocity to `SHOOTER_TARGET_VELOCITY_RPM` with a ±100 RPM window.
- `getVelocityRPM()`
  - Returns leader encoder velocity.

### Tunables (Shooter)

In `Constants.ShooterConstants`:

- `SHOOTER_TARGET_VELOCITY_RPM`
- `kP`, `kI`, `kD`
- `kS`, `kV`, `kA` (declared; feedforward assignment currently commented)
- `SHOOTER_MOTORS_CURRENT_LIMIT`
- `SHOOTER_MOTORS_VOLTAGE`
- `LEADERSHOOTERID`, `FOLLOWERSHOOTERID`

---

## Transport Subsystem

File: `src/main/java/frc/robot/subsystems/TransportSubsystem.java`

### Hardware model in code

- Motor controllers: 1x `SparkFlex` brushless
- CAN ID:
  - `Constants.TransportConstants.TRANSPORTID` (13)

### Control mode

- Closed-loop velocity control (`ControlType.kVelocity`).

### Runtime methods

- `setTransport(double velocityRPM)`
  - Sends velocity setpoint to transport motor closed-loop controller.

### Tunables (Transport)

In `Constants.TransportConstants`:

- `kTransportP`, `kTransportI`, `kTransportD`
- `kTransportS`, `kTransportV`, `kTransportA` (declared; feedforward assignment currently commented)
- `Transport_MOTORS_CURRENT_LIMIT`
- `Transport_MOTORS_VOLTAGE`
- `TRANSPORTID`

---

## Spindexer Subsystem

File: `src/main/java/frc/robot/subsystems/SpindexerSubsystem.java`

### Hardware model in code

- Motor controllers: 1x `SparkMax` brushless
- CAN ID:
  - `Constants.SpindexerConstants.SPINDEXERID` (31)

### Control mode

- Closed-loop velocity control (`ControlType.kVelocity`).

### Runtime methods

- `runSpindexer(double velocityRPM)`
  - Sends velocity setpoint to spindexer motor closed-loop controller.

### Tunables (Spindexer)

In `Constants.SpindexerConstants`:

- `kSpindexP`, `kSpindexI`, `kSpindexD`
- `kSpindexS`, `kSpindexV`, `kSpindexA` (declared; feedforward assignment currently commented)
- `SPINDEX_MOTORS_CURRENT_LIMIT`
- `SPINDEX_MOTORS_VOLTAGE`
- `SPINDEXERID`

---

## How Subsystems Are Triggered by Commands

File: `src/main/java/frc/robot/commands/RebuiltCommands.java`

### Primitive commands

- Shooter:
  - `shootFuel` -> set shooter to `SHOOTER_TARGET_VELOCITY_RPM`
  - `stopShoot` -> set shooter to `0`
- Spindexer:
  - `runSpindexer` -> set to `200.0`
  - `stopSpindexer` -> set to `0.0`
- Transport:
  - `startTransport` -> set to `0.5`
  - `stopTransport` -> set to `0`

### Composite toggles

- `toggleShoot` (`ConditionalCommand`)
  - If currently shooting (`isShooting` true): stop shooter + spindexer + transport.
  - Else: start shooter + spindexer + transport.
- `toggleIntake`
  - Included for intake behavior, separate from shooter chain.

This means shooter control is currently tied to spindexer + transport start/stop in one toggle path.

---

## Control Bindings (Current)

File: `src/main/java/frc/robot/commands/IO.java`

### Current controller and trigger thresholds

- `driverXbox` is on USB port `1`.
- `shootButton`: `right trigger axis > 0.7`
- `toggleIntakeButton`: `left trigger axis > 0.7`

### Current mappings

- `shootButton.onTrue(RebuiltCommands.toggleShoot)`
- `shootButton.onFalse(RebuiltCommands.toggleShoot)`
- `toggleIntakeButton.toggleOnTrue(RebuiltCommands.toggleIntake)`

Interpretation:

- Crossing RT above 0.7 executes `toggleShoot`.
- Dropping RT back below 0.7 executes `toggleShoot` again.
- Intake toggles each time LT crosses above 0.7.

---

## How to Change Bindings

Edit `src/main/java/frc/robot/commands/IO.java`.

### 1) Change controller USB port

Example:

```java
public static XboxController driverXbox = new XboxController(0);
```

Use this if your active driver controller is physically on a different USB index.

### 2) Change analog threshold sensitivity

Example:

```java
Trigger shootButton = new Trigger(() -> driverXbox.getRightTriggerAxis() > 0.5);
```

Lower threshold means earlier activation.

### 3) Change button type (axis -> physical button)

Example:

```java
JoystickButton shootButton = new JoystickButton(driverXbox, XboxController.Button.kA.value);
shootButton.onTrue(RebuiltCommands.toggleShoot);
```

### 4) Change press/release semantics

If you want press to start and release to stop (non-toggle behavior), map explicit commands instead of toggle on both transitions:

```java
shootButton.onTrue(RebuiltCommands.shootFuel
    .andThen(RebuiltCommands.runSpindexer)
    .andThen(RebuiltCommands.startTransport));
shootButton.onFalse(RebuiltCommands.stopShoot
    .andThen(RebuiltCommands.stopSpindexer)
    .andThen(RebuiltCommands.stopTransport));
```

### 5) Separate shooter from feeder systems

If you want independent control, bind shooter, transport, and spindexer to different buttons and remove the combined toggle dependency.

---

## Detailed Tuning Guide

Tuning should happen one mechanism at a time, then as an integrated system.

### A. Pre-tuning checklist

1. Verify CAN IDs in `Constants.java` match physical hardware.
2. Confirm motor inversion/follower direction with safe low setpoints.
3. Confirm encoder sign convention (positive setpoint gives expected direction).
4. Confirm voltage compensation and current limits are acceptable for your mechanism.
5. Confirm subsystem runs without brownouts or breaker trips.

### B. Shooter tuning

1. Start with low `kP`, keep `kI=0`, `kD=0`.
2. Command a moderate target RPM (example 1500-2500).
3. Increase `kP` until rise time is acceptable but oscillation is controlled.
4. Add small `kD` only if overshoot/ringing needs damping.
5. Add `kI` only if steady-state error persists after feedforward/P tuning.
6. Set `SHOOTER_TARGET_VELOCITY_RPM` to a real shot-tested value.
7. Revisit `isShooting()` threshold (currently `>2000`) so command logic aligns with real desired behavior.
8. Revisit `atTargetSpeed()` tolerance (currently ±100 RPM) based on shot quality spread.

### C. Transport tuning

1. Decide desired units and target range in RPM.
2. Confirm `startTransport` setpoint is realistic for closed-loop velocity mode.
3. Tune `kTransportP` first; keep I/D zero initially.
4. Increase P until transport tracks target with acceptable response.
5. Add D if oscillatory; add small I only for persistent offset.

### D. Spindexer tuning

1. Choose operational RPM range for indexing/feeding.
2. Start with low `kSpindexP`, then tune upward for response.
3. Add D only if needed; keep I minimal unless required.
4. Validate transition behavior when shooter starts/stops.

### E. Feedforward tuning path (optional but recommended)

The code declares `kS/kV/kA` constants, but feedforward assignment calls are currently commented in these subsystems.

To use feedforward:

1. Uncomment feedforward config lines in each subsystem.
2. Determine mechanism feedforward values via characterization/testing.
3. Apply values in constants.
4. Retune PID after enabling feedforward (P usually can be reduced).

### F. System integration tuning

After individual mechanisms are tuned:

1. Run shooter alone to stable speed.
2. Enable spindexer, observe shooter RPM dip and recovery.
3. Enable transport, observe full-load recovery.
4. Adjust shooter gains/setpoint and feed rates so shot consistency remains stable under load.

---

## Recommended Validation Procedure

Use this loop after each tuning change:

1. Deploy code.
2. Run one subsystem at a time at multiple setpoints.
3. Log/observe target vs measured RPM and current draw.
4. Trigger full shoot sequence and measure:
   - time to speed,
   - velocity droop during feed,
   - recovery time,
   - consistency across multiple cycles.
5. Repeat with battery states representative of match use.

Keep changes versioned and small so regressions are easy to isolate.

---

## Known Behaviors and Caveats

Based on current implementation:

1. `startTransport` uses `0.5` as a velocity setpoint while transport uses velocity closed-loop mode; this may be too small if interpreted as RPM.
2. Shooter toggle is bound on both trigger press and release. Depending on `isShooting()` threshold timing, this can cause unintuitive toggling.
3. Feedforward constants exist but are mostly not actively configured (commented setter lines).
4. Driver controller ports differ by class: `RobotContainer` drive controller uses port `0`, while `IO` uses port `1`.
5. In `Robot`, autonomous chooser selection from `RobotContainer` is not assigned to `m_autonomousCommand` in current code path, so autonomous scheduling may not run as intended.

---

## Quick Reference: What to Edit

### Change CAN IDs

- `Constants.java`
  - `ShooterConstants.LEADERSHOOTERID`
  - `ShooterConstants.FOLLOWERSHOOTERID`
  - `TransportConstants.TRANSPORTID`
  - `SpindexerConstants.SPINDEXERID`

### Change RPM targets / behavior thresholds

- `Constants.java`
  - `ShooterConstants.SHOOTER_TARGET_VELOCITY_RPM`
- `ShooterSubsystem.java`
  - `isShooting()` threshold
  - `atTargetSpeed()` tolerance
- `RebuiltCommands.java`
  - `runSpindexer` setpoint
  - `startTransport` setpoint

### Change PID / feedforward

- `Constants.java`
  - `ShooterConstants` gains
  - `TransportConstants` gains
  - `SpindexerConstants` gains
- Subsystem files:
  - uncomment and apply feedforward where desired

### Change bindings

- `IO.java`
  - controller port
  - threshold conditions
  - trigger/button mappings
  - toggle vs hold semantics

---

## Build / Deploy (WPILib)

Typical commands:

```bash
./gradlew build
./gradlew deploy
```

Use your standard team workflow for diagnostics and telemetry while tuning (Driver Station, dashboard, and REV client tooling as needed).
