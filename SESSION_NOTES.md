# Session Notes — 2026-09-11

Handoff summary of this Claude Code session, for continuity after switching logins.

## 1. Branch cleanup

- Several student branches existed on `origin` with unmerged work (`driveToPoseWorks`, `Climber`, `climberTest`, `newDriveToPoseCommand`, `newDriveToPoseMethod`, `pathEditing`).
- `origin/driveToPoseWorks` had the most recent activity (Sept 9) and was merged into `main` (merge commit `652f380`), then pushed to `origin/main`.
- The other branches (`Climber`, `climberTest`, `newDriveToPoseCommand`, `newDriveToPoseMethod`, `pathEditing`) are **still unmerged** on the remote — not touched this session.

## 2. Git auth switched from HTTPS to SSH

- `origin` remote was `https://github.com/Team-5176/5176-Rebuilt.git`; HTTPS push failed (GitHub no longer accepts password auth).
- Remote changed to `git@github.com:Team-5176/5176-Rebuilt.git`.
- A new SSH keypair was generated on this machine (`~/.ssh/id_ed25519`, ed25519, no passphrase) and the **public key was added to GitHub** (account or repo deploy key — user did this manually).
- GitHub's host key was added to `~/.ssh/known_hosts`.
- If you're now logging in as a different Windows/git user, this SSH key lives under the previous user's home directory (`C:\Users\Robotics\.ssh\`) — a new login may need its own key added to GitHub, or point at the same key, depending on how the "proper robotics login" is set up.

## 3. Code review findings + fixes (all applied, compiled clean, **not yet tested on robot, not yet committed**)

Ran a full-codebase review of `src/main/java` (2,741 lines, 13 files). Found and fixed:

1. **`RebuiltCommands.reverseTransportAndSpin`** had a `null` `BooleanSupplier` condition on its `ConditionalCommand` — this was almost certainly the recurring NullPointerException mentioned in earlier commit history. Fixed with a real condition: `() -> spindexer.isSpindexing() || transport.isTransporting()`.
2. **Shoot button did nothing but stop.** `toggleShoot` was `stopShoot.andThen(stopTransport).andThen(stopSpindexer)` — no bound control ever started the shooter/transport/spindexer. Split into `startShootSequence` / `stopShootSequence` with a real toggle (`RebuiltCommands.getToggleShoot()`) based on `ShooterSubsystem.isShooting()`.
3. **PathPlanner auto named commands** ("First Shoot Start" / "First Shoot Stop") both pointed at the same stop-only command — now point at `startShootSequence` / `stopShootSequence` respectively.
4. **Red-alliance drive-to-pose** bound the Y button three times (copy-paste) to the identical pose; X and B were unbound on red. Fixed — see item 6 below.
5. **`Vision.getBestResult()`** could NPE if the first camera result had no targets (called `getBestTarget()` without checking `hasTargets()`). Currently unreachable (nothing calls `aimAtTarget()` yet) but fixed defensively.
6. **Dead always-false clause** in `IntakeSubsystem.isIntaking()` (`Math.abs(x) < -200` can never be true) — removed.
7. **Duplicate method**: `RebuiltCommands.getReverse()` was identical to `getReverseIntake()` — removed the dead duplicate.

## 4. Button remapping

- `reverseTransportAndSpin` moved from controller button **3 (X)** to button **7 (Back/View)**. Reason: button X was double-bound — also used by `RobotContainer`'s drive-to-pose binding on blue alliance — so pressing X used to fire two unrelated commands at once.
- `configureDriveToPose()` in `RobotContainer.java` was rewritten so **X/Y/B always mean the same thing on either alliance** (left/center/right field position respectively), using the `Constants.driveToPoseConstants.RED*POSE2D` / `BLUE*POSE2D` constants that already existed in the codebase but were never wired up. **These pose values have not been verified against this year's actual field** — check them before trusting X/B near field boundaries (Y/center was already in active use before this session).

## 5. Deliverable

- `team5176_driver_controls.pdf` (project root, untracked) — one-page printable driver controller reference reflecting the current bindings above.

## 6. Current repo state (uncommitted)

```
 M src/main/java/frc/robot/RobotContainer.java
 M src/main/java/frc/robot/commands/IO.java
 M src/main/java/frc/robot/commands/RebuiltCommands.java
 M src/main/java/frc/robot/subsystems/IntakeSubsystem.java
 M src/main/java/frc/robot/subsystems/Vision.java
?? team5176_driver_controls.pdf
```

Compiles cleanly with `./gradlew compileJava` (JDK at `C:\Users\Public\wpilib\2026\jdk`, since `JAVA_HOME` isn't set globally on this machine — export it before running gradle).

## 7. Next steps

- User is installing the code on the robot to test functionality (shoot toggle, reverse transport/spindexer, drive-to-pose on both alliances).
- **Commit only after the user confirms it works on the robot** — explicitly agreed not to commit until tested.
- Once confirmed, still need real target poses for red-alliance drive-to-pose if the existing `Constants.driveToPoseConstants` values turn out to be wrong for this year's field.
- Other unmerged branches (`Climber`, `climberTest`, etc.) are still sitting on `origin`, untouched.

## 8. Reboot-loop crash found and fixed (2026-09-11, still uncommitted, still not tested on robot)

User uploaded `log-errors/log1.json` from a RoboRIO reboot loop (crashing every ~10-13s, three cycles captured). Root cause: `java.lang.IllegalArgumentException: Commands that have been composed may not be added to another composition or scheduled individually!` thrown from `RebuiltCommands.<clinit>` (the static initializer), which killed `RobotContainer`'s constructor and crashed `robotInit()` on every boot.

**Cause**: WPILib's `CommandScheduler` permanently marks a `Command` instance "composed" the first time it's used inside `.andThen()`/`ConditionalCommand`/a command group. `RebuiltCommands.java` had several `public static final Command` singletons (`stopTransport`, `stopSpindexer`, `deployIntake`, `retractIntake`, etc.) that were being reused across *multiple* different compositions — e.g. `stopTransport`/`stopSpindexer` were consumed once by `stopShootSequence` and then reused again by `reverseTransportAndSpin`. The second use throws at the moment the class loads.

Found and fixed two instances of this pattern in [RebuiltCommands.java](src/main/java/frc/robot/commands/RebuiltCommands.java):
1. `reverseTransportAndSpin` (reused `stopTransport`/`stopSpindexer`, already consumed by `stopShootSequence`) — this was the one that actually crashed at boot.
2. `angleIntake` (embedded `deployIntake`/`retractIntake` in a `ConditionalCommand`, while those same static instances were *also* bound directly to buttons in `IO.java` and to a PathPlanner auto command in `RobotContainer.java`) — this one hadn't crashed yet but was a second boot-and-a-half timebomb: pressing the intake deploy/retract button, or running the "Intake Out" auto step, would have thrown the identical exception.

**Fix**: converted both (`reverseTransportAndSpin` → `getReverseTransportAndSpin()`, `angleIntake` → `getAngleIntake()`) into factory methods that build brand-new `InstantCommand` instances on every call, matching the pattern the codebase already used for `getToggleShoot()`/`getToggleIntake()`. Also had to convert `startShootSequence`/`stopShootSequence` (used both by the shoot-toggle button binding *and* registered as PathPlanner named commands) into `getStartShootSequence()`/`getStopShootSequence()` factory methods for the same reason — the old shared static instances would have thrown the same exception the first time autonomous tried to run "First Shoot Start"/"First Shoot Stop" after the shoot-toggle button had already wrapped them in a `ConditionalCommand`.

Updated call sites: `IO.java` (button bindings) and `RobotContainer.java` (PathPlanner `NamedCommands.registerCommand` calls). Compiles clean (`./gradlew compileJava`).

**Note**: `shootFuel`, `stopShoot`, `startTransport`, `reverseTransport`, `stopTransport`, `startSpindexer`, `reverseSpindexer`, `stopSpindexer` (lines 17-26) are now unused dead fields — left in place since they're harmless (never composed with anything now) and might still be wanted for individual test bindings. Not committed yet — same "test on robot first" agreement applies, and this now needs a fresh deploy + reboot-loop retest before committing.
