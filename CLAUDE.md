# FTC Team 20223 Steel City Dynamics — BIOBUZZ (2026-27)

Robot code for an FTC robot. SDK 12.0, Pedro Pathing (`com.pedropathing:revhub:3.0.2`), Java, Android Studio. Team code lives in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.

The team's rules come from the **Programming MOP** (Steel City Dynamics Programming Manual of Operating Procedures, PDF, ask Vish for it). The game rules come from the **BIOBUZZ Competition Manual** (TU10). Older history and full source of unmoved code: `CLAUDE_HANDOFF.md`.

## Working with Vish

- Vish is the programming lead and is learning Java. Explain in plain, short, step-by-step language with simple examples and tables. Ask questions as plain text.
- He types by voice: "petro/header padding" = Pedro Pathing, "LED" = LUT, "Aria" = Arya. Guess from context and say what you assumed.
- Team: Arya (programmer, Pedro setup, prefers calls), Ashwin (build/CAD lead, source of hardware answers), Joey (turret build), Halil (Limelight), Alex and Rohit (programmers), Ms. Minnie (coach).

## Writing code

- **Simple** code: small surgical changes, no refactors, no "while I'm in here" edits.
- Comments sound like Vish: short, mostly lowercase, casual, about one per chunk (`// close enough, stop`, `// camera doesnt send anything without this`). Section headers are plain, like `//SHOOTER`. Long explanations go in chat.
- MOP rules that shape every change:
  - All hardware goes through a subsystem; OpModes call subsystems.
  - Tuned numbers live in `utils/Constants.java`. Pedro's tuned values live in Pedro's own constants (`pedro/Constants.java`).
  - Null-check anything nullable, especially vision results (`result != null && result.isValid()`).
  - Every vision-dependent auto has a no-vision fallback.
  - Names: classes PascalCase, variables camelCase, constants ALL_CAPS, OpModes `TeleOp_X` / `Auto_X`. Config names descriptive (where → what → device type, e.g. `leftLiftMotor`) and matching the code exactly.
  - Edge-detect button presses; deadband the sticks; bulk reads on AUTO; loop time on telemetry.
- Ask Vish first before: editing Gradle files, changing SDK/Pedro versions, adding libraries, changing gamepad bindings in real OpModes (drivers sign off), touching someone else's file or subsystem.
- After writing code: compile-check it, laptop-test the math with throwaway files in the scratchpad, then walk Vish through it piece by piece.

## Building and checking

- VS Code shows fake red "not on classpath" errors on FTC code. Real checks are Gradle or javac.
- Real compile check (works, from repo root in Git Bash): `ANDROID_HOME="$LOCALAPPDATA/Android/Sdk" ./gradlew :TeamCode:compileDebugJavaWithJavac -q`. Without `ANDROID_HOME` it fails with "SDK location not found" (no `local.properties`).
- Laptop tests: compile a throwaway test in the scratchpad against `TeamCode/build/intermediates/javac/debug/compileDebugJavaWithJavac/classes` + Pedro's `core-3.0.2.jar`. javac on Windows needs `cygpath -w` paths joined with `;`.
- **Pedro 3.x API:** `com.pedropathing.math.Pose` (not `geometry`), getters `x()`, `y()`, `heading()` (radians); `follower.pose()`, `follower.setPose(...)`, `follower.update()`. No sources jar is cached; check with `javap`.
- Fallback: `javac` single files against jars in the Gradle cache:
  - FTC SDK: `%USERPROFILE%\.gradle\caches\modules-2\files-2.1\org.firstinspires.ftc\<Module>\<ver>\...\*.aar` → unzip `classes.jar` (RobotCore, Hardware, FtcCommon, Vision).
  - Android: `%LOCALAPPDATA%\Android\Sdk\platforms\android-37.0\android.jar`
  - Pedro: `%USERPROFILE%\.gradle\caches\modules-2\files-2.1\com.pedropathing\core\3.0.2\...\core-3.0.2.jar`.
- Android Studio: Gradle JDK = embedded jbr-21. Leave the Gradle version alone.

## Git

- Branches: `master` (counts as MOP "main") ← `dev` ← `feature/<name>-<thing>`. Work on feature branches off `dev`; PRs go into `dev`. `master` only receives team-approved merges from `dev`.
- Per-person branches exist off `dev`: `feature/vishnu`, `feature/alex`, `feature/arya`, `feature/halil`, `feature/rohit` (local; Vish publishes them). Each gets renamed to `feature/<name>-<task>` once a task is picked.
- Commit message: `<what part>: <what you did>`, e.g. `Shooter: added LookupTable class (tested on laptop)`.
- Vish usually uses GitHub Desktop. Make branches/commits/pushes yourself only when he asks; otherwise give GitHub Desktop steps.
- AI-written code is allowed by the MOP (rule 6) when checked. Keep it visible; if asked to hide it, decline in one line and keep helping with the code.
- Old repo `../20223-New-Team-Repository` still holds unmoved work: branch `feature/vish-shooter-lut` (commit `4c95fe9`, `LookupTable` + `AimingHelper`, 16/16 and 17/17 laptop tests) and a GitHub Desktop stash `stash@{0}` (ball tracking: `VisionSubsystem` multi-target methods, `//BALL TRACKING` constants, `TeleOp_BallTrackingTest`). Full source also in `CLAUDE_HANDOFF.md` appendix.

## Team WhatsApp messages

When Vish asks for a reply to send: sound like a 10th grader (lowercase, "u", "bc", "lmk", "ngl"), plain text, several short separate texts (exactly N if he says N), ask only what the reader can answer. Open your reply to Vish with one line naming the audience.

## Robot hardware (CAD: Onshape "BioBuzz SCD 20223 Robot V2" — look only, never edit)

Config names are placeholders until the robot is wired.

- **Drive:** 4-motor mecanum. Current names `motor_front_left/back_left/front_right/back_right`, right side reversed, `forward = -gamepad1.left_stick_y`. `imu`: logo FORWARD, USB LEFT.
- **Localization:** goBILDA Pinpoint + 2 dead wheels, via Pedro's follower. `pedro/Constants.create()` returns `null` until Arya finishes Pedro setup, so anything using the follower crashes until then.
- **Shooter:** two 96mm wheels on one 120mm shaft (one RPM target), driven by **2 motors** (both with encoders, same `setVelocity`, one reversed). goBILDA 5000-series, 28 ticks/rev; gear ratio unconfirmed (assume 1.0). Motor not in CAD yet.
- **Hood:** fixed part, moved by hand between matches. LUT tables are valid only for the hood setting they were tuned at.
- **Flywheel mover ("adjusting thingy"):** 1 goBILDA dual-mode servo + pinion + passive transfer gear driving racks that slide the wheel carriage to fit pollen vs nectar.
- **Turret:** goBILDA **5-turn** Dual Mode Torque servo (2000-0025-0502), 64t gear → 176t ring (2.75:1), about 650° of travel. No position feedback, so use a settle time. Turret axis is near robot center; exact offset unmeasured.
- **Spindexer:** 2 Axon servos locked to one gear (3:1). Always command both to the same position.
- **Other:** intake 1 motor, transfer 1 motor + 1 servo, Limelight 3A (config `limelight`) on a mount servo (`limelight_servo`). Motor cap 8, servo cap 8.
- `IntakeSubsystem` currently drives 2 motors (`intake_motor_1/2`), probably misnamed shooter motors. Sort it out with the shooter work, without duplicating it.

## BIOBUZZ facts that matter for code

- AUTO 30 s, 8 s transition (robot motionless, G403), TELEOP 2:00. NECTAR into FLOWERS only in the last 60 s (G410).
- POLLEN yellow ~2.8 in, NECTAR red/blue ~3.6 in. Control at most 4 elements (G407). Robot starts with 4 preloaded POLLEN.
- HIVE: two CELLS on a pivot, one faces up; launching into the up CELL tips it (20 pts). Opening bottom ~53.5 in, top ~65.6 in above tiles. The CELL pointing at a FLOWER starts down.
- AprilTags (36h11) sit under the CELLS and **move when the HIVE tips**, so Limelight botpose is unusable for field localization this season. Tags 30–33 red far, 34–37 red audience, 38–41 blue audience, 42–45 blue far.
- AUTO: stay on your side (red columns A–C, blue D–F, G402). No catching falling balls (G409). Tip the HIVE only by launching (G417).
- FTC Dashboard / Panels are illegal in matches (R704.D); use them only for practice tuning.

## Design decisions

- **Shooter = lookup table.** Distance → flywheel RPM, linear interpolation, ~8–12 rows, never extrapolate (`inRange()` false → don't shoot). Separate tables for POLLEN and NECTAR. Velocity control (`setVelocity`), tuned PIDF first (start F 11.7, P 1.2). Keep tables under ~5000 RPM.
- **Fire only when** in range + turret reachable + turret settled + flywheel ready (±50 RPM for 3 loops).
- **Turret aims from Pedro odometry only.** Distance = flat floor distance from turret axis to the up CELL's opening.
- **Limelight is for finding balls**, not HIVE aiming. Ball pipelines: 1 pollen, 2 red nectar (hue inversion), 3 blue nectar. Slot 6 = old AprilTag pipeline, keep it as is.
- **Which CELL is up:** known at match start, then a driver "HIVE tipped" button flips it.
- **One TeleOp**, alliance picked during INIT. Auto saves its final pose for TeleOp.

## Current work: shooter + turret (agreed 2026-11-22, on `feature/vishnu-shooter-turret`)

Status 2026-11-23: all files below written, Gradle compile passes, 13/13 laptop math tests pass, not yet run on the robot, not committed. Test OpModes set Pedro's start pose from `TEST_START_X/Y/HEADING_DEG` in Constants.

- `utils/LookupTable.java`, `utils/AimingHelper.java`: copied from the old repo, updated to the Pedro 3.x Pose API.
- `utils/Constants.java`: add `//AIMING` (cell aim points, placeholders 72,72; `RED_FAR_CELL_STARTS_UP=false`, `BLUE_FAR_CELL_STARTS_UP=true`; turret offset 0,0), `//SHOOTER`, `//TURRET`.
- `subsystems/ShooterSubsystem.java`: 2 flywheel motors, mover servo (`MOVER_POLLEN_POS` / `MOVER_NECTAR_POS`), hood servo code behind `USE_HOOD = false`, RPM-only tables with a `HOOD_SETTING_NOTE`, `FLYWHEEL_GEAR_RATIO = 1.0`, `IDLE_RPM = 0`, `isReady()`. Feeding is out of scope (transfer, later).
- `subsystems/TurretSubsystem.java`: angle → servo position via `TURRET_CENTER_POS` and position-per-degree, limits ±180°, reports "can't reach", settle time scaled by move size, centers in INIT and when not aiming, tracks only after start. Check the 5-turn servo's PWM range (likely 500–2500 µs).
- `opmodes/teleop/TeleOp_ShooterTuning.java` and `TeleOp_TurretTest.java`: test OpModes, both usable without Pedro (manual distance; manual turret calibration). Buttons: INIT X = blue, B = red; A = pollen/nectar, Y = HIVE tipped, d-pad up/down = RPM ±50 or turret nudge, bumpers = manual distance ±6 in. Telemetry shows loop time.
- Placeholder config names: `shooterMotor1`, `shooterMotor2`, `hoodServo`, `turretServo`, `flywheelMoverServo`.

## Open items

- Ashwin: turret axis offset from robot center, flywheel motor gearing, whether a hood servo is coming, how far turret wiring can twist.
- Arya: Pedro setup (`Constants.create()`), and his unpushed shooter code (should go to `feature/arya-shooter` → PR into `dev`).
- Pedro folder is `pedro/`; MOP wants `pedroPathing/`. Rename only with Arya's OK.
- `teleopLimelight.java` grabs hardware directly in the OpMode (breaks MOP rule 7). Someone else's file; ask before refactoring.
- `master` → `main` rename and branch protection on `master` (MOP 3.12/3.14), done by the repo owner (dolphindot10).
- Measure on the real field/robot: cell aim points, turret center/range, starting-cell assumption, forward direction.
