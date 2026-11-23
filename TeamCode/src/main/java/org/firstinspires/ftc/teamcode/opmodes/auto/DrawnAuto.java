package org.firstinspires.ftc.teamcode.opmodes.auto;
import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.pedro.PedroCommands.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;   // your Pedro Pathing 3 Constants (change the package if yours is elsewhere)
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

/**
 * Red alliance AUTO, path drawn in the BIOBUZZ simulator, rewired to use the team subsystems.
 *
 * ("motor_front_left" etc., left side reversed).
 */
@Autonomous(name = "Drawn Auto – Red", group = "BIOBUZZ")
public class DrawnAuto extends OpMode {

    // ---------------- Tunables ----------------
    /** TODO MEASURE: the point the turret aims at (Pedro inches). PLACEHOLDER VALUES. */
    private static final double GOAL_X = 72.0;
    private static final double GOAL_Y = 144.0;

    /** Flywheel must be within this many RPM of target ... */
    private static final double RPM_TOLERANCE = 75.0;
    /** ... for at least this long before a launch. */
    private static final double RPM_STABLE_SEC = 0.15;
    /** Minimum time the turret command must be steady before a launch. */
    private static final double TURRET_SETTLE_SEC = 0.10;
    /** Fire anyway if not ready after this long, so auto never hangs. */
    private static final double SPINUP_TIMEOUT_SEC = 2.0;
    /** How long the transfer feeds to empty the robot. MEASURE: time how long 3 balls take. */
    private static final int LAUNCH_MS = 1600;

    // ---------------- Hardware ----------------
    private Follower follower;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private TransferSubsystem transfer;
    private LimelightSubsystem limelight;

    // ---------------- State ----------------
    private boolean flywheelOn = false;
    private boolean intaking = false;
    private double launchWaitStart = 0.0;
    private boolean lastLaunchTimedOut = false;
    private double aimRpm = 0.0;
    private String step = "init";
    private LimelightSubsystem.SeesawState seesaw = LimelightSubsystem.SeesawState.UNKNOWN;

    @Override public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(drawn0);
        follower.update();

        shooter = new ShooterSubsystem();
        shooter.init(hardwareMap);
        intake = new IntakeSubsystem();
        intake.init(hardwareMap);
        transfer = new TransferSubsystem();
        transfer.init(hardwareMap);
        limelight = new LimelightSubsystem(hardwareMap);
        limelight.setAlliance(LimelightSubsystem.AllianceColor.RED);
    }

    @Override public void init_loop() {
        // Read the seesaw while waiting for start (camera tilts up to the hive tags).
        limelight.prepareForAiming();
        limelight.updateSeesawVote();
        seesaw = limelight.getConfirmedSeesaw();

        Pose p = follower.pose();
        telemetry.addData("Start pose", "x %.1f  y %.1f  heading %.0f",
                p.x(), p.y(), Math.toDegrees(p.heading()));
        telemetry.addData("Seesaw", "%s (votes %d)  %s",
                seesaw, limelight.getVoteCount(), limelight.getHeightDebug());
        telemetry.update();
    }

    @Override public void start() {
        flywheelOn = true;           // spin up immediately; the first launch waits until it's at speed
        schedule(drawnPath());
    }

    @Override public void loop() {
        follower.update();
        Pose p = follower.pose();

        // Turret tracks the goal the whole time, and the flywheel follows the distance-based RPM.
        aimRpm = shooter.updateAutoAim(p.x(), p.y(), p.heading(), GOAL_X, GOAL_Y);
        if (flywheelOn) shooter.runFlywheels(aimRpm);

        // The spindexer oscillation needs runActive() every loop.
        if (intaking) intake.runActive(getRuntime());

        Scheduler.execute();

        telemetry.addData("Step", step);
        telemetry.addData("Pose", "x %.1f  y %.1f  heading %.0f", p.x(), p.y(), Math.toDegrees(p.heading()));
        telemetry.addData("Flywheel", "%.0f / %.0f RPM", shooter.getCurrentRPM(), shooter.getCurrentTargetRPM());
        telemetry.addData("Turret", "pos %.2f  aimable %b", shooter.getCurrentTurretPos(), shooter.isTurretAimable());
        if (lastLaunchTimedOut) telemetry.addLine("WARNING: last launch fired on timeout (not ready)");
        telemetry.addData("Seesaw (from init)", seesaw);
        telemetry.update();
    }

    @Override public void stop() {
        shooter.stopFlywheels();
        intake.stop();
        transfer.stop();
        limelight.stop();
    }

    // >>> BIOBUZZ drawn path – made with "Draw a path" in the BIOBUZZ simulator.
    // Pedro coordinates in inches (0–144), headings in degrees.
    private final PoseFactory drawnDeg = PoseFactory.degrees();
    private final Pose drawn0 = drawnDeg.of(60.0, 8.8, 90.0);   // start
    private final Pose drawn1 = drawnDeg.of(48.5, 127.5, 90.0);
    private final Pose drawn2 = drawnDeg.of(59.0, 122.0, -90.0);
    private final Pose drawn3 = drawnDeg.of(12.0, 105.5, -178.3);
    private final Pose drawn4 = drawnDeg.of(59.0, 121.5, -90.0);
    private final Pose drawn5 = drawnDeg.of(11.5, 109.0, -180.0);
    private final Pose drawn1c = drawnDeg.of(61.0, 121.0, 0);   // bends the line into drawn1
    private final Pose drawn2c = drawnDeg.of(46.8, 105.3, 0);   // bends the line into drawn2
    private final Pose drawn3c = drawnDeg.of(37.0, 106.3, 0);   // bends the line into drawn3

    /** The drawn path: each drive, then what the robot does at that point. */
    private Command drawnPath() {
        return sequential(
                setStep("1: launch preload"),
                launchAll(),

                setStep("2: drive to drawn1"),
                follow(follower, curve(drawn0, drawn1c, drawn1).constant(drawn0)),
                setStep("3: intake at drawn1"),
                intakeOn(),
                waitMs(2000),
                intakeOff(),

                setStep("4: drive to drawn2"),
                follow(follower, curve(drawn1, drawn2c, drawn2).linear(drawn1, drawn2)),
                setStep("5: launch at drawn2"),
                launchAll(),
                waitMs(500),

                setStep("6: intake while driving to drawn3"),
                intakeOn(),
                follow(follower, curve(drawn2, drawn3c, drawn3).tangent()),
                waitMs(2000),
                intakeOff(),

                setStep("7: drive to drawn4"),
                follow(follower, line(drawn3, drawn4).linear(drawn3, drawn4)),
                setStep("8: launch at drawn4"),
                launchAll(),

                setStep("9: park at drawn5"),
                follow(follower, line(drawn4, drawn5).linear(drawn4, drawn5)),
                instant(() -> { flywheelOn = false; shooter.stopFlywheels(); }),
                setStep("done"));
    }
    // <<< BIOBUZZ drawn path

    // ---------------- Actions ----------------

    /** Wait until flywheel is at speed and turret has settled (or timeout), feed for LAUNCH_MS, then stop. */
    private Command launchAll() {
        return sequential(
                instant(() -> launchWaitStart = getRuntime()),
                waitUntil(() -> {
                    if (shooterReady()) { lastLaunchTimedOut = false; return true; }
                    if (getRuntime() - launchWaitStart > SPINUP_TIMEOUT_SEC) { lastLaunchTimedOut = true; return true; }
                    return false;
                }),
                instant(() -> transfer.feed()),
                waitMs(LAUNCH_MS),
                instant(() -> transfer.stop()));
    }

    private boolean shooterReady() {
        return shooter.isReadyStable(RPM_TOLERANCE, RPM_STABLE_SEC, getRuntime())
                && shooter.isTurretAimable()
                && shooter.isTurretSettled(TURRET_SETTLE_SEC);
    }

    private Command intakeOn() {
        return instant(() -> intaking = true);
    }

    private Command intakeOff() {
        return instant(() -> { intaking = false; intake.stop(); });
    }

    private Command setStep(String name) {
        return instant(() -> step = name);
    }
}
