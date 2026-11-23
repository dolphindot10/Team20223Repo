/*
package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem.SeesawState;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

import java.util.Locale;

@Autonomous(name = "Biobuzz 4-Cycle Aggressive Auto", group = "Competition")
public class AggressiveBiobuzzAuto extends LinearOpMode {

    public static class AutoConfig {
        // ---- Alliance ----
        public static LimelightSubsystem.AllianceColor ALLIANCE = LimelightSubsystem.AllianceColor.RED;

        // ---- Start pose ----
        public static double START_X = 72.0;
        public static double START_Y = 12.0;
        public static double START_HEADING = Math.PI / 2.0;

        // ---- Seesaw / scoring targets ----
        public static double LEFT_RAISED_TARGET_X = 65.0;
        public static double RIGHT_RAISED_TARGET_X = 79.0;
        public static double RAISED_TARGET_Y = 144.0;
        public static SeesawState DEFAULT_SEESAW = SeesawState.LEFT_RAISED;

        // ---- Pinpoint ----
        public static double PINPOINT_X_OFFSET_MM = -120.0;
        public static double PINPOINT_Y_OFFSET_MM = 45.0;
        public static GoBildaPinpointDriver.EncoderDirection X_ENCODER_DIR = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        public static GoBildaPinpointDriver.EncoderDirection Y_ENCODER_DIR = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        public static long PINPOINT_IMU_SETTLE_MS = 300;

        // ---- Timing ----
        public static double MAX_AUTO_RUNTIME = 29.5;
        public static double MIN_TIME_FOR_CYCLE_SEC = 6.0;
        public static int MAX_CYCLES = 4;

        // ---- Shooting ----
        public static double TURRET_SETTLING_DELAY_SEC = 0.15;
        public static double AIM_READY_TOL_RPM = 80.0;
        public static double AIM_STABLE_SEC = 0.15;
        public static double FEED_READY_TOL_RPM = 120.0;
        public static double AIM_TIMEOUT_SEC = 2.0;
        public static double FEED_TIME_SEC = 1.0;
        public static double FEED_MAX_STATE_SEC = 2.0;
        public static double FALLBACK_SHOT_RPM = 2200.0;

        // ---- Driving ----
        public static double DRIVE_KP = 0.035;
        public static double LATERAL_KP = 0.05;
        public static double HEADING_KP = 1.5;
        public static double MIN_DRIVE_POWER = 0.15;
        public static double MAX_LATERAL_POWER = 0.4;
        public static double LATERAL_SIGN = 1.0;
        public static double POSITION_TOL_IN = 2.0;

        public static double SWEEP_X = 72.0;
        public static double SWEEP_BASE_Y = 65.0;
        public static double SWEEP_STEP_Y = 5.0;
        public static double SWEEP_MAX_POWER = 0.7;
        public static double SWEEP_TIMEOUT_SEC = 2.5;

        public static double RETRACT_DISTANCE_IN = 8.0;
        public static double RETRACT_MAX_POWER = 0.6;
        public static double RETRACT_TIMEOUT_SEC = 1.0;

        public static double PARK_X = 72.0;
        public static double PARK_Y = 12.0;
        public static double PARK_MAX_POWER = 0.7;
        public static double PARK_TIMEOUT_SEC = 2.0;

        // ---- Intake ----
        public static boolean SELECTIVE_INTAKE = false;

        // ---- Safety ----
        public static double ODOMETRY_STALE_SEC = 0.5;
    }

    private final MecanumDriveSubsystem drive = new MecanumDriveSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final TransferSubsystem transfer = new TransferSubsystem();
    private LimelightSubsystem limelight;

    private enum AutoState {
        AIM_AND_SPINUP,
        FEED_TARGET,
        SWEEP_COLLECT,
        RETRACT_BACK,
        PARK,
        IDLE,
        FAULT_STOP
    }

    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime odometryStaleTimer = new ElapsedTime();

    private AutoState currentState = AutoState.AIM_AND_SPINUP;
    private int currentCycle = 1;

    private SeesawState seesaw = SeesawState.UNKNOWN;
    private double activeTargetX = AutoConfig.LEFT_RAISED_TARGET_X;
    private double activeTargetY = AutoConfig.RAISED_TARGET_Y;

    private double feedTimeAccum = 0.0;
    private String skipReason = "";

    @Override
    public void runOpMode() {
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        shooter.init(hardwareMap);
        transfer.init(hardwareMap);

        limelight = new LimelightSubsystem(hardwareMap);
        limelight.setAlliance(AutoConfig.ALLIANCE);

        shooter.setHoodPosition(0.25);
        shooter.setFlywheelShift(0.1);
        transfer.stop();

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(AutoConfig.PINPOINT_X_OFFSET_MM, AutoConfig.PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(AutoConfig.X_ENCODER_DIR, AutoConfig.Y_ENCODER_DIR);
        pinpoint.resetPosAndIMU();
        sleep(AutoConfig.PINPOINT_IMU_SETTLE_MS);

        SeesawState confirmedSeesaw = SeesawState.UNKNOWN;
        double lastShotRPM = AutoConfig.FALLBACK_SHOT_RPM;
        while (!isStarted() && !isStopRequested()) {
            pinpoint.update();
            limelight.prepareForAiming();
            limelight.updateSeesawVote();

            SeesawState c = limelight.getConfirmedSeesaw();
            if (c != SeesawState.UNKNOWN) confirmedSeesaw = c;
            applySeesaw(confirmedSeesaw != SeesawState.UNKNOWN ? confirmedSeesaw : AutoConfig.DEFAULT_SEESAW);

            lastShotRPM = shooter.updateAutoAim(
                    AutoConfig.START_X, AutoConfig.START_Y, AutoConfig.START_HEADING, activeTargetX, activeTargetY);

            Pose2D p = pinpoint.getPosition();
            telemetry.addData("=== SEESAW VISION ===", "");
            telemetry.addData("Confirmed", confirmedSeesaw == SeesawState.UNKNOWN
                    ? "NOT YET (using default " + AutoConfig.DEFAULT_SEESAW + ")" : confirmedSeesaw);
            telemetry.addData("Agreeing frames", limelight.getVoteCount() + "/" + LimelightSubsystem.Config.CONFIRM_FRAMES);
            telemetry.addData("Tag heights", limelight.getHeightDebug());
            telemetry.addData("Target X/Y", "%.1f, %.1f", activeTargetX, activeTargetY);
            telemetry.addData("=== ODOMETRY CHECK (don't move the robot) ===", "");
            if (p != null) {
                telemetry.addData("Pinpoint X/Y/H", "%.1f, %.1f, %.2f rad",
                        p.getX(DistanceUnit.INCH), p.getY(DistanceUnit.INCH), p.getHeading(AngleUnit.RADIANS));
            }
            telemetry.update();
        }

        if (isStopRequested()) {
            limelight.stop();
            return;
        }

        boolean seesawConfirmed = confirmedSeesaw != SeesawState.UNKNOWN;
        applySeesaw(seesawConfirmed ? confirmedSeesaw : AutoConfig.DEFAULT_SEESAW);

        runtime.reset();
        odometryStaleTimer.reset();
        stateTimer.reset();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, AutoConfig.START_X, AutoConfig.START_Y,
                AngleUnit.RADIANS, AutoConfig.START_HEADING));

        enterState(AutoState.AIM_AND_SPINUP);

        double lastValidX = AutoConfig.START_X;
        double lastValidY = AutoConfig.START_Y;
        double lastValidHeading = AutoConfig.START_HEADING;
        boolean hasReceivedValidPose = false;
        boolean odometryValid = false;
        double lastLoopTime = 0.0;
        double retractTargetY = AutoConfig.START_Y;

        try {
            while (opModeIsActive() && runtime.seconds() < AutoConfig.MAX_AUTO_RUNTIME) {
                pinpoint.update();
                Pose2D pose = pinpoint.getPosition();

                boolean currentPoseFinite = pose != null
                        && Double.isFinite(pose.getX(DistanceUnit.INCH))
                        && Double.isFinite(pose.getY(DistanceUnit.INCH))
                        && Double.isFinite(pose.getHeading(AngleUnit.RADIANS));

                if (currentPoseFinite) {
                    lastValidX = pose.getX(DistanceUnit.INCH);
                    lastValidY = pose.getY(DistanceUnit.INCH);
                    lastValidHeading = pose.getHeading(AngleUnit.RADIANS);
                    odometryValid = true;
                    odometryStaleTimer.reset();
                    hasReceivedValidPose = true;
                } else if (hasReceivedValidPose && odometryStaleTimer.seconds() > AutoConfig.ODOMETRY_STALE_SEC) {
                    odometryValid = false;
                    enterState(AutoState.FAULT_STOP);
                }

                if (!hasReceivedValidPose) {
                    stopAll();
                    telemetry.addData("Status", "Waiting for a valid Pinpoint pose");
                    telemetry.update();
                    idle();
                    continue;
                }

                double currentRuntime = runtime.seconds();
                double remaining = AutoConfig.MAX_AUTO_RUNTIME - currentRuntime;

                double dt = Math.min(0.1, currentRuntime - lastLoopTime);
                lastLoopTime = currentRuntime;
                double stateElapsed = stateTimer.seconds();

                switch (currentState) {
                    case AIM_AND_SPINUP: {
                        limelight.prepareForAiming();

                        lastShotRPM = shooter.updateAutoAim(
                                lastValidX, lastValidY, lastValidHeading, activeTargetX, activeTargetY);
                        shooter.runFlywheels(lastShotRPM);
                        transfer.stop();

                        boolean shooterReady = shooter.isReadyStable(
                                AutoConfig.AIM_READY_TOL_RPM, AutoConfig.AIM_STABLE_SEC, currentRuntime);
                        boolean turretAimable = shooter.isTurretAimable();
                        boolean settled = shooter.isTurretSettled(AutoConfig.TURRET_SETTLING_DELAY_SEC);

                        if (shooterReady && turretAimable && settled) {
                            enterState(AutoState.FEED_TARGET);
                        } else if (stateElapsed > AutoConfig.AIM_TIMEOUT_SEC) {
                            skipReason = "Cycle " + currentCycle + " aim timeout (rpmReady=" + shooterReady
                                    + ", aimable=" + turretAimable + ", settled=" + settled + ")";
                            enterState(finishShotAttempt(remaining));
                        }
                    }
                    break;

                    case FEED_TARGET: {
                        limelight.prepareForAiming();

                        lastShotRPM = shooter.updateAutoAim(
                                lastValidX, lastValidY, lastValidHeading, activeTargetX, activeTargetY);
                        shooter.runFlywheels(lastShotRPM);

                        boolean okToFeed = shooter.isReady(AutoConfig.FEED_READY_TOL_RPM) && shooter.isTurretAimable();
                        if (okToFeed) {
                            transfer.feed();
                            feedTimeAccum += dt;
                        } else {
                            transfer.stop();
                        }

                        if (feedTimeAccum >= AutoConfig.FEED_TIME_SEC || stateElapsed > AutoConfig.FEED_MAX_STATE_SEC) {
                            transfer.stop();
                            if (feedTimeAccum < AutoConfig.FEED_TIME_SEC) {
                                skipReason = "Cycle " + currentCycle + " under-fed ("
                                        + String.format(Locale.US, "%.2f", feedTimeAccum) + "s of "
                                        + AutoConfig.FEED_TIME_SEC + "s)";
                            }
                            enterState(finishShotAttempt(remaining));
                        }
                    }
                    break;

                    case SWEEP_COLLECT: {
                        limelight.prepareForIntake();

                        shooter.runFlywheels(lastShotRPM);
                        transfer.stop();

                        if (!AutoConfig.SELECTIVE_INTAKE || limelight.hasValidAllianceNectarTarget()) {
                            intake.runActive(currentRuntime);
                        } else {
                            intake.stop();
                        }

                        double sweepTargetY = AutoConfig.SWEEP_BASE_Y + currentCycle * AutoConfig.SWEEP_STEP_Y;
                        driveToPose(AutoConfig.SWEEP_X, sweepTargetY, AutoConfig.START_HEADING,
                                lastValidX, lastValidY, lastValidHeading, AutoConfig.SWEEP_MAX_POWER);

                        if (stateElapsed > AutoConfig.SWEEP_TIMEOUT_SEC
                                || lastValidY >= sweepTargetY - AutoConfig.POSITION_TOL_IN) {
                            intake.stop();
                            drive.stop();
                            retractTargetY = lastValidY - AutoConfig.RETRACT_DISTANCE_IN;
                            enterState(AutoState.RETRACT_BACK);
                        }
                    }
                    break;

                    case RETRACT_BACK: {
                        limelight.prepareForAiming();
                        lastShotRPM = shooter.updateAutoAim(
                                lastValidX, lastValidY, lastValidHeading, activeTargetX, activeTargetY);
                        shooter.runFlywheels(lastShotRPM);

                        driveToPose(AutoConfig.SWEEP_X, retractTargetY, AutoConfig.START_HEADING,
                                lastValidX, lastValidY, lastValidHeading, AutoConfig.RETRACT_MAX_POWER);

                        if (stateElapsed > AutoConfig.RETRACT_TIMEOUT_SEC
                                || lastValidY <= retractTargetY + AutoConfig.POSITION_TOL_IN) {
                            drive.stop();
                            enterState(AutoState.AIM_AND_SPINUP);
                        }
                    }
                    break;

                    case PARK: {
                        shooter.stopFlywheels();
                        transfer.stop();
                        intake.stop();

                        driveToPose(AutoConfig.PARK_X, AutoConfig.PARK_Y, AutoConfig.START_HEADING,
                                lastValidX, lastValidY, lastValidHeading, AutoConfig.PARK_MAX_POWER);

                        double parkDist = Math.hypot(AutoConfig.PARK_X - lastValidX, AutoConfig.PARK_Y - lastValidY);

                        if (stateElapsed > AutoConfig.PARK_TIMEOUT_SEC || parkDist < AutoConfig.POSITION_TOL_IN) {
                            drive.stop();
                            enterState(AutoState.IDLE);
                        }
                    }
                    break;

                    case FAULT_STOP:
                    case IDLE:
                    default: {
                        stopAll();
                    }
                    break;
                }

                telemetry.addData("Seesaw", seesaw + (seesawConfirmed ? " (confirmed)" : " (DEFAULT - NOT CONFIRMED)"));
                telemetry.addData("Target (X, Y)", "%.1f, %.1f", activeTargetX, activeTargetY);
                telemetry.addData("Cycle", currentCycle + "/" + AutoConfig.MAX_CYCLES);
                telemetry.addData("State", currentState);
                telemetry.addData("State Time (s)", "%.2f", stateElapsed);
                telemetry.addData("Time left (s)", "%.1f", remaining);
                telemetry.addData("Flywheel RPM", "%.0f / %.0f", shooter.getCurrentRPM(), shooter.getCurrentTargetRPM());
                telemetry.addData("Pose", "%.1f, %.1f, %.2f", lastValidX, lastValidY, lastValidHeading);
                telemetry.addData("Odometry valid", odometryValid);
                if (!skipReason.isEmpty()) telemetry.addData("Last skip/issue", skipReason);
                telemetry.update();
            }
        } finally {
            stopAll();
            limelight.stop();
        }
    }

    private void enterState(AutoState next) {
        currentState = next;
        stateTimer.reset();
        if (next == AutoState.FEED_TARGET) feedTimeAccum = 0.0;
        if (next == AutoState.AIM_AND_SPINUP) shooter.resetReadyState();
    }

    private AutoState finishShotAttempt(double remainingSec) {
        if (currentCycle >= AutoConfig.MAX_CYCLES) return AutoState.PARK;
        if (remainingSec < AutoConfig.MIN_TIME_FOR_CYCLE_SEC) {
            skipReason = "Not enough time for another cycle (" + String.format(Locale.US, "%.1f", remainingSec) + "s left)";
            return AutoState.PARK;
        }
        currentCycle++;
        return AutoState.SWEEP_COLLECT;
    }

    private void applySeesaw(SeesawState state) {
        seesaw = state;
        activeTargetX = (state == SeesawState.RIGHT_RAISED)
                ? AutoConfig.RIGHT_RAISED_TARGET_X
                : AutoConfig.LEFT_RAISED_TARGET_X;
        activeTargetY = AutoConfig.RAISED_TARGET_Y;
    }

    private void stopAll() {
        drive.stop();
        intake.stop();
        shooter.stopFlywheels();
        transfer.stop();
    }

    private void driveToPose(double targetX, double targetY, double targetHeading,
                             double x, double y, double heading, double maxPower) {
        double errX = targetX - x;
        double errY = targetY - y;

        double forwardErr = errX * Math.cos(heading) + errY * Math.sin(heading);
        double rightErr = errX * Math.sin(heading) - errY * Math.cos(heading);

        double forward = 0.0;
        if (Math.abs(forwardErr) > AutoConfig.POSITION_TOL_IN) {
            double mag = Math.min(maxPower, Math.max(AutoConfig.MIN_DRIVE_POWER, Math.abs(forwardErr) * AutoConfig.DRIVE_KP));
            forward = Math.copySign(mag, forwardErr);
        }

        double strafe = 0.0;
        if (Math.abs(rightErr) > AutoConfig.POSITION_TOL_IN) {
            double mag = Math.min(AutoConfig.MAX_LATERAL_POWER, Math.abs(rightErr) * AutoConfig.LATERAL_KP);
            strafe = AutoConfig.LATERAL_SIGN * Math.copySign(mag, rightErr);
        }

        double headingError = wrapAngle(targetHeading - heading);
        double turn = -headingError * AutoConfig.HEADING_KP;

        drive.driveRobotCentric(forward, strafe, turn, 1.0);
    }

    private static double wrapAngle(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }
}
*/