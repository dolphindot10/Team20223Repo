package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

import java.util.List;

@TeleOp(name = "Biobuzz Custom TeleOp Master", group = "Competition")
public class BiobuzzTeleop extends LinearOpMode {

    // Field coordinate definitions for TeleOp starting pose.
    // NOTE: these must be in the SAME coordinate system as TARGET_X/Y (and ideally the same as auto).
    private static final double START_X = 0.0;
    private static final double START_Y = 0.0;
    private static final double START_HEADING = 0.0;

    private static final double TARGET_X = 0.0;
    private static final double TARGET_Y = 72.0;

    // Shooter ready check
    private static final double READY_TOLERANCE_RPM = 150.0;
    private static final double READY_STABLE_SEC = 0.10;

    private boolean autoAimActive = false;
    private boolean lastBState = false;
    private boolean lastOptionsBackState = false;
    private boolean lastShooterReady = false;

    private final MecanumDriveSubsystem drivetrain = new MecanumDriveSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final TransferSubsystem transfer = new TransferSubsystem();

    @Override
    public void runOpMode() throws InterruptedException {

        // Bulk reads: one hub read per loop instead of one per sensor/encoder call. Faster loops.
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        drivetrain.init(hardwareMap);
        shooter.init(hardwareMap);
        intake.init(hardwareMap);
        transfer.init(hardwareMap);

        LimelightServoSubsystem limelightTilt = new LimelightServoSubsystem(hardwareMap);

        GoBildaPinpointDriver odoComputer = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odoComputer.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odoComputer.setOffsets(-120.0, 45.0, DistanceUnit.MM);   // VERIFY: must match your Pedro Constants
        odoComputer.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);  // VERIFY with the localization test

        // Reset once in init, while the robot is sitting still. (resetPosAndIMU already recalibrates the IMU.)
        odoComputer.resetPosAndIMU();

        telemetry.addData("Status", "Subsystems mapped. Keep the robot still until start.");
        telemetry.update();

        waitForStart();

        // Establish known starting field pose. No second IMU reset here: it takes time to finish
        // and can overwrite this setPosition, and the robot may already be moving.
        odoComputer.setPosition(new Pose2D(DistanceUnit.INCH, START_X, START_Y, AngleUnit.RADIANS, START_HEADING));

        ElapsedTime loopTimer = new ElapsedTime();

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();

            odoComputer.update();
            Pose2D currentPose = odoComputer.getPosition();
            double robotX = currentPose.getX(DistanceUnit.INCH);
            double robotY = currentPose.getY(DistanceUnit.INCH);
            double botHeading = currentPose.getHeading(AngleUnit.RADIANS);

            // DRIVER 1: FIELD-CENTRIC MECANUM DRIVE
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            // Heading reset: only resets the HEADING, keeps X/Y so auto-aim still knows where the robot is.
            boolean currentOptionsBackState = gamepad1.options || gamepad1.back;
            if (currentOptionsBackState && !lastOptionsBackState) {
                odoComputer.setPosition(new Pose2D(DistanceUnit.INCH, robotX, robotY, AngleUnit.RADIANS, START_HEADING));
                botHeading = START_HEADING;
                gamepad1.rumble(200);
            }
            lastOptionsBackState = currentOptionsBackState;

            double speedMultiplier = gamepad1.right_bumper ? 0.35 : 1.0;
            drivetrain.driveFieldCentric(forward, strafe, rotate, botHeading, speedMultiplier);

            // OPERATOR 2: INTAKE & SPINDEXER CYCLING
            if (gamepad2.left_trigger > 0.1) {
                intake.runActive(getRuntime());
            } else if (gamepad2.left_bumper) {
                intake.runReverse();
            } else {
                intake.stop();
            }

            // OPERATOR 2: ODOMETRY & TURRET AUTO-AIMING
            if (gamepad2.b && !lastBState) {
                autoAimActive = !autoAimActive;
            }
            lastBState = gamepad2.b;

            boolean aiming = autoAimActive || gamepad2.right_trigger > 0.1;
            double currentTargetRPM = ShooterSubsystem.BASE_SHOOTER_RPM;

            if (aiming) {
                currentTargetRPM = shooter.updateAutoAim(robotX, robotY, botHeading, TARGET_X, TARGET_Y);
                double distanceToGoal = Math.hypot(TARGET_X - robotX, TARGET_Y - robotY);
                telemetry.addData("Auto-Aim", "TRACKING | Dist: %.1fin | RPM: %.0f | %s",
                        distanceToGoal, currentTargetRPM,
                        shooter.isTurretAimable() ? "in range" : "OUT OF TURRET RANGE");
            } else {
                shooter.setManualTurret(gamepad2.right_stick_x);
                telemetry.addData("Auto-Aim", "MANUAL CONTROL");
            }

            // Hood + camera tilt. NOTE: the RPM lookup table is only valid at the hood position it was measured at.
            if (gamepad2.dpad_up) {
                shooter.setHoodPosition(0.75);
                limelightTilt.servoUp();
            } else if (gamepad2.dpad_down) {
                shooter.setHoodPosition(0.25);
                limelightTilt.servoDown();
            }

            // OPERATOR 2: FLYWHEEL VELOCITY & READY CHECK
            if (gamepad2.a) {
                shooter.runFlywheels(currentTargetRPM);
            } else {
                shooter.stopFlywheels();
            }

            // Stateful check (must be polled every loop): stable for READY_STABLE_SEC, with built-in
            // hysteresis so it doesn't flicker. When auto-aiming, the turret must also be able to reach the goal.
            boolean shooterReady = gamepad2.a
                    && shooter.isReadyStable(READY_TOLERANCE_RPM, READY_STABLE_SEC, getRuntime())
                    && (!aiming || shooter.isTurretAimable());

            if (shooterReady && !lastShooterReady) {
                gamepad2.rumble(150);
            }
            lastShooterReady = shooterReady;

            // OPERATOR 2: TRANSFER MOTOR & GATE SERVO INTERLOCK
            if (gamepad2.right_bumper && shooterReady) {
                transfer.feed();
            } else {
                transfer.stop();
            }

            // OPERATOR 2: FLYWHEEL GAP ADJUSTMENT
            if (gamepad2.x) {
                shooter.setFlywheelShift(0.9);
            } else if (gamepad2.y) {
                shooter.setFlywheelShift(0.1);
            }

            // TELEMETRY
            telemetry.addData("Shooter", shooterReady ? "READY TO FIRE" : (gamepad2.a ? "SPINNING UP..." : "OFF"));
            telemetry.addData("Shooter RPM", "%.0f (Target: %.0f)", shooter.getCurrentRPM(), shooter.getCurrentTargetRPM());
            telemetry.addData("Odometry", "X: %.1fin | Y: %.1fin | H: %.1f deg", robotX, robotY, Math.toDegrees(botHeading));
            telemetry.addData("Loop", "%.0f ms", loopTimer.milliseconds());
            loopTimer.reset();
            telemetry.update();
        }

        // Leave everything off when the OpMode ends.
        shooter.stopFlywheels();
        intake.stop();
        transfer.stop();
        drivetrain.stop();
    }
}
