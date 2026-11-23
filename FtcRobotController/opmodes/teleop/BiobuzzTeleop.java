/*
package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightServoSubsystem;

@TeleOp(name = "Biobuzz Custom TeleOp Master", group = "Competition")
public class BiobuzzTeleop extends LinearOpMode {

    // Field coordinate definitions for TeleOp starting pose
    private static final double START_X = 0.0;
    private static final double START_Y = 0.0;
    private static final double START_HEADING = 0.0;

    private static final double TARGET_X = 0.0;
    private static final double TARGET_Y = 72.0;

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

        drivetrain.init(hardwareMap);
        shooter.init(hardwareMap);
        intake.init(hardwareMap);
        transfer.init(hardwareMap);

        LimelightServoSubsystem limelightTilt = new LimelightServoSubsystem(hardwareMap);

        GoBildaPinpointDriver odoComputer = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odoComputer.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odoComputer.setOffsets(-120.0, 45.0, DistanceUnit.MM);
        odoComputer.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odoComputer.recalibrateIMU();
        odoComputer.resetPosAndIMU();

        telemetry.addData("Status", "Fully Modularized Subsystems Mapped & Odometry Ready");
        telemetry.update();

        waitForStart();
        odoComputer.resetPosAndIMU();

        // Establish known starting field pose.
        odoComputer.setPosition(new Pose2D(DistanceUnit.INCH, START_X, START_Y, AngleUnit.RADIANS, START_HEADING));

        while (opModeIsActive()) {

            odoComputer.update();
            Pose2D currentPose = odoComputer.getPosition();

            double robotX = currentPose != null ? currentPose.getX(DistanceUnit.INCH) : 0.0;
            double robotY = currentPose != null ? currentPose.getY(DistanceUnit.INCH) : 0.0;
            double botHeading = currentPose != null ? currentPose.getHeading(AngleUnit.RADIANS) : 0.0;

            // DRIVER 1: FIELD-CENTRIC MECANUM DRIVE
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            boolean currentOptionsBackState = gamepad1.options || gamepad1.back;
            if (currentOptionsBackState && !lastOptionsBackState) {
                odoComputer.resetPosAndIMU();
                odoComputer.setPosition(new Pose2D(DistanceUnit.INCH, START_X, START_Y, AngleUnit.RADIANS, START_HEADING));
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

            double currentTargetRPM = ShooterSubsystem.BASE_SHOOTER_RPM;

            if (autoAimActive || gamepad2.right_trigger > 0.1) {
                currentTargetRPM = shooter.updateAutoAim(robotX, robotY, botHeading, TARGET_X, TARGET_Y);

                double distanceToGoal = Math.hypot(TARGET_X - robotX, TARGET_Y - robotY);

                telemetry.addData("Auto-Aim Status", "TRACKING (LUT) | Dist: %.1fin | RPM: %.0f", distanceToGoal, currentTargetRPM);
            } else {
                shooter.setManualTurret(gamepad2.right_stick_x);

                if (gamepad2.dpad_up) {
                    shooter.setHoodPosition(0.75);
                    limelightTilt.servoUp();
                } else if (gamepad2.dpad_down) {
                    shooter.setHoodPosition(0.25);
                    limelightTilt.servoDown();
                }

                telemetry.addData("Auto-Aim Status", autoAimActive ? "SEARCHING..." : "MANUAL CONTROL");
            }

            // OPERATOR 2: FLYWHEEL VELOCITY & READY CHECK
            if (gamepad2.a) {
                shooter.runFlywheels(currentTargetRPM);
            } else {
                shooter.stopFlywheels();
            }

            double currentRPM = shooter.getCurrentRPM();
            boolean shooterReady = gamepad2.a && shooter.isReady(150.0);

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
            telemetry.addData("Shooter Status", shooterReady ? "READY TO FIRE" : "SPINNING UP...");
            telemetry.addData("Shooter RPM", "%.1f RPM (Target: %.0f)", currentRPM, currentTargetRPM);
            telemetry.addData("Odometry Position", "X: %.1fin | Y: %.1fin", robotX, robotY);
            telemetry.addData("Pinpoint Heading", "%.1f Deg", Math.toDegrees(botHeading));
            telemetry.update();
        }
    }
}
*/