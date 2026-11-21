package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LimelightServoSubsystem;

@TeleOp(name = "Limelight Servo Test" )
public class LimelightServoTest extends OpMode {
    private LimelightServoSubsystem limelightServo;

    // How far each bumper press nudges the servo
    private static final double NUDGE_STEP = 0.05;

    // Previous bumper states, used to detect a single press
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    @Override
    public void init() {
        limelightServo = new LimelightServoSubsystem(hardwareMap);
        telemetry.addLine("Limelight servo test ready");
        telemetry.addLine("D-pad Up / Down: full up / full down");
        telemetry.addLine("Right / Left bumper: nudge up / down");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Full up / full down
        if (gamepad1.dpad_up) {
            limelightServo.servoUp();
        } else if (gamepad1.dpad_down) {
            limelightServo.servoDown();
        }

        // Small nudges, one step per press
        if (gamepad1.right_bumper && !lastRightBumper) {
            double target = limelightServo.getServoPosition() + NUDGE_STEP;
            limelightServo.moveToPosition(Math.min(1.0, target));
        }
        if (gamepad1.left_bumper && !lastLeftBumper) {
            double target = limelightServo.getServoPosition() - NUDGE_STEP;
            limelightServo.moveToPosition(Math.max(0.0, target));
        }
        lastRightBumper = gamepad1.right_bumper;
        lastLeftBumper = gamepad1.left_bumper;

        telemetry.addData("Commanded position", "%.2f", limelightServo.getServoPosition());
        telemetry.addData("isServoUp flag", limelightServo.isServoUp());
        telemetry.update();
    }
}