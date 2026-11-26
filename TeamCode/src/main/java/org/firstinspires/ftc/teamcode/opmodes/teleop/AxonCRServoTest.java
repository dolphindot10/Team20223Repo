package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * Test OpMode for an Axon servo in CONTINUOUS ROTATION mode with analog feedback.
 *
 * Config names:
 *   axon_servo  -> Continuous Rotation Servo
 *   axon_analog -> Analog Input
 *
 * Controls (gamepad1):
 *   Left stick Y -> speed and direction (proportional)
 *   A            -> spin +0.3
 *   B            -> spin -0.3
 *   X            -> stop
 *   Y            -> reset total rotation counter to 0
 */
@TeleOp(name = "Axon CR Servo Test", group = "Test")
public class AxonCRServoTest extends LinearOpMode {

    private static final double MAX_VOLTAGE = 3.3; // analog feedback max voltage

    @Override
    public void runOpMode() {
        CRServo servo = hardwareMap.get(CRServo.class, "axon_servo");
        AnalogInput analog = hardwareMap.get(AnalogInput.class, "axon_analog");

        // Total rotation tracking (feedback wraps 0-360, so unwrap it)
        double lastAngle = analog.getVoltage() / MAX_VOLTAGE * 360.0;
        double totalRotation = 0;

        telemetry.addLine("Axon CR servo test ready. Press START.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double power = -gamepad1.left_stick_y;

            if (gamepad1.a) power = 0.3;
            if (gamepad1.b) power = -0.3;
            if (gamepad1.x) power = 0;

            servo.setPower(power);

            double voltage = analog.getVoltage();
            double angle = voltage / MAX_VOLTAGE * 360.0;

            // Unwrap across the 360 -> 0 boundary
            double delta = angle - lastAngle;
            if (delta > 180) delta -= 360;
            if (delta < -180) delta += 360;
            totalRotation += delta;
            lastAngle = angle;

            if (gamepad1.y) totalRotation = 0;

            telemetry.addData("Power", "%.2f", power);
            telemetry.addData("Analog voltage", "%.3f V", voltage);
            telemetry.addData("Angle (0-360)", "%.1f deg", angle);
            telemetry.addData("Total rotation", "%.1f deg (%.2f turns)",
                    totalRotation, totalRotation / 360.0);
            telemetry.update();
        }
    }
}