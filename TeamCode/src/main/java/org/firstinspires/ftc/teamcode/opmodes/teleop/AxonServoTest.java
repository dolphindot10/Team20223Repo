package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * Test OpMode for an Axon servo (standard/positional mode) with analog feedback.
 *
 * Config names:
 *   axon_servo  -> Servo
 *   axon_analog -> Analog Input
 *
 * Controls (gamepad1):
 *   A          -> go to 0.0
 *   B          -> go to 0.5
 *   Y          -> go to 1.0
 *   D-pad up   -> nudge +0.01
 *   D-pad down -> nudge -0.01
 *   Left stick Y -> fine control (hold to sweep)
 */
@TeleOp(name = "Axon Servo Test", group = "Test")
public class AxonServoTest extends LinearOpMode {

    private static final double MAX_VOLTAGE = 3.3; // analog feedback max voltage

    @Override
    public void runOpMode() {
        ServoImplEx servo = hardwareMap.get(ServoImplEx.class, "axon_servo");
        AnalogInput analog = hardwareMap.get(AnalogInput.class, "axon_analog");

        // Axon servos use the full 500-2500 us range
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        double target = 0.5;
        boolean upLast = false;
        boolean downLast = false;

        telemetry.addLine("Axon servo test ready. Press START.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Preset positions
            if (gamepad1.a) target = 0.0;
            if (gamepad1.b) target = 0.5;
            if (gamepad1.y) target = 1.0;

            // Nudge with d-pad (edge detected)
            if (gamepad1.dpad_up && !upLast) target += 0.01;
            if (gamepad1.dpad_down && !downLast) target -= 0.01;
            upLast = gamepad1.dpad_up;
            downLast = gamepad1.dpad_down;

            // Sweep with left stick
            target += -gamepad1.left_stick_y * 0.005;

            target = Math.max(0.0, Math.min(1.0, target));
            servo.setPosition(target);

            double voltage = analog.getVoltage();
            double degrees = voltage / MAX_VOLTAGE * 360.0;

            telemetry.addData("Target position", "%.3f", target);
            telemetry.addData("Analog voltage", "%.3f V", voltage);
            telemetry.addData("Analog angle", "%.1f deg", degrees);
            telemetry.update();
        }
    }
}