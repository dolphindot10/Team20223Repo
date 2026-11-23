package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

@TeleOp(name = "Basic Drive & Intake TeleOp", group = "TeleOp")
public class BasicDriveIntakeTeleOp extends LinearOpMode {

    private final MecanumDriveSubsystem drive = new MecanumDriveSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();

    @Override
    public void runOpMode() {
        drive.init(hardwareMap);
        intake.init(hardwareMap);

        telemetry.addData("Status", "Initialized and Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = -gamepad1.right_stick_x; // Inverted to fix reversed turning

            drive.driveRobotCentric(forward, strafe, rotate, 1.0);

            // Track intake power in a variable so telemetry matches actual power
            double intakePower = 0.0;
            if (gamepad1.right_bumper) {
                intakePower = 1.0;
            } else if (gamepad1.left_bumper) {
                intakePower = -1.0;
            }

            if (intakePower != 0.0) {
                intake.runPower(intakePower);
            } else {
                intake.stop();
            }

            telemetry.addData("Drive Powers", "Fwd: %.2f | Str: %.2f | Rot: %.2f", forward, strafe, rotate);
            telemetry.addData("Intake Status", intakePower != 0.0 ? String.format("%.2f Power", intakePower) : "Stopped");
            telemetry.update();
        }
    }
}