package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem {
    private DcMotorEx intakeMotor;
    //private Servo spindexerServol;
    //private Servo spindexerServo2;

    public void init(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotorEx.class, "intakeMotor");
        //spindexerServol = hwMap.get(Servo.class, "spindexerServol");
        //spindexerServo2 = hwMap.get(Servo.class, "spindexerServo2");

        if (intakeMotor != null) {
            intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }
        //if (spindexerServol != null) spindexerServol.setPosition(0.5);
        //if (spindexerServo2 != null) spindexerServo2.setPosition(0.5);
    }

    public void runActive(double runtimeSeconds) {
        if (intakeMotor != null) {
            intakeMotor.setPower(1.0);
        }
        double pos1 = 0.5 + 0.2 * Math.sin(runtimeSeconds * 5.0);
        double pos2 = 0.5 + 0.2 * Math.cos(runtimeSeconds * 5.0);
        //if (spindexerServol != null) spindexerServol.setPosition(pos1);
        //if (spindexerServo2 != null) spindexerServo2.setPosition(pos2);
    }

    public void runReverse() {
        if (intakeMotor != null) {
            intakeMotor.setPower(-0.8);
        }
        //if (spindexerServol != null) spindexerServol.setPosition(0.5);
        //if (spindexerServo2 != null) spindexerServo2.setPosition(0.5);
    }

    /**
     * Drives intake motor at custom power while keeping spindexer servos centered.
     */
    public void runPower(double power) {
        if (intakeMotor != null) {
            intakeMotor.setPower(power);
        }
        //if (spindexerServol != null) spindexerServol.setPosition(0.5);
        //if (spindexerServo2 != null) spindexerServo2.setPosition(0.5);
    }

    public void stop() {
        if (intakeMotor != null) {
            intakeMotor.setPower(0.0);
        }
        //if (spindexerServol != null) spindexerServol.setPosition(0.5);
        //if (spindexerServo2 != null) spindexerServo2.setPosition(0.5);
    }
}