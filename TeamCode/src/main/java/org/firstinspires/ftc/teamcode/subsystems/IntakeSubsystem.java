package org.firstinspires.ftc.teamcode.subsystems;

<<<<<<< Updated upstream
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem {

    private static final double TICKS_PER_REV = 537.6;

    private DcMotorEx intakeMotor1;
    private DcMotorEx intakeMotor2;

    private double currentPower = 0.0;

    public void init(HardwareMap hardwareMap) {
        // Initialize both intake motors from hardware map ("intake_motor_1" and "intake_motor_2")
        intakeMotor1 = hardwareMap.get(DcMotorEx.class, "intake_motor_1");
        intakeMotor2 = hardwareMap.get(DcMotorEx.class, "intake_motor_2");

        // Reverse motor 2 if mounted opposite to motor 1 so both pull/push in the same direction
        intakeMotor2.setDirection(DcMotorEx.Direction.REVERSE);

        // Set brake behavior on both motors so they stop simultaneously
        intakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set run mode on both motors simultaneously
        intakeMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Sets power to both intake motors simultaneously.
     *
     * @param power Power level between -1.0 and 1.0
     */
    public void setPower(double power) {
        this.currentPower = power;
        if (intakeMotor1 != null) {
            intakeMotor1.setPower(power);
        }
        if (intakeMotor2 != null) {
            intakeMotor2.setPower(power);
        }
    }

    /**
     * Stops both intake motors.
     */
    public void stop() {
        setPower(0.0);
    }

    /**
     * Gets the current target power set to the intake motors.
     */
    public double getPower() {
        return currentPower;
    }

    /**
     * Calculates and returns the current RPM of Intake Motor 1.
     */
    public double getMotor1RPM() {
        if (intakeMotor1 == null) return 0.0;
        return (intakeMotor1.getVelocity() / TICKS_PER_REV) * 60.0;
    }

    /**
     * Calculates and returns the current RPM of Intake Motor 2.
     */
    public double getMotor2RPM() {
        if (intakeMotor2 == null) return 0.0;
        return (intakeMotor2.getVelocity() / TICKS_PER_REV) * 60.0;
    }
}
=======
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
>>>>>>> Stashed changes
