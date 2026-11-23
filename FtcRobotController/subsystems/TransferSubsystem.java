package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TransferSubsystem {

    private DcMotorEx transferMotor;
    private Servo transferServo;

    private static final double GATE_CLOSED_POS = 0.15;
    private static final double GATE_OPEN_POS = 0.85;

    public void init(HardwareMap hwMap) {
        transferMotor = hwMap.get(DcMotorEx.class, "transferMotor");
        transferServo = hwMap.get(Servo.class, "transferServo");

        if (transferMotor != null) {
            transferMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        if (transferServo != null) {
            transferServo.setPosition(GATE_CLOSED_POS);
        }
    }

    public void feed() {
        if (transferMotor != null) {
            transferMotor.setPower(0.9);
        }
        if (transferServo != null) {
            transferServo.setPosition(GATE_OPEN_POS);
        }
    }

    public void stop() {
        if (transferMotor != null) {
            transferMotor.setPower(0.0);
        }
        if (transferServo != null) {
            transferServo.setPosition(GATE_CLOSED_POS);
        }
    }
}