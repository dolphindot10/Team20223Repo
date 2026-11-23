package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.LookupTable;

// flywheel (2 motors on one shaft), the mover servo, and the hood
// looks up how fast to spin from how far away the hive is
public class ShooterSubsystem {

    private DcMotorEx motor1;
    private DcMotorEx motor2;
    private Servo mover;
    private Servo hood; // stays null when USE_HOOD is off

    // one set of tables per ball size
    private LookupTable pollenRpm;
    private LookupTable nectarRpm;
    private LookupTable pollenHood;
    private LookupTable nectarHood;

    private boolean nectar = false; // false = pollen
    private double targetRpm = 0;
    private int readyLoops = 0;

    public void init(HardwareMap hwMap) {
        motor1 = hwMap.get(DcMotorEx.class, Constants.SHOOTER_MOTOR_1_NAME);
        motor2 = hwMap.get(DcMotorEx.class, Constants.SHOOTER_MOTOR_2_NAME);
        mover = hwMap.get(Servo.class, Constants.MOVER_SERVO_NAME);

        // motors face each other on the shaft so one spins backwards
        // if they fight on the bench, swap which one is reversed
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        // velocity mode needs the encoder plugged in on BOTH motors
        PIDFCoefficients pidf = new PIDFCoefficients(
                Constants.SHOOTER_P, Constants.SHOOTER_I, Constants.SHOOTER_D, Constants.SHOOTER_F);
        for (DcMotorEx m : new DcMotorEx[]{motor1, motor2}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // coast, dont brake a heavy wheel
        }

        if (Constants.USE_HOOD) {
            hood = hwMap.get(Servo.class, Constants.HOOD_SERVO_NAME);
        }

        // crashes right here if a table is set up wrong, not mid match
        pollenRpm = new LookupTable(Constants.POLLEN_DISTANCES, Constants.POLLEN_RPM);
        nectarRpm = new LookupTable(Constants.NECTAR_DISTANCES, Constants.NECTAR_RPM);
        pollenHood = new LookupTable(Constants.POLLEN_DISTANCES, Constants.POLLEN_HOOD);
        nectarHood = new LookupTable(Constants.NECTAR_DISTANCES, Constants.NECTAR_HOOD);

        setNectar(false);
    }

    // pick which ball we're shooting. moves the mover to fit it
    public void setNectar(boolean isNectar) {
        nectar = isNectar;
        mover.setPosition(isNectar ? Constants.MOVER_NECTAR_POS : Constants.MOVER_POLLEN_POS);
    }

    // move the mover anywhere. only for finding the pollen/nectar spots
    public void setMoverPosition(double position) {
        mover.setPosition(position);
    }

    public boolean isNectar() {
        return nectar;
    }

    // the rpm table for the ball we have right now
    private LookupTable rpmTable() {
        return nectar ? nectarRpm : pollenRpm;
    }

    // true if this distance is inside the table. outside it we dont shoot
    public boolean inRange(double distance) {
        return rpmTable().inRange(distance);
    }

    // what the table says for this distance (doesnt spin anything)
    public double getTableRpm(double distance) {
        return rpmTable().get(distance);
    }

    // spin up for a shot from this far away
    public void aimAt(double distance) {
        setRpm(rpmTable().get(distance));

        if (hood != null) {
            LookupTable hoodTable = nectar ? nectarHood : pollenHood;
            hood.setPosition(hoodTable.get(distance));
        }
    }

    // not shooting, go to idle speed
    public void idle() {
        setRpm(Constants.IDLE_RPM);
    }

    // spin both motors at this wheel rpm. used by aimAt and the tuning opmode
    public void setRpm(double rpm) {
        targetRpm = rpm;
        double ticksPerSec = rpmToTicks(rpm);
        motor1.setVelocity(ticksPerSec);
        motor2.setVelocity(ticksPerSec);
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    // how fast the wheel is actually going, average of both motors
    public double getActualRpm() {
        double avg = (Math.abs(motor1.getVelocity()) + Math.abs(motor2.getVelocity())) / 2.0;
        return ticksToRpm(avg);
    }

    // call this every loop so the ready check can count
    public void update() {
        boolean close = targetRpm > 0
                && Math.abs(getActualRpm() - targetRpm) <= Constants.SHOOTER_READY_RPM;
        readyLoops = close ? readyLoops + 1 : 0;
    }

    // true once the wheel has been at speed for a few loops in a row
    public boolean isReady() {
        return readyLoops >= Constants.SHOOTER_READY_LOOPS;
    }

    public void stop() {
        setRpm(0);
    }

    // wheel rpm -> motor ticks per second
    private double rpmToTicks(double rpm) {
        return rpm * Constants.FLYWHEEL_GEAR_RATIO * Constants.SHOOTER_TICKS_PER_REV / 60.0;
    }

    // motor ticks per second -> wheel rpm
    private double ticksToRpm(double ticksPerSec) {
        return ticksPerSec * 60.0 / Constants.SHOOTER_TICKS_PER_REV / Constants.FLYWHEEL_GEAR_RATIO;
    }
}
