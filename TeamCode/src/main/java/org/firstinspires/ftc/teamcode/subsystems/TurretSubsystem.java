package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.utils.Constants;

// the turret. turns an angle (degrees, 0 = straight ahead, + = left) into a servo spot
public class TurretSubsystem {

    private ServoImplEx servo;

    private double targetDeg = 0;
    private boolean reachable = true;

    // when the turret should be done moving (System.currentTimeMillis)
    private long settledAtMs = 0;

    public void init(HardwareMap hwMap) {
        servo = hwMap.get(ServoImplEx.class, Constants.TURRET_SERVO_NAME);

        // 5 turn servo only uses all 5 turns with the wider pulse range
        servo.setPwmRange(new PwmControl.PwmRange(Constants.TURRET_PWM_MIN_US, Constants.TURRET_PWM_MAX_US));
    }

    // point the turret at this angle
    public void setAngle(double deg) {
        // cant go past the limits or the wires wrap. go to the limit instead
        reachable = deg >= Constants.TURRET_MIN_DEG && deg <= Constants.TURRET_MAX_DEG;
        double clamped = Math.max(Constants.TURRET_MIN_DEG, Math.min(Constants.TURRET_MAX_DEG, deg));

        // bigger move = wait longer. tiny tracking moves dont restart the wait
        double moved = Math.abs(clamped - targetDeg);
        if (moved > Constants.TURRET_SETTLE_IGNORE_DEG) {
            long waitMs = (long) (Constants.TURRET_SETTLE_MIN_MS + moved * Constants.TURRET_SETTLE_MS_PER_DEG);
            settledAtMs = System.currentTimeMillis() + waitMs;
        }

        targetDeg = clamped;
        servo.setPosition(Constants.TURRET_CENTER_POS + clamped * Constants.TURRET_POS_PER_DEG);
    }

    // straight ahead
    public void center() {
        setAngle(0);
    }

    // put the servo anywhere. only for finding TURRET_CENTER_POS
    public void setServoPosition(double position) {
        servo.setPosition(position);
    }

    // where we last told the servo to go (0 to 1)
    public double getServoPosition() {
        return servo.getPosition();
    }

    public double getTargetDeg() {
        return targetDeg;
    }

    // false if the hive is past the turret limits. driver has to turn the robot
    public boolean isReachable() {
        return reachable;
    }

    // true once we've waited long enough for the last move
    public boolean isSettled() {
        return System.currentTimeMillis() >= settledAtMs;
    }
}
