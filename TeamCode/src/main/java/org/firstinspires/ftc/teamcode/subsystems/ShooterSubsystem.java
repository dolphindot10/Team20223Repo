package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterSubsystem {

    /** Tunables. Add @Config (FTC Dashboard) to this class if you want to tune live. */
    public static class Config {
        public static double TICKS_PER_REV = 28.0;

        // ---- Turret calibration (MEASURE THESE, don't trust the defaults) ----
        /** Servo position when the turret points straight ahead. */
        public static double TURRET_CENTER = 0.5;
        /** +1 or -1. Flip if a positive field angle moves the turret the wrong way. */
        public static double TURRET_DIRECTION = 1.0;
        /** Robot-relative angle (rad) that corresponds to TURRET_HALF_SPAN of servo travel. */
        public static double TURRET_RANGE_RAD = Math.PI / 2.0;
        /** Servo travel (in servo units) from center at TURRET_RANGE_RAD. */
        public static double TURRET_HALF_SPAN = 0.4;
        public static double TURRET_MIN_POS = 0.1;
        public static double TURRET_MAX_POS = 0.9;
        /** Direction the turret points at TURRET_CENTER, relative to robot forward (rad). */
        public static double TURRET_ZERO_OFFSET_RAD = 0.0;
        /** Turret pivot position relative to the Pinpoint tracking point, in inches. */
        public static double TURRET_OFFSET_FORWARD_IN = 0.0;
        public static double TURRET_OFFSET_RIGHT_IN = 0.0;
        /** Extra settle time per unit of commanded servo travel (servo units -> seconds). Measure it. */
        public static double TURRET_SEC_PER_SERVO_UNIT = 1.0;

        // ---- Flywheel ----
        public static double MIN_SHOT_RPM = 1500.0;
        public static double MAX_SHOT_RPM = 3500.0;
        /** A change in target bigger than this invalidates the "already ready" state. */
        public static double RETARGET_RESET_RPM = 100.0;
        /** If isReadyStable() wasn't polled for this long, its history is stale and is discarded. */
        public static double READY_CHECK_STALE_SEC = 0.25;

        /**
         * Optional custom velocity PIDF. OFF by default because the values depend on your motor.
         * Starting point: F ~= 32767 / (max ticks per second). For a 6000 RPM motor at 28 tick/rev that is
         * 2800 ticks/s -> F ~= 11.7. Raise P until RPM recovers quickly after a shot without oscillating.
         */
        public static boolean USE_CUSTOM_PIDF = false;
        public static double VEL_P = 10.0;
        public static double VEL_I = 3.0;
        public static double VEL_D = 0.0;
        public static double VEL_F = 11.7;
    }

    public static final double BASE_SHOOTER_RPM = 2000.0;

    // Distance (in) -> RPM. Valid ONLY at the hood position used when it was measured.
    private static final double[] LUT_DISTANCES = {50.0, 100.0, 150.0, 200.0};
    private static final double[] LUT_RPMS = {1800.0, 2150.0, 2700.0, 3400.0};

    private DcMotorEx shooterMotor;
    private Servo turretServo;
    private Servo hoodServo;
    private Servo flywheelShiftServo;

    private double currentTurretPos = Config.TURRET_CENTER;
    private double currentTargetRPM = 0.0;
    private boolean lastTargetAimable = false;

    private double readyStartTime = 0.0;
    private boolean wasReadyLastCheck = false;
    private double lastReadyCheckTime = -999.0;

    // Turret motion settling tracking
    private double lastTargetServoPos = Config.TURRET_CENTER;
    private double lastTurretTravel = 0.0;
    private final ElapsedTime turretSettlingTimer = new ElapsedTime();

    public void init(HardwareMap hwMap) {
        shooterMotor = hwMap.get(DcMotorEx.class, "shooterMotor");
        turretServo = hwMap.get(Servo.class, "turretServo");
        hoodServo = hwMap.get(Servo.class, "hoodServo");
        flywheelShiftServo = hwMap.get(Servo.class, "flywheelShiftServo");

        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        if (Config.USE_CUSTOM_PIDF) {
            shooterMotor.setPIDFCoefficients(
                    DcMotorEx.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(Config.VEL_P, Config.VEL_I, Config.VEL_D, Config.VEL_F));
        }

        hoodServo.setPosition(0.25);
        flywheelShiftServo.setPosition(0.1);
        turretServo.setPosition(Config.TURRET_CENTER);

        turretSettlingTimer.reset();
    }

    // ---------------------------------------------------------------------------------------
    // Flywheel
    // ---------------------------------------------------------------------------------------

    public void runFlywheels(double targetRPM) {
        if (Math.abs(targetRPM - currentTargetRPM) > Config.RETARGET_RESET_RPM) {
            resetReadyState();
        }
        currentTargetRPM = targetRPM;
        double targetTicksPerSec = (targetRPM * Config.TICKS_PER_REV) / 60.0;
        shooterMotor.setVelocity(targetTicksPerSec);
    }

    public void stopFlywheels() {
        if (shooterMotor != null) {
            shooterMotor.setVelocity(0.0);
        }
        currentTargetRPM = 0.0;
        resetReadyState();
    }

    /** Forget any "already at speed" history. Called automatically on big target changes and stale polls. */
    public void resetReadyState() {
        wasReadyLastCheck = false;
        readyStartTime = 0.0;
    }

    public double getCurrentRPM() {
        if (shooterMotor == null) return 0.0;
        return (shooterMotor.getVelocity() * 60.0) / Config.TICKS_PER_REV;
    }

    /** Stateless check: is the wheel within tolerance right now? No hysteresis, no history. */
    public boolean isReady(double toleranceRPM) {
        if (currentTargetRPM <= 0.0) return false;
        return Math.abs(getCurrentRPM() - currentTargetRPM) < toleranceRPM;
    }

    /**
     * Stateful check: within tolerance continuously for minStableSeconds. Once ready, tolerance widens
     * 1.5x so the flag doesn't flicker. Poll it every loop while you wait on it; if polling stops for a
     * while, the history is discarded so a stale "ready" can never carry over.
     */
    public boolean isReadyStable(double toleranceRPM, double minStableSeconds, double currentTimeSeconds) {
        if (currentTargetRPM <= 0.0) {
            resetReadyState();
            lastReadyCheckTime = currentTimeSeconds;
            return false;
        }

        if (currentTimeSeconds - lastReadyCheckTime > Config.READY_CHECK_STALE_SEC) {
            wasReadyLastCheck = false;
        }
        lastReadyCheckTime = currentTimeSeconds;

        double activeTolerance = wasReadyLastCheck ? (toleranceRPM * 1.5) : toleranceRPM;
        boolean currentlyWithinTolerance = Math.abs(getCurrentRPM() - currentTargetRPM) < activeTolerance;

        if (currentlyWithinTolerance) {
            if (!wasReadyLastCheck) {
                readyStartTime = currentTimeSeconds;
                wasReadyLastCheck = true;
            }
            return (currentTimeSeconds - readyStartTime) >= minStableSeconds;
        } else {
            wasReadyLastCheck = false;
            return false;
        }
    }

    // ---------------------------------------------------------------------------------------
    // Aiming
    // ---------------------------------------------------------------------------------------

    /**
     * Points the turret at (targetX, targetY) and returns the flywheel RPM for that distance.
     * Distance and angle are measured from the turret pivot, not the odometry point.
     */
    public double updateAutoAim(double robotX, double robotY, double botHeading, double targetX, double targetY) {
        double cosH = Math.cos(botHeading);
        double sinH = Math.sin(botHeading);

        // Robot forward in field frame = (cosH, sinH); robot right = (sinH, -cosH).
        double turretX = robotX + Config.TURRET_OFFSET_FORWARD_IN * cosH + Config.TURRET_OFFSET_RIGHT_IN * sinH;
        double turretY = robotY + Config.TURRET_OFFSET_FORWARD_IN * sinH - Config.TURRET_OFFSET_RIGHT_IN * cosH;

        double deltaX = targetX - turretX;
        double deltaY = targetY - turretY;
        double distanceToGoal = Math.hypot(deltaX, deltaY);

        double fieldTargetAngle = Math.atan2(deltaY, deltaX);
        double relativeAngle = fieldTargetAngle - botHeading - Config.TURRET_ZERO_OFFSET_RAD;
        relativeAngle = Math.atan2(Math.sin(relativeAngle), Math.cos(relativeAngle));

        double targetServoPos = Config.TURRET_CENTER
                + Config.TURRET_DIRECTION * (relativeAngle / Config.TURRET_RANGE_RAD) * Config.TURRET_HALF_SPAN;

        // Commanded aimability range check (does not imply physical sensor-based position feedback)
        lastTargetAimable = (targetServoPos >= Config.TURRET_MIN_POS && targetServoPos <= Config.TURRET_MAX_POS);

        commandTurret(targetServoPos);

        double targetRPM = calculateTargetRPM(distanceToGoal);
        return Math.max(Config.MIN_SHOT_RPM, Math.min(Config.MAX_SHOT_RPM, targetRPM));
    }

    public boolean isTurretAimable() {
        return lastTargetAimable;
    }

    /**
     * True once the turret command has been steady long enough. The required time grows with how far the
     * last command moved the servo, because a big swing takes longer than a nudge.
     */
    public boolean isTurretSettled(double baseSettlingTimeSec) {
        double required = baseSettlingTimeSec + lastTurretTravel * Config.TURRET_SEC_PER_SERVO_UNIT;
        return turretSettlingTimer.seconds() >= required;
    }

    public void setManualTurret(double stickX) {
        commandTurret(Config.TURRET_CENTER + (stickX * Config.TURRET_HALF_SPAN));
    }

    private void commandTurret(double requestedPos) {
        double pos = Math.max(Config.TURRET_MIN_POS, Math.min(Config.TURRET_MAX_POS, requestedPos));

        if (Math.abs(pos - lastTargetServoPos) > 0.005) {
            lastTurretTravel = Math.abs(pos - lastTargetServoPos);
            turretSettlingTimer.reset();
            lastTargetServoPos = pos;
        }

        currentTurretPos = pos;
        if (turretServo != null) {
            turretServo.setPosition(currentTurretPos);
        }
    }

    // ---------------------------------------------------------------------------------------
    // Misc
    // ---------------------------------------------------------------------------------------

    public void setHoodPosition(double position) {
        if (hoodServo != null) hoodServo.setPosition(position);
    }

    public void setFlywheelShift(double position) {
        if (flywheelShiftServo != null) flywheelShiftServo.setPosition(position);
    }

    public double getCurrentTargetRPM() {
        return currentTargetRPM;
    }

    public double getCurrentTurretPos() {
        return currentTurretPos;
    }

    private double calculateTargetRPM(double distanceInches) {
        int last = LUT_DISTANCES.length - 1;
        if (distanceInches <= LUT_DISTANCES[0]) return LUT_RPMS[0];
        if (distanceInches >= LUT_DISTANCES[last]) return LUT_RPMS[last];

        for (int i = 0; i < last; i++) {
            if (distanceInches >= LUT_DISTANCES[i] && distanceInches <= LUT_DISTANCES[i + 1]) {
                double proportion = (distanceInches - LUT_DISTANCES[i]) / (LUT_DISTANCES[i + 1] - LUT_DISTANCES[i]);
                return LUT_RPMS[i] + proportion * (LUT_RPMS[i + 1] - LUT_RPMS[i]);
            }
        }
        return BASE_SHOOTER_RPM;
    }
}