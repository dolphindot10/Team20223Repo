package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;
import java.util.Locale;

public class LimelightSubsystem {

    /** Tunables. Add @Config (FTC Dashboard) to this class if you want to tune live. */
    public static class Config {
        public static int LEFT_TAG_ID = 1;
        /** Set to a valid tag id to enable the more robust "compare both sides" mode. -1 = disabled. */
        public static int RIGHT_TAG_ID = -1;

        /** Which axis of the robot-space tag pose is "height": 0 = x, 1 = y, 2 = z. VERIFY on the robot. */
        public static int HEIGHT_AXIS = 2;

        /** Single-tag mode: left tag taller than this (inches) means LEFT is raised. */
        public static double RAISED_HEIGHT_THRESHOLD_INCHES = 24.0;
        /** Two-tag mode: heights must differ by at least this many inches to call a side. */
        public static double MIN_HEIGHT_DIFF_INCHES = 4.0;

        /** Number of distinct, agreeing camera frames needed to confirm a seesaw reading. */
        public static int CONFIRM_FRAMES = 10;

        /** Results older than this are ignored. */
        public static long MAX_STALENESS_MS = 100;

        /** Nectar must fill at least this percent of the image to count as a target. */
        public static double MIN_NECTAR_AREA_PERCENT = 0.5;
    }

    public enum AllianceColor {
        RED,
        BLUE
    }

    public enum SeesawState {
        LEFT_RAISED,
        RIGHT_RAISED,
        UNKNOWN
    }

    private enum Mode {
        AIMING,
        INTAKE
    }

    private static final int PIPELINE_APRILTAG_HIVE = 0;
    private static final int PIPELINE_COLOR_RED = 1;
    private static final int PIPELINE_COLOR_BLUE = 2;

    private final Limelight3A limelight;
    private final LimelightServoSubsystem tiltServo;

    private AllianceColor currentAlliance = AllianceColor.RED;

    private Mode currentMode = null;
    private int requestedPipeline = -1;

    // Seesaw voting state
    private SeesawState voteState = SeesawState.UNKNOWN;
    private int voteCount = 0;
    private double lastVoteTimestamp = -1.0;
    private double lastLeftHeightIn = Double.NaN;
    private double lastRightHeightIn = Double.NaN;

    public LimelightSubsystem(HardwareMap hardwareMap) {
        // hardwareMap.get() throws if the device is missing; it never returns null.
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        tiltServo = new LimelightServoSubsystem(hardwareMap);

        limelight.setPollRateHz(100);
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    public void setAlliance(AllianceColor alliance) {
        this.currentAlliance = alliance;
    }

    // ---------------------------------------------------------------------------------------
    // Mode / pipeline control (idempotent: safe to call every loop)
    // ---------------------------------------------------------------------------------------

    public void prepareForAiming() {
        if (currentMode != Mode.AIMING) {
            tiltServo.servoUp();
            currentMode = Mode.AIMING;
        }
        switchPipeline(PIPELINE_APRILTAG_HIVE);
    }

    public void prepareForIntake() {
        if (currentMode != Mode.INTAKE) {
            tiltServo.servoDown();
            currentMode = Mode.INTAKE;
        }
        switchPipeline(colorPipeline());
    }

    private int colorPipeline() {
        return (currentAlliance == AllianceColor.RED) ? PIPELINE_COLOR_RED : PIPELINE_COLOR_BLUE;
    }

    private void switchPipeline(int pipeline) {
        if (pipeline != requestedPipeline) {
            limelight.pipelineSwitch(pipeline);
            requestedPipeline = pipeline;
            // Anything read before the switch settles is not trustworthy for voting.
            voteState = SeesawState.UNKNOWN;
            voteCount = 0;
        }
    }

    /** A result is unusable if it is missing, invalid, from the wrong pipeline, or too old. */
    private boolean isStale(LLResult result, int expectedPipeline) {
        return result == null
                || !result.isValid()
                || result.getPipelineIndex() != expectedPipeline
                || result.getStaleness() >= Config.MAX_STALENESS_MS;
    }

    // ---------------------------------------------------------------------------------------
    // Seesaw detection
    // ---------------------------------------------------------------------------------------

    /** Single-frame reading. UNKNOWN means "no trustworthy information", never a guess. */
    public SeesawState readSeesawRaw() {
        LLResult result = limelight.getLatestResult();
        lastLeftHeightIn = Double.NaN;
        lastRightHeightIn = Double.NaN;

        if (isStale(result, PIPELINE_APRILTAG_HIVE)) return SeesawState.UNKNOWN;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null) return SeesawState.UNKNOWN;

        for (LLResultTypes.FiducialResult tag : tags) {
            int id = tag.getFiducialId();
            if (id == Config.LEFT_TAG_ID) {
                lastLeftHeightIn = heightInches(tag);
            } else if (Config.RIGHT_TAG_ID >= 0 && id == Config.RIGHT_TAG_ID) {
                lastRightHeightIn = heightInches(tag);
            }
        }

        boolean haveLeft = !Double.isNaN(lastLeftHeightIn);
        boolean haveRight = !Double.isNaN(lastRightHeightIn);

        if (haveLeft && haveRight) {
            // Preferred: compare the two sides against each other. Independent of mounting height.
            double diff = lastLeftHeightIn - lastRightHeightIn;
            if (Math.abs(diff) < Config.MIN_HEIGHT_DIFF_INCHES) return SeesawState.UNKNOWN;
            return diff > 0 ? SeesawState.LEFT_RAISED : SeesawState.RIGHT_RAISED;
        }
        if (haveLeft) {
            // Fallback: absolute threshold on the left tag only.
            return lastLeftHeightIn > Config.RAISED_HEIGHT_THRESHOLD_INCHES
                    ? SeesawState.LEFT_RAISED
                    : SeesawState.RIGHT_RAISED;
        }
        return SeesawState.UNKNOWN;
    }

    /**
     * Call once per init loop. Counts consecutive distinct camera frames that agree.
     * Repeat reads of the same frame are ignored so the loop rate cannot fake consensus.
     */
    public void updateSeesawVote() {
        LLResult result = limelight.getLatestResult();
        if (result == null) return;

        double ts = result.getTimestamp();
        if (ts == lastVoteTimestamp) return;
        lastVoteTimestamp = ts;

        SeesawState reading = readSeesawRaw();
        if (reading == SeesawState.UNKNOWN) return; // no info: neither confirms nor resets

        if (reading == voteState) {
            voteCount++;
        } else {
            voteState = reading;
            voteCount = 1;
        }
    }

    /** The confirmed state, or UNKNOWN if not enough agreeing frames have been seen yet. */
    public SeesawState getConfirmedSeesaw() {
        return voteCount >= Config.CONFIRM_FRAMES ? voteState : SeesawState.UNKNOWN;
    }

    public int getVoteCount() {
        return voteCount;
    }

    /** For init telemetry: the tag heights (inches) from the last raw read. Use this to verify the axis. */
    public String getHeightDebug() {
        return String.format(Locale.US, "L=%.1f in, R=%.1f in", lastLeftHeightIn, lastRightHeightIn);
    }

    private double heightInches(LLResultTypes.FiducialResult tag) {
        Position p = tag.getTargetPoseRobotSpace().getPosition();
        double raw;
        switch (Config.HEIGHT_AXIS) {
            case 0:
                raw = p.x;
                break;
            case 1:
                raw = p.y;
                break;
            default:
                raw = p.z;
                break;
        }
        // Position carries its own unit (the Limelight SDK reports meters). Never compare raw numbers.
        return p.unit.toInches(raw);
    }

    // ---------------------------------------------------------------------------------------
    // Nectar detection (color pipeline)
    // ---------------------------------------------------------------------------------------

    public boolean hasValidAllianceNectarTarget() {
        LLResult result = limelight.getLatestResult();
        if (isStale(result, colorPipeline())) return false;
        return result.getTa() > Config.MIN_NECTAR_AREA_PERCENT;
    }

    public LimelightServoSubsystem getTiltServo() {
        return tiltServo;
    }
}