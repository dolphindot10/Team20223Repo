package org.firstinspires.ftc.teamcode.utils;

// numbers we might need to change go here
public class Constants {

    //VISION (Limelight)

    // pipeline 6 is the apriltag one
    public static final int VISION_PIPELINE = 6;

    // how fast it turns to face the tag. too high and it shakes
    // if it turns the wrong way make this negative
    public static final double AIM_KP = 0.02;

    // stop turning when its this close (degrees)
    public static final double AIM_DEADBAND_DEG = 1.0;

    // fastest it can turn while aiming
    public static final double AIM_MAX_POWER = 0.5;

    // give up aiming after this many seconds
    public static final double AIM_TIMEOUT_SEC = 2.0;

    //AIMING (pedro coordinates, inches)

    // spot the ball should fly through for each cell of the hive
    // placeholders = middle of the field. measure these on the real field
    public static final double RED_FAR_CELL_X = 72;
    public static final double RED_FAR_CELL_Y = 72;
    public static final double RED_AUDIENCE_CELL_X = 72;
    public static final double RED_AUDIENCE_CELL_Y = 72;
    public static final double BLUE_FAR_CELL_X = 72;
    public static final double BLUE_FAR_CELL_Y = 72;
    public static final double BLUE_AUDIENCE_CELL_X = 72;
    public static final double BLUE_AUDIENCE_CELL_Y = 72;

    // which cell is up when the match starts (manual 10.3.1)
    // the cell pointing at a flower starts down. double check on the field
    public static final boolean RED_FAR_CELL_STARTS_UP = false;
    public static final boolean BLUE_FAR_CELL_STARTS_UP = true;

    // where the turret spins compared to the middle of the robot
    // 0 = right in the middle. ashwin measures this from the CAD
    public static final double TURRET_FORWARD_IN = 0;
    public static final double TURRET_LEFT_IN = 0;

    // where you put the robot before INIT in the test opmodes (pedro coordinates)
    // pedro only knows where it is if this matches real life
    public static final double TEST_START_X = 72;
    public static final double TEST_START_Y = 72;
    public static final double TEST_START_HEADING_DEG = 0;

    //SHOOTER

    // placeholder config names, change once the robot is wired
    public static final String SHOOTER_MOTOR_1_NAME = "shooterMotor1";
    public static final String SHOOTER_MOTOR_2_NAME = "shooterMotor2";
    public static final String MOVER_SERVO_NAME = "flywheelMoverServo";
    public static final String HOOD_SERVO_NAME = "hoodServo";

    // goBILDA 5000 series, 28 ticks per motor turn
    public static final double SHOOTER_TICKS_PER_REV = 28;

    // motor turns per wheel turn. 1.0 = direct drive. ask ashwin
    public static final double FLYWHEEL_GEAR_RATIO = 1.0;

    // velocity PIDF. tune F first, then P, BEFORE filling the tables
    // F = 32767 / max ticks per sec (~2800)
    public static final double SHOOTER_P = 1.2;
    public static final double SHOOTER_I = 0;
    public static final double SHOOTER_D = 0;
    public static final double SHOOTER_F = 11.7;

    // ready = this close to the target for this many loops in a row
    public static final double SHOOTER_READY_RPM = 50;
    public static final int SHOOTER_READY_LOOPS = 3;

    // speed when we're not aiming. 0 = off
    public static final double IDLE_RPM = 0;

    // mover servo spots for each ball size. find by nudging in the tuning opmode
    public static final double MOVER_POLLEN_POS = 0.5;
    public static final double MOVER_NECTAR_POS = 0.5;

    // hood is moved by hand right now, so this stays off
    public static final boolean USE_HOOD = false;

    // the tables only work for the hood spot they were tuned at
    // if someone moves the hood, retune
    public static final String HOOD_SETTING_NOTE = "not tuned yet";

    // shooting tables. distance (inches) -> wheel RPM
    // placeholder numbers, fill these in with TeleOp_ShooterTuning
    // distances have to go smallest to biggest. keep RPM under ~5000
    public static final double[] POLLEN_DISTANCES = {36, 48, 60, 72, 96, 120};
    public static final double[] POLLEN_RPM = {2600, 2800, 3000, 3250, 3700, 4150};

    public static final double[] NECTAR_DISTANCES = {36, 48, 60, 72, 96, 120};
    public static final double[] NECTAR_RPM = {2600, 2800, 3000, 3250, 3700, 4150};

    // hood tables, only used if USE_HOOD is on. same distances as above
    public static final double[] POLLEN_HOOD = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    public static final double[] NECTAR_HOOD = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};

    //TURRET

    // placeholder config name
    public static final String TURRET_SERVO_NAME = "turretServo";

    // 5 turn servo wants the full 500-2500 pulse to use all 5 turns
    // double check on the goBILDA page for 2000-0025-0502
    public static final double TURRET_PWM_MIN_US = 500;
    public static final double TURRET_PWM_MAX_US = 2500;

    // servo spot where the turret points straight forward. find it in TeleOp_TurretTest
    public static final double TURRET_CENTER_POS = 0.5;

    // how much servo position moves the turret 1 degree
    // 5 turn servo = 1800 degrees over 0-1, gear 176/64 = 2.75
    // so 2.75 / 1800. check it in TeleOp_TurretTest
    public static final double TURRET_POS_PER_DEG = 2.75 / 1800.0;

    // furthest the turret can go each way (degrees) so wires dont wrap
    public static final double TURRET_MIN_DEG = -180;
    public static final double TURRET_MAX_DEG = 180;

    // servo cant tell us where it is, so we wait after moving
    // wait = min + this much per degree moved. guesses, time it for real
    public static final double TURRET_SETTLE_MS_PER_DEG = 5;
    public static final double TURRET_SETTLE_MIN_MS = 100;

    // moves smaller than this dont restart the wait (tracking makes tiny moves)
    public static final double TURRET_SETTLE_IGNORE_DEG = 2;
}
