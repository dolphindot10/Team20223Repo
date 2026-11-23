package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.utils.AimingHelper;
import org.firstinspires.ftc.teamcode.utils.Constants;

// testing the turret. robot doesnt drive, push it around by hand
// INIT: X = blue, B = red. turret goes to center
// A = switch manual / tracking, Y = hive tipped
// manual: dpad up/down = small nudge, dpad left/right = big nudge
//   use it to find TURRET_CENTER_POS and check TURRET_POS_PER_DEG with a protractor
// tracking: turret follows the hive using pedro (needs pedro set up)
@TeleOp(name = "TeleOp_TurretTest", group = "Test")
public class TeleOp_TurretTest extends OpMode {

    TurretSubsystem turret = new TurretSubsystem();
    AimingHelper aiming = new AimingHelper(false);

    // null until arya finishes pedro setup
    Follower follower;

    boolean tracking = false;
    double manualPos = Constants.TURRET_CENTER_POS;

    ElapsedTime loopTimer = new ElapsedTime();

    // last loop's buttons so one press = one step
    boolean lastUp, lastDown, lastLeft, lastRight, lastA, lastY;

    @Override
    public void init() {
        turret.init(hardwareMap);

        // straight ahead so we fit in the 18in box
        turret.center();

        // read all sensors in one go each loop, way faster
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        follower = org.firstinspires.ftc.teamcode.pedro.Constants.create(hardwareMap);
        if (follower != null) {
            follower.setPose(new Pose(Constants.TEST_START_X, Constants.TEST_START_Y,
                    Math.toRadians(Constants.TEST_START_HEADING_DEG)));
        }
    }

    @Override
    public void init_loop() {
        // pick alliance
        if (gamepad1.x) aiming.setAlliance(false);
        if (gamepad1.b) aiming.setAlliance(true);
        telemetry.addData("Alliance", aiming.isRed() ? "RED" : "BLUE");
        telemetry.addData("Pedro", follower == null ? "not set up, manual mode only" : "ok");
        telemetry.update();
    }

    @Override
    public void loop() {
        // switch modes. tracking only works if pedro is set up
        if (gamepad1.a && !lastA && follower != null) {
            tracking = !tracking;
            manualPos = turret.getServoPosition();
        }
        if (gamepad1.y && !lastY) aiming.hiveTipped();

        if (follower != null) follower.update();

        if (tracking) {
            // point at our up cell
            Pose pose = follower.pose();
            turret.setAngle(aiming.getTurretAngle(pose));
            telemetry.addData("Distance (in)", "%.1f", aiming.getDistance(pose));
            telemetry.addData("Robot", "x %.1f y %.1f heading %.1f",
                    pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        } else {
            // move the servo by hand
            if (gamepad1.dpad_up && !lastUp) manualPos += 0.002;
            if (gamepad1.dpad_down && !lastDown) manualPos -= 0.002;
            if (gamepad1.dpad_right && !lastRight) manualPos += 0.02;
            if (gamepad1.dpad_left && !lastLeft) manualPos -= 0.02;
            manualPos = Math.max(0, Math.min(1, manualPos));
            turret.setServoPosition(manualPos);

            // if TURRET_POS_PER_DEG is right, this matches the protractor
            double deg = (manualPos - Constants.TURRET_CENTER_POS) / Constants.TURRET_POS_PER_DEG;
            telemetry.addData("Servo pos", "%.3f", manualPos);
            telemetry.addData("Should be at (deg)", "%.1f", deg);
        }

        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastLeft = gamepad1.dpad_left;
        lastRight = gamepad1.dpad_right;
        lastA = gamepad1.a;
        lastY = gamepad1.y;

        telemetry.addData("Mode", tracking ? "TRACKING" : "MANUAL");
        telemetry.addData("Turret angle (deg)", "%.1f", turret.getTargetDeg());
        telemetry.addData("Can reach", turret.isReachable());
        telemetry.addData("Settled", turret.isSettled());
        telemetry.addData("Cell up", aiming.isFarCellUp() ? "far" : "audience");
        telemetry.addData("Loop (ms)", "%.1f", loopTimer.milliseconds());
        loopTimer.reset();
        telemetry.update();
    }
}
