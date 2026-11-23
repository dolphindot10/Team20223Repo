package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.utils.AimingHelper;
import org.firstinspires.ftc.teamcode.utils.Constants;

// for filling in the shooting tables. robot doesnt drive, push it or park it
// INIT: X = blue, B = red
// dpad up/down = rpm +-50, B = stop wheel, A = pollen/nectar
// dpad left/right = nudge mover (to find MOVER_POLLEN_POS / MOVER_NECTAR_POS)
// bumpers = manual distance +-6in (used when pedro isnt set up), Y = hive tipped
@TeleOp(name = "TeleOp_ShooterTuning", group = "Test")
public class TeleOp_ShooterTuning extends OpMode {

    ShooterSubsystem shooter = new ShooterSubsystem();
    AimingHelper aiming = new AimingHelper(false);

    // null until arya finishes pedro setup
    Follower follower;

    double rpm = 0;
    double manualDistance = 60;
    double moverPos = Constants.MOVER_POLLEN_POS;

    ElapsedTime loopTimer = new ElapsedTime();

    // last loop's buttons so one press = one step
    boolean lastUp, lastDown, lastLeft, lastRight, lastA, lastB, lastY, lastLB, lastRB;

    @Override
    public void init() {
        shooter.init(hardwareMap);

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
        telemetry.addData("Pedro", follower == null ? "not set up, using manual distance" : "ok");
        telemetry.update();
    }

    @Override
    public void loop() {
        // speed
        if (gamepad1.dpad_up && !lastUp) rpm += 50;
        if (gamepad1.dpad_down && !lastDown) rpm = Math.max(0, rpm - 50);
        if (gamepad1.b && !lastB) rpm = 0;

        // ball type. also moves the mover to that ball's spot
        if (gamepad1.a && !lastA) {
            shooter.setNectar(!shooter.isNectar());
            moverPos = shooter.isNectar() ? Constants.MOVER_NECTAR_POS : Constants.MOVER_POLLEN_POS;
        }

        // nudge the mover to find its spots
        if (gamepad1.dpad_right && !lastRight) moverPos = Math.min(1, moverPos + 0.01);
        if (gamepad1.dpad_left && !lastLeft) moverPos = Math.max(0, moverPos - 0.01);
        shooter.setMoverPosition(moverPos);

        // manual distance for when pedro isnt working. measure with a tape
        if (gamepad1.right_bumper && !lastRB) manualDistance += 6;
        if (gamepad1.left_bumper && !lastLB) manualDistance = Math.max(0, manualDistance - 6);

        if (gamepad1.y && !lastY) aiming.hiveTipped();

        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastLeft = gamepad1.dpad_left;
        lastRight = gamepad1.dpad_right;
        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastY = gamepad1.y;
        lastLB = gamepad1.left_bumper;
        lastRB = gamepad1.right_bumper;

        // distance from pedro if it works, otherwise the manual one
        double distance = manualDistance;
        if (follower != null) {
            follower.update();
            distance = aiming.getDistance(follower.pose());
        }

        shooter.setRpm(rpm);
        shooter.update();

        // this is the row you write into Constants once 4/5 shots go in
        telemetry.addData("Distance (in)", "%.1f %s", distance, follower == null ? "(manual)" : "(pedro)");
        telemetry.addData("Ball", shooter.isNectar() ? "NECTAR" : "POLLEN");
        telemetry.addData("Target RPM", rpm);
        telemetry.addData("Actual RPM", "%.0f", shooter.getActualRpm());
        telemetry.addData("Ready", shooter.isReady());
        telemetry.addData("Mover pos", "%.2f", moverPos);
        telemetry.addData("Hood setting", Constants.HOOD_SETTING_NOTE);
        telemetry.addData("Battery (V)", "%.2f", hardwareMap.voltageSensor.iterator().next().getVoltage());

        // what the table says right now, to compare
        telemetry.addData("Table RPM", "%.0f %s", shooter.getTableRpm(distance),
                shooter.inRange(distance) ? "" : "(OUT OF RANGE)");
        telemetry.addData("Cell up", aiming.isFarCellUp() ? "far" : "audience");
        telemetry.addData("Loop (ms)", "%.1f", loopTimer.milliseconds());
        loopTimer.reset();
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.stop();
    }
}
