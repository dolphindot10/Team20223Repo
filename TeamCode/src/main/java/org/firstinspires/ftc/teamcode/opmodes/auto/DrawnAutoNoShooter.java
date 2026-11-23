package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.pedro.PedroCommands.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;   // your Pedro Pathing 3 Constants (change the package if yours is elsewhere)
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Same path as DrawnAuto, but ONLY driving + intake (for testing before the shooter is built).
 * Every launch is replaced by a plain wait of LAUNCH_PLACEHOLDER_MS, so the timing stays close to the real auto.
 * Set LAUNCH_PLACEHOLDER_MS to 0 if you just want to test the driving quickly.
 */
@Autonomous(name = "Drawn Auto – Red (no shooter)", group = "BIOBUZZ")
public class DrawnAutoNoShooter extends OpMode {

    /** Stand-in for each launch: about how long the real launch takes (spin-up + 1600 ms feed). */
    private static final int LAUNCH_PLACEHOLDER_MS = 2000;

    private Follower follower;
    private IntakeSubsystem intake;

    private boolean intaking = false;
    private String step = "init";

    @Override public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(drawn0);
        follower.update();

        intake = new IntakeSubsystem();
        intake.init(hardwareMap);
    }

    @Override public void init_loop() {
        Pose p = follower.pose();
        telemetry.addData("Start pose", "x %.1f  y %.1f  heading %.0f",
                p.x(), p.y(), Math.toDegrees(p.heading()));
        telemetry.update();
    }

    @Override public void start() {
        schedule(drawnPath());
    }

    @Override public void loop() {
        follower.update();

        // The spindexer oscillation needs runActive() every loop.
        if (intaking) intake.runActive(getRuntime());

        Scheduler.execute();

        Pose p = follower.pose();
        telemetry.addData("Step", step);
        telemetry.addData("Pose", "x %.1f  y %.1f  heading %.0f", p.x(), p.y(), Math.toDegrees(p.heading()));
        telemetry.addData("Intake", intaking ? "ON" : "off");
        telemetry.update();
    }

    @Override public void stop() {
        intake.stop();
    }

    // Path copied from DrawnAuto. If you change the points there, copy them here too.
    private final PoseFactory drawnDeg = PoseFactory.degrees();
    private final Pose drawn0 = drawnDeg.of(60.0, 8.8, 90.0);   // start
    private final Pose drawn1 = drawnDeg.of(48.5, 127.5, 90.0);
    private final Pose drawn2 = drawnDeg.of(59.0, 122.0, -90.0);
    private final Pose drawn3 = drawnDeg.of(12.0, 105.5, -178.3);
    private final Pose drawn4 = drawnDeg.of(59.0, 121.5, -90.0);
    private final Pose drawn5 = drawnDeg.of(11.5, 109.0, -180.0);
    private final Pose drawn1c = drawnDeg.of(61.0, 121.0, 0);   // bends the line into drawn1
    private final Pose drawn2c = drawnDeg.of(46.8, 105.3, 0);   // bends the line into drawn2
    private final Pose drawn3c = drawnDeg.of(37.0, 106.3, 0);   // bends the line into drawn3

    private Command drawnPath() {
        return sequential(
                setStep("1: (launch preload - skipped)"),
                waitMs(LAUNCH_PLACEHOLDER_MS),

                setStep("2: drive to drawn1"),
                follow(follower, curve(drawn0, drawn1c, drawn1).constant(drawn0)),
                setStep("3: intake at drawn1"),
                intakeOn(),
                waitMs(2000),
                intakeOff(),

                setStep("4: drive to drawn2"),
                follow(follower, curve(drawn1, drawn2c, drawn2).linear(drawn1, drawn2)),
                setStep("5: (launch at drawn2 - skipped)"),
                waitMs(LAUNCH_PLACEHOLDER_MS),
                waitMs(500),

                setStep("6: intake while driving to drawn3"),
                intakeOn(),
                follow(follower, curve(drawn2, drawn3c, drawn3).tangent()),
                waitMs(2000),
                intakeOff(),

                setStep("7: drive to drawn4"),
                follow(follower, line(drawn3, drawn4).linear(drawn3, drawn4)),
                setStep("8: (launch at drawn4 - skipped)"),
                waitMs(LAUNCH_PLACEHOLDER_MS),

                setStep("9: park at drawn5"),
                follow(follower, line(drawn4, drawn5).linear(drawn4, drawn5)),
                setStep("done"));
    }

    private Command intakeOn() {
        return instant(() -> intaking = true);
    }

    private Command intakeOff() {
        return instant(() -> { intaking = false; intake.stop(); });
    }

    private Command setStep(String name) {
        return instant(() -> step = name);
    }
}