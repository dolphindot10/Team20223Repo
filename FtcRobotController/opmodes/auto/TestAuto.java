package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Pedro Test Auto", group = "Autonomous")
public class TestAuto extends LinearOpMode {

    private Follower follower;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose point1    = new Pose(24, 0, Math.toRadians(0));
    private final Pose point2    = new Pose(24, 24, Math.toRadians(0));
    private final Pose endPose   = new Pose(0, 0, Math.toRadians(0));

    private PathChain testPath;

    public void buildPaths() {
        PathBuilder builder = follower.pathBuilder();

        builder
                .addPath(new BezierLine(startPose, point1))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .addPath(new BezierLine(point1, point2))
                .setConstantHeadingInterpolation(point1.getHeading())
                .addPath(new BezierLine(point2, endPose))
                .setConstantHeadingInterpolation(point2.getHeading());

        testPath = builder.build();
    }

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        telemetry.addData("Status", "Initialized. Ready to start.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        follower.followPath(testPath, true);

        while (opModeIsActive() && !isStopRequested()) {
            follower.update();

            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading (Deg)", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("Is Busy", follower.isBusy());
            telemetry.update();
        }
    }
}