package org.firstinspires.ftc.teamcode.Opmodes.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Triangle Spin 0.1", group = "PedroPathTestCases")

public class TriangleSpin extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private PathChain trianglePath;

    public void buildPaths() {
        // Geometric calculations for a 36-inch Equilateral Triangle
        // Height = side * sqrt(3) / 2 ~= 31.177 inches
        double side = 48.0;
        double height = 31.177;

        // Poses for the triangle vertices
        Pose pointA = new Pose(0, 0, 0);
        Pose pointB = new Pose(side, 0, 0);
        Pose pointC = new Pose(side / 2.0, height, 0);

        trianglePath = follower.pathBuilder()
                // --- LEG 1: A to B ---
                // Move 36 inches, Spin 0 -> 180 degrees
                .addPath(new BezierLine(pointA, pointB))
                .setLinearHeadingInterpolation(0, Math.PI)

                // --- LEG 2: B to C ---
                // Move 36 inches, Spin 180 -> 360 degrees
                .addPath(new BezierLine(pointB, pointC))
                .setLinearHeadingInterpolation(Math.PI, 2 * Math.PI)

                // --- LEG 3: C to A ---
                // Move 36 inches, Spin 360 -> 540 degrees (finishes facing 180 relative to start)
                .addPath(new BezierLine(pointC, pointA))
                .setLinearHeadingInterpolation(2 * Math.PI, 3 * Math.PI)

                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start the path
                follower.followPath(trianglePath, true);
                setPathState(1);
                break;

            case 1:
                // Wait until the robot returns to Point A and finishes the spin
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        buildPaths();

        // Start at Point A with 0 heading
        follower.setStartingPose(new Pose(0, 0, 0));
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}







