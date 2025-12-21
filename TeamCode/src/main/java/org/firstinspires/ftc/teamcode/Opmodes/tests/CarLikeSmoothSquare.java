package org.firstinspires.ftc.teamcode.Opmodes.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "CarLike Smooth Square 0.1", group = "PedroPathTestCases")
public class CarLikeSmoothSquare extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    private PathChain squarePath;

    /** * Builds a square where the robot's nose always points forward.
     * Includes a radius 'r' to allow the robot to maintain speed through corners.
     **/
    public void buildPaths() {
        int side = 24;
        int r = 6; // Increase for smoother/faster turns, decrease for sharper ones.

        squarePath = follower.pathBuilder()
                // SIDE 1
                .addPath(new BezierLine(new Pose(0, 0, 0), new Pose(side - r, 0, 0)))
                .setTangentHeadingInterpolation()

                // CORNER 1
                .addPath(new BezierCurve(new Pose(side - r, 0, 0), new Pose(side, 0, 0), new Pose(side, r, 0)))
                .setTangentHeadingInterpolation()

                // SIDE 2
                .addPath(new BezierLine(new Pose(side, r, 0), new Pose(side, side - r, 0)))
                .setTangentHeadingInterpolation()

                // CORNER 2
                .addPath(new BezierCurve(new Pose(side, side - r, 0), new Pose(side, side, 0), new Pose(side - r, side, 0)))
                .setTangentHeadingInterpolation()

                // SIDE 3
                .addPath(new BezierLine(new Pose(side - r, side, 0), new Pose(r, side, 0)))
                .setTangentHeadingInterpolation()

                // CORNER 3
                .addPath(new BezierCurve(new Pose(r, side, 0), new Pose(0, side, 0), new Pose(0, side - r, 0)))
                .setTangentHeadingInterpolation()

                // SIDE 4
                .addPath(new BezierLine(new Pose(0, side - r, 0), new Pose(0, r, 0)))
                .setTangentHeadingInterpolation()

                // CORNER 4 (Returning to start)
                .addPath(new BezierCurve(new Pose(0, r, 0), new Pose(0, 0, 0), new Pose(r, 0, 0)))
                .setTangentHeadingInterpolation()

                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // 'true' forces the follower to hold the end pose, reducing drift
                follower.followPath(squarePath, true);
                setPathState(1);
                break;

            case 1:
                // The state ends only when the robot is settled at (0,0)
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

        // Ensure this points to your actual Follower initialization logic
        follower = Constants.createFollower(hardwareMap);

        // Tuning: increase if too slow, decrease if robot flips
        follower.setMaxPower(0.7);

        buildPaths();

        // Starting heading must match the first path tangent (0 degrees)
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