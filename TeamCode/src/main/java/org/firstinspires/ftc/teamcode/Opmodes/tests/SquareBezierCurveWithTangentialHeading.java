package org.firstinspires.ftc.teamcode.Opmodes.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Square Bezier Curve With Tangential Heading 0.1", group = "PedroPathTestCases")
public class SquareBezierCurveWithTangentialHeading extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    // We use a PathChain for smooth, non-stop movement
    private PathChain squarePath;

    /** This builds a single PathChain consisting of 4 segments.
     * The follower will pass through each corner without stopping. **/
    public void buildPaths() {
        int side = 36;
        squarePath = follower.pathBuilder()
                // Side 1: (0,0) to (24,0)
                .addPath(
                        new BezierCurve(
                                new Pose(0, 0, 0),
                                new Pose(side, 0, 0),
                                new Pose(side, side, 0)

                        ))
                .setTangentHeadingInterpolation()

                // Side 2: (24,0) to (24,24)
                .addPath(
                        new BezierCurve(
                                new Pose(side, side, 0),
                                new Pose(0, side, 0),
                                new Pose(0, 0, 0)
                        ))
                .setTangentHeadingInterpolation()

                // Side 3: (24,24) to (0,24)
                /* .addPath(
                          new BezierCurve(
                                  new Pose(side, side, 0),
                                  new Pose(0, side, 0),
                                  new Pose(0, 0, 0)
                          ))
                  .setConstantHeadingInterpolation(0)

                  // Side 4: (0,24) back to (0,0)
                  .addPath(
                          new BezierCurve(
                                  new Pose(0, side, 0),
                                  new Pose(0, 0, 0),
                                  new Pose(0, 0, 0)
                          ))
                  .setConstantHeadingInterpolation(0)*/

                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start the entire chain
                follower.followPath(squarePath);
                setPathState(1);
                break;

            case 1:
                // Check if the robot has finished all 4 sides
                if (!follower.isBusy()) {
                    setPathState(-1); // End Autonomous
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

        // Initialize the follower using your team's constants
        follower = Constants.createFollower(hardwareMap);

        // buildPaths requires the follower to be initialized first for the pathBuilder
        buildPaths();

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

        // Telemetry for monitoring movement
        telemetry.addData("Path State: ", pathState);
        telemetry.addData("Is Busy: ", follower.isBusy());
        telemetry.addData("X: ", follower.getPose().getX());
        telemetry.addData("Y: ", follower.getPose().getY());
        telemetry.update();
    }
}


