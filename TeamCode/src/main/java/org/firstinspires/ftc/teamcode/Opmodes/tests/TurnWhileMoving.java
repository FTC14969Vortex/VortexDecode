package org.firstinspires.ftc.teamcode.Opmodes.tests;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Turn while moving", group = "PedroPathTestCases")
public class TurnWhileMoving extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private PathChain rotationPath;

    public void buildPaths() {
        rotationPath = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(0, 0, 0),
                                new Pose(36, 36,90)))
                .setLinearHeadingInterpolation(0,90)
                .build(); 
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Follow the path and hold the end position to prevent drift
                follower.followPath(rotationPath, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.turn(Math.toRadians(180),true);
                    setPathState(2);
                }
            case 2:
                if (!follower.isBusy()) {
                    // Path finished, robot should be at (24,0) facing 180 degrees
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
        // Ensure starting pose matches the start of our path
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
        telemetry.addData("Heading (Deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}