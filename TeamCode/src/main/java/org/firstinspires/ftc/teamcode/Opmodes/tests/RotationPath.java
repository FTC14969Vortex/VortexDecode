package org.firstinspires.ftc.teamcode.Opmodes.tests;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Move and Turn 0.84", group = "PedroPathTestCases")
public class RotationPath extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private PathChain rotationPath;

    public void buildPaths() {
        // Start: (0,0) Heading 0
        // End:   (48,0) Heading 180 (Math.PI)

        rotationPath = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(0, 0, 0), new Pose(48, 0,Math.toRadians(180))))
                .setLinearHeadingInterpolation(0,Math.toRadians(180))
//                .addPath(
//                        new BezierLine(
//                                new Pose(0, 0, 0),
//                                new Pose(0, 0,0)))
//                .setLinearHeadingInterpolation(0,0)
                .build(); // .build() ensures all math is calculated correctly
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(rotationPath,false);
                //follower.turnTo(Math.toRadians(90));
                // Follow the path and hold the end position to prevent drift
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    //follower.turnTo(Math.toRadians(180));
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    //follower.turnTo(Math.toRadians(270));
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    //follower.turnTo(Math.toRadians(45));
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