package org.firstinspires.ftc.teamcode.Opmodes.tests;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Hozefa 0.01", group = "PedroPathTestCases")
public class Hozefa extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private int pathState = 0;


    public void autonomousPathUpdate() {

        if(pathState == 0) {
            Pose pose1 = new Pose(0, 0, 0);
            Pose pose2 = new Pose(24, 0, Math.toRadians(180));
            BezierLine line1 = new BezierLine(pose1, pose2);
            PathBuilder builder = follower.pathBuilder();
            builder.addPath(line1);
            builder.setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading());
            PathChain pathChain1 = builder.build();
            follower.followPath(pathChain1);
            setPathState(1);
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