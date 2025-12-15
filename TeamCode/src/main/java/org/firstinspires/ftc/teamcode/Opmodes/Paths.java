package org.firstinspires.ftc.teamcode.Opmodes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Paths {
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;
    public PathChain Path9;
    public PathChain Path10;

    public Paths(Follower follower) {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(17.646, 118.293), new Pose(57.731, 88.230))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-40), Math.toRadians(-40))
                .build();
        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(57.731, 88.230), new Pose(17.855, 84.242))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(17.855, 84.242), new Pose(16.033, 70.396))
                )
                .setTangentHeadingInterpolation()
                .build();
        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(16.033, 70.396), new Pose(57.936, 87.886))
                )
                .setTangentHeadingInterpolation()
                .build();
        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(57.936, 87.886), new Pose(35.345, 59.464))
                )
                .setTangentHeadingInterpolation()
                .build();
        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(35.345, 59.464), new Pose(16.761, 59.829))
                )
                .setTangentHeadingInterpolation()
                .build();
        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(16.761, 59.829), new Pose(58.300, 87.886))
                )
                .setTangentHeadingInterpolation()
                .build();
        Path8 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(58.300, 87.886), new Pose(34.616, 35.780))
                )
                .setTangentHeadingInterpolation()
                .build();
        Path9 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(34.616, 35.780), new Pose(17.855, 35.780))
                )
                .setTangentHeadingInterpolation()
                .build();
        Path10 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(17.855, 35.780), new Pose(58.300, 87.521))
                )
                .setTangentHeadingInterpolation()
                .build();
    }

}