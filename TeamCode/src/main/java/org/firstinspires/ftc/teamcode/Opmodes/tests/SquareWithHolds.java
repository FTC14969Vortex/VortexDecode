package org.firstinspires.ftc.teamcode.Opmodes.tests;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Square with 3sec Holds", group = "PedroPathTestCases")
public class SquareWithHolds extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    // Define 4 separate paths for the sides
    private Path side1, side2, side3, side4;
    public void buildPaths() {
        double sideLength = 36.0;
        // SIDE 1: (0,0) -> (36,0)
        side1 = new Path(new BezierLine(new Pose(0, 0, 0), new Pose(sideLength, 0, 0)));
        side1.setConstantHeadingInterpolation(0);
        // SIDE 2: (36,0) -> (36,36)
        side2 = new Path(new BezierLine(new Pose(sideLength, 0, 0), new Pose(sideLength, sideLength, 0)));
        side2.setConstantHeadingInterpolation(0);
        // SIDE 3: (36,36) -> (0,36)
        side3 = new Path(new BezierLine(new Pose(sideLength, sideLength, 0), new Pose(0, sideLength, 0)));
        side3.setConstantHeadingInterpolation(0);
        // SIDE 4: (0,36) -> (0,0)
        side4 = new Path(new BezierLine(new Pose(0, sideLength, 0), new Pose(0, 0, 0)));
        side4.setConstantHeadingInterpolation(0);
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            // --- SIDE 1 ---
            case 0:
                // followPath(path, true) tells Pedro to hold position aggressively at the end
                follower.followPath(side1, true);
                setPathState(1);
                break;
            case 1:
                // Wait for robot to finish path AND for 3 seconds to pass
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 3.0) {
                        setPathState(2);
                    }
                }
                break;
            // --- SIDE 2 ---
            case 2:
                follower.followPath(side2, true);
                setPathState(3);
                break;
            case 3:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 3.0) {
                        setPathState(4);
                    }
                }
                break;
            // --- SIDE 3 ---
            case 4:
                follower.followPath(side3, true);
                setPathState(5);
                break;
            case 5:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 3.0) {
                        setPathState(6);
                    }
                }
                break;
            // --- SIDE 4 ---
            case 6:
                follower.followPath(side4, true);
                setPathState(7);
                break;
            case 7:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 3.0) {
                        setPathState(-1); // End Auto
                    }
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
        // Ensure this matches the start of side1
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
        telemetry.addData("Time in State", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();
    }
}






















