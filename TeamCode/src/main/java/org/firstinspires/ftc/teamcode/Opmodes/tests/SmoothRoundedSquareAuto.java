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
@Autonomous(name = "Smooth Rounded Square Auto 0.2", group = "PedroPathTestCases")
public class SmoothRoundedSquareAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private PathChain squarePath;
    /** * This builds a "Filleted Square".
     * It uses straight lines for sides and Bezier Curves for corners.
     **/
    public void buildPaths() {
        int side = 36;
        int r = 6; // Corner Radius: increase this if the robot still jerks/tips
        squarePath = follower.pathBuilder()
                // --- SIDE 1: Bottom ---
                .addPath(new BezierLine(new Pose(0, 0, 0), new Pose(side - r, 0, 0)))
                .setConstantHeadingInterpolation(0)
                // --- CORNER 1: Bottom Right ---
                .addPath(new BezierCurve(new Pose(side - r, 0, 0), new Pose(side, 0, 0), new Pose(side, r, 0)))
                .setConstantHeadingInterpolation(0)
                // --- SIDE 2: Right ---
                .addPath(new BezierLine(new Pose(side, r, 0), new Pose(side, side - r, 0)))
                .setConstantHeadingInterpolation(0)
                // --- CORNER 2: Top Right ---
                .addPath(new BezierCurve(new Pose(side, side - r, 0), new Pose(side, side, 0), new Pose(side - r, side, 0)))
                .setConstantHeadingInterpolation(0)
                // --- SIDE 3: Top ---
                .addPath(new BezierLine(new Pose(side - r, side, 0), new Pose(r, side, 0)))
                .setConstantHeadingInterpolation(0)
                // --- CORNER 3: Top Left ---
                .addPath(new BezierCurve(new Pose(r, side, 0), new Pose(0, side, 0), new Pose(0, side - r, 0)))
                .setConstantHeadingInterpolation(0)
                // --- SIDE 4: Left ---
                .addPath(new BezierLine(new Pose(0, side - r, 0), new Pose(0, r, 0)))
                .setConstantHeadingInterpolation(0)
                // --- CORNER 4: Bottom Left (Return to Start) ---
                .addPath(new BezierCurve(new Pose(0, r, 0), new Pose(0, 0, 0), new Pose(r, 0, 0)))
                .setConstantHeadingInterpolation(0)
                .build();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(squarePath);
                setPathState(1);
                break;
            case 1:
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
        // IMPORTANT: Cap max power to 50% for initial testing to prevent flipping
        follower.setMaxPower(0.5);
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
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();
    }
}
