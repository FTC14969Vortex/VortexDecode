package org.firstinspires.ftc.teamcode.ReferenceNewTemplateOptionA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.Helper.Chassis;
import org.firstinspires.ftc.teamcode.Helper.Intake;
import org.firstinspires.ftc.teamcode.Helper.DecodeAprilTag;
import org.firstinspires.ftc.teamcode.Helper.Util;

public class IntakeManager {

    // ------------- Local state enums -------------
    public enum IntakeState {
        IDLE,           // not doing anything
        DRIVING_TO_BALL,
        ALIGNING_BALL,  // optional (can be NO-OP for now)
        INTAKING,
        DONE
    }

    public enum IntakeResult {
        NONE,           // no cycle yet
        GOT_BALLS,      // >=1 ball
        NO_BALL,        // 0 ball after cycle
        ABORTED         // e.g. time too short, or error
    }

    // ------------- Dependencies -------------
    private final Chassis chassis;
    private final Intake intake;
    private final Telemetry telemetry;

    // ------------- Local state -------------
    private IntakeState state = IntakeState.IDLE;
    private IntakeResult result = IntakeResult.NONE;

    private int targetSpotIndex = -1;
    private int ballCount = 0;
    private long startTimeMs;
    private long intakeTimeoutMs = 2500; // tune

    public IntakeManager(Chassis chassis, Intake intake, Telemetry telemetry) {
        this.chassis = chassis;
        this.intake = intake;
        this.telemetry = telemetry;
    }

    // ------------ Exposed getters (for GameManager) -------------
    public IntakeState getState()      { return state; }
    public IntakeResult getResult()    { return result; }
    public int getBallCount()          { return ballCount; }

    /**
     * Called ONCE when GameManager decides "we are in INTAKE_MODE for spot i".
     * This just sets up local state; the heavy work happens in update().
     */
    public void startCycle(Pose2D ballPose, int spotIndex, long timeoutMs) {
        if (state != IntakeState.IDLE && state != IntakeState.DONE) {
            // already busy; ignore
            return;
        }
        this.targetSpotIndex = spotIndex;
        this.intakeTimeoutMs = timeoutMs;
        this.ballCount = 0;
        this.result = IntakeResult.NONE;

        // 1) Start driving to this ball position
        //TODO: use the right drive function, or create a new one
        // should be chassis.drive(ballPose.getX(DistanceUnit.CM), ballPose.getY(DistanceUnit.CM), ballPose.getHeading(AngleUnit.RADIANS));
        chassis.drive(ballPose.getX(DistanceUnit.CM));

        this.state = IntakeState.DRIVING_TO_BALL;
        this.startTimeMs = System.currentTimeMillis();
    }

    /**
     * Called EVERY loop from LinearOpMode.
     * This is the local FSM.
     */
    public void update() {
        switch (state) {

            case IDLE:
            case DONE:
                // nothing to do – waiting for next startCycle()
                break;

            case DRIVING_TO_BALL:
                // TODO:  Use your Chassis + odo to check if we reached target,
                //  create a if statement and only when we have arrived at the position shall we change the state to ALIGNING BALL
                // you implement this
                // Optional: do local alignment with vision/IMU.
                // For now, we just go to ALIGNING_BALL and immediately skip.
                state = IntakeState.ALIGNING_BALL;

                break;

            case ALIGNING_BALL:
                // For now: treat alignment as instant NO-OP.
                // Later, you can call a vision-based align here.
                state = IntakeState.INTAKING;

                // Start the intake hardware.
                intake.startIntake();
                startTimeMs = System.currentTimeMillis();
                break;

            case INTAKING:
                long elapsed = System.currentTimeMillis() - startTimeMs;

                // Example: count balls using distance sensor or beam-break.
                // ballCount = intake.getCurrentBallCount(); // your impl

                if (elapsed > intakeTimeoutMs) {
                    // stop intake
                    intake.stopIntake();

                    // interpret ballCount
                    if (ballCount > 0) {
                        result = IntakeResult.GOT_BALLS;
                    } else {
                        result = IntakeResult.NO_BALL;
                    }
                    state = IntakeState.DONE;
                }
                break;
        }

        // Optional telemetry
        telemetry.addData("IntakeState", state);
        telemetry.addData("IntakeResult", result);
        telemetry.addData("Intake BallCount", ballCount);
    }

    /**
     * Called by GameManager AFTER it has reacted to DONE (updated globals).
     */
    public void acknowledgeDone() {
        if (state == IntakeState.DONE) {
            state = IntakeState.IDLE;
        }
    }

    /**
     * Used by GameManager if time runs out and we want to interrupt.
     */
    public void abort() {
        intake.stopIntake();
        result = IntakeResult.ABORTED;
        state = IntakeState.DONE;
    }
}
8:12
Option A shooting manager:
        package org.firstinspires.ftc.teamcode.ReferenceNewTemplateOptionA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.Helper.*;

public class ShootManager {

    public enum ShootState {
        IDLE,
        DRIVING_TO_SHOT,
        ALIGNING_TAG,
        SPINUP,
        SHOOTING,
        DONE
    }

    public enum ShootResult {
        NONE,
        SUCCESS,
        PARTIAL,
        FAIL_ALIGN,
        FAIL_JAM
    }

    private final Chassis chassis;
    private final FlyWheel flyWheel;
    private final Flipper flipper;
    private final Kicker kicker;
    private final Intake intake;
    private final DecodeAprilTag aprilTag;
    private final Telemetry telemetry;

    private ShootState state  = ShootState.IDLE;
    private ShootResult result = ShootResult.NONE;

    private int targetSpotIndex = -1;
    private int ballsToShoot = 0;
    private long timeoutMs = 4000;
    private long startTimeMs;

    private final String targetTagName = "SOME_TAG_NAME"; // configure

    public ShootManager(Chassis chassis,
                        FlyWheel flyWheel,
                        Flipper flipper,
                        Kicker kicker,
                        Intake intake,
                        DecodeAprilTag aprilTag,
                        Telemetry telemetry) {
        this.chassis = chassis;
        this.flyWheel = flyWheel;
        this.flipper = flipper;
        this.kicker = kicker;
        this.intake = intake;
        this.aprilTag = aprilTag;
        this.telemetry = telemetry;
    }

    public ShootState getState()   { return state; }
    public ShootResult getResult() { return result; }

    /**
     * Called once when GameManager decides "we are in SHOOT_MODE for this spot".
     */
    public void startCycle(Pose2D shootPose, int spotIndex, int ballsToShoot, long timeoutMs) {
        if (state != ShootState.IDLE && state != ShootState.DONE) {
            return;
        }
        this.targetSpotIndex = spotIndex;
        this.ballsToShoot = ballsToShoot;
        this.timeoutMs = timeoutMs;
        this.result = ShootResult.NONE;

        // Start driving to the shooting pose
        //TODO we need to call DriveManager here and then drive to the shootPose;
        // chassis.drive(shootPose.getX(DistanceUnit.CM), shootPose.getY(DistanceUnit.CM), shootPose.getHeading(AngleUnit.RADIANS));
        state = ShootState.DRIVING_TO_SHOT;
        startTimeMs = System.currentTimeMillis();
    }

    public void update() {
        long elapsed = System.currentTimeMillis() - startTimeMs;

        switch (state) {
            case IDLE:
            case DONE:
                break;

            case DRIVING_TO_SHOT:
                //TODO, if statement below shall also check if the robotics is at target position already
                //ideally: if(chassis.isAtTargetPose() || elapsed > timeoutMs)
                if (elapsed > timeoutMs) {
                    // If we timed out before reaching exact pose, we still TRY to align.
                    state = ShootState.ALIGNING_TAG;
                }
                break;

            case ALIGNING_TAG:
                // Use your Util.autoAlignWithAprilTag helper (blocking), or
                // break it into smaller steps. For now, call it once when entering:

                Util.AlignmentResult alignResult =
                        Util.autoAlignWithAprilTag(chassis.getOpMode(),
                                aprilTag,
                                targetTagName,
                                chassis,
                                telemetry);

                if (!alignResult.success) {
                    result = ShootResult.FAIL_ALIGN;
                    state = ShootState.DONE;
                } else {
                    // distance used for velocity calculations
                    double distanceInch = alignResult.distance;
                    // Spinup is done in one blocking helper call for now:
                    Util.prepareForShooting(flyWheel, kicker, flipper, intake, distanceInch, telemetry);
                    state = ShootState.SHOOTING;
                }
                break;

            case SHOOTING:
                // Use your existing Util.shoot() helper.
                // It already handles jam clearing etc. and returns to intake mode.
                // You could extend it to return a result; for now we assume SUCCESS.

                // Example (blocking call):
                Util.shoot(flyWheel, kicker, flipper, intake,
                        /* robotDistanceFromTag (fallback) */ 50.0,
                        aprilTag, targetTagName, telemetry);

                result = ShootResult.SUCCESS; // or PARTIAL/JAM if you extend Util.shoot
                state = ShootState.DONE;
                break;
        }

        telemetry.addData("ShootState", state);
        telemetry.addData("ShootResult", result);
    }

    public void acknowledgeDone() {
        if (state == ShootState.DONE) {
            state = ShootState.IDLE;
        }
    }

    public void abort() {
        // clean up: stop flywheel, reset mechanisms, etc.
        flyWheel.stop();
        kicker.setGatePosition(Kicker.GATE_CLOSE);
        flipper.resetFlipper();
        result = ShootResult.FAIL_JAM; // or ABORT
        state = ShootState.DONE;
    }
}