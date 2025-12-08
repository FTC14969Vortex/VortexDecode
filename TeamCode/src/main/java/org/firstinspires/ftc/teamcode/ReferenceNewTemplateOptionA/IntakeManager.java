package org.firstinspires.ftc.teamcode.ReferenceNewTemplateOptionA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
        chassis.drive(ballPose.getX(), ballPose.getY(), ballPose.getHeading());
        // (above: you can wrap your Util.moveRobot / field-centric logic)

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
                // Use your Chassis + odo to check if we reached target.
                if (chassis.isAtTargetPose()) {  // you implement this
                    // Optional: do local alignment with vision/IMU.
                    // For now, we just go to ALIGNING_BALL and immediately skip.
                    state = IntakeState.ALIGNING_BALL;
                }
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
