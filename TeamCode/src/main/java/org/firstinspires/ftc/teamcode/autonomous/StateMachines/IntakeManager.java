package org.firstinspires.ftc.teamcode.autonomous.StateMachines;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.Intake;
import org.firstinspires.ftc.teamcode.Helper.Util;

public class IntakeManager {

    // ------------- Local state enums -------------
    public enum IntakeState {
        IDLE,           // not doing anything
        RUNNING,
        DONE
    }

    public enum IntakeResult {
        NONE,           // no cycle yet
        GOT_BALLS,      // >=1 ball
        NO_BALL,        // 0 ball after cycle
        ABORTED         // e.g. time too short, or error
    }

    private  Intake intake;
    private  DistanceSensor channelSensor;
    private Telemetry telemetry;
    private boolean dryRun = false;

    private IntakeState state = IntakeState.IDLE;
    private IntakeResult result = IntakeResult.NONE;

    private final ElapsedTime timer = new ElapsedTime();

    private long intakeTimeoutMs = 2500; //
    private int ballCount = 0;

    private int targetBalls;
    private int ballsCollected;
    private boolean previousDetected = false;  // Track previous sensor state for edge detection

//    private long startTimeMs;

    public IntakeManager(Intake intake,
                         Telemetry telemetry) {
        this.intake = intake;
        this.telemetry = telemetry;
    }

    public IntakeManager(Intake intake,
                         Telemetry telemetry,
                         boolean dryRun) {
        this.intake = intake;
        this.telemetry = telemetry;
        this.dryRun = dryRun;
    }

    // ---------- Public API for GameManager ----------

    public IntakeState getState()  { return state; }
    public IntakeResult getResult(){ return result; }

    public boolean isIdle() { return state == IntakeState.IDLE; }
    public boolean isDone() { return state == IntakeState.DONE; }

    public void resetCycle() {
        state = IntakeState.IDLE;
        result = IntakeResult.NONE;
        ballsCollected = 0;
        previousDetected = false;  // Reset edge detection state
    }

    public void startCycle(int targetBalls, long timeoutSec) {
        if (state != IntakeState.IDLE) return;   // ignore if busy

        this.targetBalls = targetBalls;
        this.intakeTimeoutMs = timeoutSec * 1000;  // Convert seconds to milliseconds
        this.ballCount = 0;

        timer.reset();
        
        if (dryRun) {
            // Testing mode: skip hardware, immediately complete
            finish(IntakeResult.GOT_BALLS);
        } else {
            // Normal mode: start intake motor and begin running
            intake.startIntake();
            state = IntakeState.RUNNING;
            result = IntakeResult.NONE;
        }

        telemetry.addData("Intake", "Start cycle: target=%d timeout=%.1fs",
                targetBalls, timeoutSec);
    }

    /** Allow GameManager to cancel mid-cycle (e.g., time is low). */
    public void abortCycle() {
        if (state == IntakeState.RUNNING) {
            finish(IntakeResult.ABORTED);
        }
    }

    // ---------- Internal FSM ----------

    public void update(double nowSec) {
        switch (state) {
            case IDLE:
            case DONE:
                // nothing to do
                return;

            case RUNNING:
                runUpdate();
                return;
        }
    }

    private void runUpdate() {
        // 1) Timeout?
        if (timer.seconds() * 1000.0 > intakeTimeoutMs) {
            if (ballsCollected == 0) {
                finish(IntakeResult.NO_BALL);
            } else {
                finish(IntakeResult.GOT_BALLS);
            }
            telemetry.addData("Intake", "Timeout; balls=%d result=%s",
                    ballsCollected, result);
            return;
        }

        // 2) Read sensor & detect new ball edge (if sensor available)
        boolean detected = false;
        if (channelSensor != null) {
            try {
                detected = Util.isObjectDetected(channelSensor, telemetry);
            } catch (Exception e) {
                finish(IntakeResult.ABORTED);
                telemetry.addData("Intake", "ERROR reading sensor: %s", e.getMessage());
                return;
            }
        } else {
            // No sensor available - rely on timeout only (sensor detection skipped)
            // This allows intake to work without a sensor, but won't detect balls mid-cycle
            detected = false;
        }

        // 2.5) Edge detection: increment ballsCollected on rising edge (false -> true)
        if (detected && !previousDetected) {
            ballsCollected++;
            telemetry.addData("Intake", "Ball detected! Total: %d/%d", ballsCollected, targetBalls);
        }
        previousDetected = detected;  // Update previous state for next cycle

        // 3) Reached target balls?
        if (ballsCollected >= targetBalls) {
            finish(IntakeResult.GOT_BALLS);
            telemetry.addData("Intake", "Target reached, balls=%d", ballsCollected);
        }
    }

    /** Single place that:
     *  - stops motor
     *  - sets DONE
     *  - records result
     */
    private void finish(IntakeResult finalResult) {
        if (!dryRun) {
            intake.stopIntake();               // <- intake.stop() here, only once
        }
        state = IntakeState.DONE;
        result = finalResult;
    }
}
