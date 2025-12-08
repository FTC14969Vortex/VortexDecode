package org.firstinspires.ftc.teamcode.newStructureOptionB;

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

    private final Intake intake;
    private final DistanceSensor channelSensor;
    private final Telemetry telemetry;

    private IntakeState state = IntakeState.IDLE;
    private IntakeResult result = IntakeResult.NONE;

    private final ElapsedTime timer = new ElapsedTime();

    private long intakeTimeoutMs = 2500; //
    private int ballCount = 0;

//    private long startTimeMs;

    public IntakeController(Intake intake,
                            DistanceSensor channelSensor,
                            Telemetry telemetry) {
        this.intake = intake;
        this.channelSensor = channelSensor;
        this.telemetry = telemetry;
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
    }

    public void startCycle(int targetBalls, long timeoutSec) {
        if (state != IntakeState.IDLE) return;   // ignore if busy

        this.targetBalls = targetBalls;
        this.intakeTimeoutMs = timeoutSec;
        this.ballCount = 0;

        timer.reset();
        intake.startIntake();        // <- motor start is here
        state = IntakeState.RUNNING;
        result = IntakeResult.NONE;

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
        if (timer.seconds() > timeoutSec) {
            if (ballsCollected == 0) {
                finish(IntakeResult.NO_BALL);
            } else {
                finish(IntakeResult.GOT_BALLS);
            }
            telemetry.addData("Intake", "Timeout; balls=%d result=%s",
                    ballsCollected, result);
            return;
        }

        // 2) Read sensor & detect new ball edge
        boolean detected;
        try {
            detected = Util.isObjectDetected(channelSensor, telemetry);
        } catch (Exception e) {
            finish(IntakeResult.ABORTED);
            telemetry.addData("Intake", "ERROR reading sensor: %s", e.getMessage());
            return;
        }

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
        intake.stop();               // <- intake.stop() here, only once
        state = IntakeState.DONE;
        result = finalResult;
    }
}
