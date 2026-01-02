package org.firstinspires.ftc.teamcode.autonomous.StateMachines;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.Helper.*;

public class ShootManager {

    public enum ShooterState {
        IDLE,        // not doing anything
        SPINNING_UP, // ramping flywheel to target velocity
        FIRING,      // running flipper/kicker sequence
        DONE         // finished this cycle (success or failure)
    }

    public enum ShooterResult {
        NONE,        // no cycle yet
        SUCCESS,     // shot sequence completed
        JAM_FAILED,  // could not spin up or detected jam
        ABORTED      // cancelled due to time or gameManager
    }

    private final FlyWheel flyWheel;
    private final Kicker kicker;
    private final Flipper flipper;
    private final Intake intake;
    private final Telemetry telemetry;
    private boolean dryRun = false;

    private ShooterState state  = ShooterState.IDLE;
    private ShooterResult result = ShooterResult.NONE;

    private final ElapsedTime timer = new ElapsedTime();

    // parameters for this cycle
    private double targetDistanceInch = 0.0;
    private double timeoutSec = 3.0;

    // internal
    private int shotIndex = 0;
    private final int maxShots = 3; // or 1, or configurable

    public ShootManager(FlyWheel flyWheel,
                        Kicker kicker,
                        Flipper flipper,
                        Intake intake,
                        Telemetry telemetry) {
        this.flyWheel = flyWheel;
        this.kicker = kicker;
        this.flipper = flipper;
        this.intake = intake;
        this.telemetry = telemetry;
    }

    public ShootManager(FlyWheel flyWheel,
                        Kicker kicker,
                        Flipper flipper,
                        Intake intake,
                        Telemetry telemetry,
                        boolean dryRun) {
        this.flyWheel = flyWheel;
        this.kicker = kicker;
        this.flipper = flipper;
        this.intake = intake;
        this.telemetry = telemetry;
        this.dryRun = dryRun;
    }

// -------- Public API for GameManager --------

    public ShooterState getState()  { return state; }
    public ShooterResult getResult(){ return result; }

    public boolean isIdle() { return state == ShooterState.IDLE; }
    public boolean isDone() { return state == ShooterState.DONE; }

    public void resetCycle() {
        // stop everything and clear
        if (!dryRun) {
            flyWheel.stop();
            kicker.setGatePosition(Kicker.GATE_CLOSE);
            flipper.resetFlipper();
        }
        state  = ShooterState.IDLE;
        result = ShooterResult.NONE;
        shotIndex = 0;
    }

    /** GameManager calls this only when:
     *  - robot is already aligned to goal (by Drive subsystem)
     *  - we know our distance to goal (for flywheel tuning)
     */
    public void startCycle(double distanceInch, double timeoutSec) {
        if (state != ShooterState.IDLE) return;

        this.targetDistanceInch = distanceInch;
        this.timeoutSec = timeoutSec;
        this.shotIndex = 0;

        if (dryRun) {
            // Testing mode: skip hardware, immediately complete
            finish(ShooterResult.SUCCESS);
        } else {
            // Normal mode: prepare mechanisms for shooting and begin spin-up
            Util.prepareFlyWheelToShoot(flyWheel, kicker, intake, targetDistanceInch, telemetry);
            timer.reset();
            state  = ShooterState.SPINNING_UP;
            result = ShooterResult.NONE;
        }

        telemetry.addData("Shooter", "Start cycle: dist=%.1f timeout=%.1fs",
                distanceInch, timeoutSec);
    }

    public void abortCycle() {
        if (state == ShooterState.IDLE) return;
        finish(ShooterResult.ABORTED);
        telemetry.addData("Shooter", "Aborted");
    }

    public void update(double nowSec) {
        switch (state) {
            case IDLE:
            case DONE:
                return;

            case SPINNING_UP:
                updateSpinUp();
                return;

            case FIRING:
                updateFiring();
                return;
        }
    }

// -------- Internal state handlers --------

    private void updateSpinUp() {
        if (timer.seconds() > timeoutSec) {
            finish(ShooterResult.JAM_FAILED);
            telemetry.addData("Shooter", "Spin-up timeout (jam?)");
            return;
        }

        // Use your Util.waitForFlyWheelShootingVelocity model,
        // but in non-blocking form. Rough sketch:
        int targetVelocity = Util.getRequiredFlyWheelVelocity(targetDistanceInch);

        // Example: check if we are close enough:
        double currentVelocity = flyWheel.getVelocity();
        if (currentVelocity >= 0.95 * targetVelocity) {
            // Good enough: go to firing phase
            state = ShooterState.FIRING;
            timer.reset();  // use timer for firing sequence
            telemetry.addData("Shooter", "Spin-up OK (%.0f / %d rpm)",
                    currentVelocity, targetVelocity);
        } else {
            // still spinning up – motor power logic can live here or in FlyWheel
            // e.g., flyWheel.setPower(someControlLaw(targetVelocity, currentVelocity));
        }
    }

    private void updateFiring() {
        if (timer.seconds() > timeoutSec) {
            finish(ShooterResult.ABORTED);
            telemetry.addData("Shooter", "Firing timeout");
            return;
        }

        // Simplest version:
        //   shotIndex = 0..maxShots-1
        //   for each: open gate once, operate flipper, reset flipper
        // For brevity, we compress that logic here.

        // Example pseudo-step: (you’d replace with your timing-based sequence)
        if (shotIndex < maxShots) {
            // run one shot step (non-blocking; use internal sub-timers if needed)
            boolean thisShotCompleted = runOneShotStep(shotIndex);
            if (thisShotCompleted) {
                shotIndex++;
            }
        } else {
            // all shots done
            finish(ShooterResult.SUCCESS);
            telemetry.addData("Shooter", "All shots completed: %d", maxShots);
        }
    }

    /** One place that stops mechanisms and marks DONE+result. */
    private void finish(ShooterResult finalResult) {
        if (!dryRun) {
            flyWheel.stop();
            flipper.resetFlipper();
            kicker.setGatePosition(Kicker.GATE_CLOSE);
        }
        state  = ShooterState.DONE;
        result = finalResult;
    }

    /** Example placeholder for one-shot micro-FSM */
    private boolean runOneShotStep(int index) {
        // You can internalize per-shot timing here with a small sub-state
        // or a small per-shot timer. Return true only when that shot is
        // fully completed.
        //
        // For now, just pretend it’s instant:
        //   - kicker.setPosition(Kicker.gateShoot);
        //   - flipper.turnFlipper(angleForShot(index));
        //   - reset flipper
        return true;
    }
}


