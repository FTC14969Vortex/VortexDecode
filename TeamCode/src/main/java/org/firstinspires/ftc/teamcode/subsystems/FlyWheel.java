package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class FlyWheel {

    
    private DcMotorEx flyWheel;
    private OpMode opMode;
    public static int FLYWHEEL_SHOOTING_VELOCITY = 1200;
    public double FLYWHEEL_SHOOTING_POWER = (0.65*FLYWHEEL_SHOOTING_VELOCITY)/1500;
    
    // Non-blocking spin-up state machine
    private enum SpinUpState {
        IDLE,
        RAMPING_UP,
        VELOCITY_CONTROL
    }
    
    private SpinUpState spinUpState = SpinUpState.IDLE;
    private double targetRpm = 0.0;
    private static final double TARGET_DELTA = 10.0; // RPM tolerance for "same target"
    



    public void init (OpMode opMode) {

        this.opMode = opMode;

        flyWheel = opMode.hardwareMap.get(DcMotorEx.class, "flyWheel");
        flyWheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void start() {
        flyWheel.setPower(FLYWHEEL_SHOOTING_POWER);
    }
    public void start(Double flyWheelShootingPower) {
        flyWheel.setPower(flyWheelShootingPower);
    }
    public void stop() {
        flyWheel.setPower(0.0);
    }
    
    /**
     * Fast stop using reverse maximum power to quickly ramp down the flywheel
     * @param maxWaitTimeMs Maximum time to wait for flywheel to stop (in milliseconds)
     * @param targetVelocity Target velocity to consider "stopped" (default: 50 RPM)
     * @return FlyWheelSpinUpResult containing success status, duration, and final velocity
     */
    public FlyWheelSpinUpResult fastStop(long maxWaitTimeMs, double targetVelocity) {
        long startTime = System.currentTimeMillis();
        long durationInMillis = 0;
        
        // Switch to power control mode for braking
        flyWheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        
        // Apply reverse maximum power to brake quickly
        flyWheel.setPower(-1.0);
        
        // Wait until velocity drops to target or timeout
        while (Math.abs(flyWheel.getVelocity()) > targetVelocity) {
            // Small delay to prevent excessive CPU usage
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
            
            durationInMillis = System.currentTimeMillis() - startTime;
            if (durationInMillis > maxWaitTimeMs) {
                // Timeout during braking
                double achievedVelocity = flyWheel.getVelocity();
                flyWheel.setPower(0.0); // Stop applying reverse power
                return new FlyWheelSpinUpResult(false, durationInMillis, achievedVelocity, targetVelocity);
            }
        }
        
        // Stop applying reverse power once target velocity is reached
        flyWheel.setPower(0.0);
        
        durationInMillis = System.currentTimeMillis() - startTime;
        double finalVelocity = flyWheel.getVelocity();
        
        return new FlyWheelSpinUpResult(true, durationInMillis, finalVelocity, targetVelocity);
    }
    
    /**
     * Fast stop using reverse maximum power with default settings
     * Uses 500ms timeout and 50 RPM target velocity
     * @return FlyWheelSpinUpResult containing success status, duration, and final velocity
     */
    public FlyWheelSpinUpResult fastStop() {
        return fastStop(500, 50.0); // Default: 500ms timeout, 50 RPM target
    }
    public void setPower(double power) {

        flyWheel.setPower(power);
    }

    public double getPower(){
        return flyWheel.getPower();

    }

    public double getVelocity(){
        return flyWheel.getVelocity();
    }

    /**
     * Sets the flywheel to a specific shooting velocity using a two-phase approach:
     * Phase 1: Ramp up at maximum power to quickly reach target velocity
     * Phase 2: Switch to direct velocity control to maintain target velocity
     * 
     * @param targetVelocity The desired shooting velocity in RPM
     * @param maxWaitTimeMs Maximum time to wait for velocity to be reached (in milliseconds)
     * @return FlyWheelSpinUpResult containing success status, duration, and achieved velocity
     */
    public FlyWheelSpinUpResult setToShootingVelocity(double targetVelocity, long maxWaitTimeMs) {
        // Configuration constants
        final double VELOCITY_TOLERANCE_PERCENT = 2.0;  // 2% tolerance
        final double JAM_DETECTION_THRESHOLD = 0.75;  // 75% of target velocity
        
        long startTime = System.currentTimeMillis();
        long durationInMillis = 0;
        
        // Calculate tolerance thresholds
        double lowerThreshold = (1.0 - VELOCITY_TOLERANCE_PERCENT / 100.0) * targetVelocity;  // 98% of target
        double jamThreshold = JAM_DETECTION_THRESHOLD * targetVelocity;  // 75% of target
        
        // Phase 1: Ramp up to target velocity at maximum power
        flyWheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // Use power control for ramp-up
        flyWheel.setPower(1.0); // Maximum power for quick ramp-up
        
        while (flyWheel.getVelocity() < lowerThreshold) {
            // Small delay to prevent excessive CPU usage
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
            
            durationInMillis = System.currentTimeMillis() - startTime;
            if (durationInMillis > maxWaitTimeMs) {
                // Timeout during ramp-up
                double achievedVelocity = flyWheel.getVelocity();
                boolean success = achievedVelocity >= jamThreshold;
                return new FlyWheelSpinUpResult(success, durationInMillis, achievedVelocity, targetVelocity);
            }
        }
        
        // Phase 2: Switch to direct velocity control to maintain target velocity
        flyWheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // Switch to velocity control
        flyWheel.setVelocity(targetVelocity); // Set target velocity directly
        
        durationInMillis = System.currentTimeMillis() - startTime;
        double achievedVelocity = flyWheel.getVelocity();
        
        return new FlyWheelSpinUpResult(true, durationInMillis, achievedVelocity, targetVelocity);
    }
    
    /**
     * Convenience method to set flywheel to the default shooting velocity
     * @param maxWaitTimeMs Maximum time to wait for velocity to be reached (in milliseconds)
     * @return FlyWheelSpinUpResult containing success status, duration, and achieved velocity
     */
    public FlyWheelSpinUpResult setToShootingVelocity(long maxWaitTimeMs) {
        return setToShootingVelocity(FLYWHEEL_SHOOTING_VELOCITY, maxWaitTimeMs);
    }
    
    /**
     * Directly sets the flywheel to a target velocity using encoder-based velocity control
     * (without the power ramp-up phase)
     * @param targetVelocity The desired shooting velocity in RPM
     */
    public void setVelocity(double targetVelocity) {
        flyWheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flyWheel.setVelocity(targetVelocity);
    }
    
    /**
     * Result class for flywheel spin-up operations
     */
    public static class FlyWheelSpinUpResult {
        public boolean success;
        public long durationMs;
        public double achievedVelocity;
        public double targetVelocity;
        
        public FlyWheelSpinUpResult(boolean success, long durationMs, double achievedVelocity, double targetVelocity) {
            this.success = success;
            this.durationMs = durationMs;
            this.achievedVelocity = achievedVelocity;
            this.targetVelocity = targetVelocity;
        }
    }
    
    // ========== NON-BLOCKING SPIN-UP API ==========
    
    /**
     * Start non-blocking spin-up to target RPM.
     * Idempotent: If already spinning toward same target (within TARGET_DELTA), does nothing.
     * Only resets to RAMPING_UP if target changes significantly or state is IDLE.
     * 
     * @param targetRpm Target RPM for flywheel
     */
    public void startSpinUp(double targetRpm) {
        // Fail-fast: Check initialization
        if (flyWheel == null) {
            throw new IllegalStateException("FlyWheel not initialized. Call init(OpMode) first.");
        }
        
        // Idempotent: If already spinning toward same target, don't reset
        if (spinUpState != SpinUpState.IDLE && 
            Math.abs(targetRpm - this.targetRpm) < TARGET_DELTA) {
            // Already spinning toward same target - do nothing
            return;
        }
        
        // Start ramp-up phase (power control) - only if target changed or was IDLE
        flyWheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flyWheel.setPower(1.0);
        // Store target and initialize internal state machine
        this.targetRpm = targetRpm;
        this.spinUpState = SpinUpState.RAMPING_UP;
    }
    
    /**
     * Update internal state machine for non-blocking spin-up.
     * Must be called in update loop for state machine to progress AND to maintain velocity control.
     * Critical: Must be called in BOTH SPINNING_UP and FIRING states to maintain velocity between shots.
     */
    public void updateSpinUp() {
        // Fail-fast: Check initialization
        if (flyWheel == null) {
            throw new IllegalStateException("FlyWheel not initialized. Call init(OpMode) first.");
        }
        
        // Internal state machine manages ramp-up → mode switch → velocity control
        double currentVelocity = flyWheel.getVelocity();
        
        switch (spinUpState) {
            case RAMPING_UP:
                double lowerThreshold = 0.98 * targetRpm;
                if (currentVelocity >= lowerThreshold) {
                    // Switch to velocity control mode
                    flyWheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    flyWheel.setVelocity(targetRpm);
                    spinUpState = SpinUpState.VELOCITY_CONTROL;
                }
                break;
                
            case VELOCITY_CONTROL:
                // Maintain velocity at target (critical - must be called regularly)
                flyWheel.setVelocity(targetRpm);
                // (isAtTarget() checks completion)
                break;
                
            case IDLE:
                // Nothing to do
                break;
        }
    }
    
    /**
     * Check if flywheel is at target velocity.
     * 
     * @param targetRpm Target RPM
     * @param tolerance Tolerance factor (e.g., 0.95 means 95% of target)
     * @return true if current velocity >= (targetRpm * tolerance)
     */
    public boolean isAtTarget(double targetRpm, double tolerance) {
        if (flyWheel == null) {
            return false;
        }
        double currentVelocity = flyWheel.getVelocity();
        return currentVelocity >= (targetRpm * tolerance);
    }
    
    /**
     * Stop spin-up and reset state to IDLE.
     * Stops motor and resets internal state machine.
     */
    public void stopSpinUp() {
        if (flyWheel != null) {
            flyWheel.setPower(0.0);
        }
        spinUpState = SpinUpState.IDLE;
        targetRpm = 0.0;
    }
    
    /**
     * Get current spin-up state (for debugging/telemetry).
     * 
     * @return Current SpinUpState
     */
    public SpinUpState getSpinUpState() {
        return spinUpState;
    }
    
    /**
     * Get current target RPM (for debugging/telemetry).
     * 
     * @return Current target RPM
     */
    public double getTargetRpm() {
        return targetRpm;
    }
}
