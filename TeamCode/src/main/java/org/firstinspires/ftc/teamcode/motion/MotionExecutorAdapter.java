package org.firstinspires.ftc.teamcode.motion;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Non-blocking adapter for MotionExecutor that extracts the control loop from blocking methods.
 * 
 * This adapter allows MotionExecutor to be used in non-blocking state machines by executing
 * one control loop iteration per update() call instead of using a blocking while loop.
 */
public class MotionExecutorAdapter {
    
    public enum AdapterState {
        IDLE,        // No motion active
        EXECUTING,   // Motion in progress
        COMPLETE,    // Motion finished successfully
        FAILED,      // Motion failed (timeout, stall, etc.)
        ABORTED      // Motion cancelled
    }
    
    private final MotionExecutor motionExecutor;
    private AdapterState state = AdapterState.IDLE;
    
    private double targetX, targetY, targetHeading;
    private double maxVelocity, timeoutSec;
    private final ElapsedTime timer;
    private final ElapsedTime stallTimer;
    private double lastUpdateTimeSec = 0.0;
    private double previousVx = 0.0;
    private double previousVy = 0.0;
    private double previousOmega = 0.0;
    private double lastPositionError = Double.MAX_VALUE;
    
    private MotionExecutor.MotionResult result;
    
    public MotionExecutorAdapter(MotionExecutor motionExecutor) {
        this.motionExecutor = motionExecutor;
        this.timer = new ElapsedTime();
        this.stallTimer = new ElapsedTime();
    }
    
    /**
     * Start a new motion to the target pose (non-blocking).
     * 
     * @param x Target X coordinate (reference point coordinates, inches)
     * @param y Target Y coordinate (reference point coordinates, inches)
     * @param heading Target heading (degrees)
     * @param velocity Maximum linear velocity (inches/sec)
     * @param timeout Timeout in seconds
     */
    public void startMoveToPose(double x, double y, double heading, 
                                double velocity, double timeout) {
        if (state == AdapterState.EXECUTING) {
            // Don't start new motion if already executing
            return;
        }
        
        // Store target, reset state
        this.targetX = x;
        this.targetY = y;
        this.targetHeading = heading;
        this.maxVelocity = velocity;
        this.timeoutSec = timeout;
        this.state = AdapterState.EXECUTING;
        this.result = null;
        this.timer.reset();
        this.stallTimer.reset();  // CRITICAL: Reset stall timer (matches MotionExecutor line 755)
        this.lastUpdateTimeSec = 0.0;
        this.previousVx = 0.0;
        this.previousVy = 0.0;
        this.previousOmega = 0.0;
        
        // Update state and set target to calculate initial error (matches MotionExecutor lines 758, 765, 768, 781)
        motionExecutor.updateState();
        MotionState motionState = motionExecutor.getMotionState();
        motionState.setTarget(x, y, heading);
        this.lastPositionError = motionState.getPositionError();  // Initialize from actual initial error
        
        // Reset PID controllers (CRITICAL!) - do this after setting target
        DriveHardware driveHardware = motionExecutor.getDriveHardware();
        driveHardware.getDistanceController().reset(0.0);
        driveHardware.getHeadingController().reset(heading);
    }
    
    /**
     * Abort the current motion immediately.
     */
    public void abort() {
        if (state != AdapterState.EXECUTING) return;
        
        // Stop MotionExecutor (set velocity to zero)
        motionExecutor.setVelocity(0, 0, 0, MotionState.CoordinateMode.FIELD_CENTRIC);
        
        finish(new MotionExecutor.MotionResult(false, 0, 0, timer.milliseconds(), "Aborted"));
    }
    
    /**
     * Update the adapter (one control loop iteration).
     * Must be called every OpMode loop cycle while motion is executing.
     * 
     * @param nowSec Current time in seconds (from OpMode getRuntime())
     */
    public void update(double nowSec) {
        if (state != AdapterState.EXECUTING) return;
        
        // Calculate dt for velocity ramping
        double dt = (lastUpdateTimeSec > 0) 
            ? Math.max(nowSec - lastUpdateTimeSec, 0.001)
            : MotionConfig.CONTROL_LOOP_PERIOD_MS / 1000.0;
        lastUpdateTimeSec = nowSec;
        
        // Update odometry
        motionExecutor.updateState();
        
        // Check timeout
        if (timer.seconds() > timeoutSec) {
            MotionState motionState = motionExecutor.getMotionState();
            finish(new MotionExecutor.MotionResult(false, 
                motionState.getPositionError(), 
                motionState.getHeadingError(), 
                timer.milliseconds(), 
                "Timeout"));
            return;
        }
        
        // Check completion (uses proper heading wraparound handling)
        MotionState motionState = motionExecutor.getMotionState();
        motionState.setTarget(targetX, targetY, targetHeading);
        if (motionState.atTarget()) {
            finish(new MotionExecutor.MotionResult(true, 
                motionState.getPositionError(), 
                motionState.getHeadingError(), 
                timer.milliseconds(), 
                "Arrived"));
            return;
        }
        
        // Stall detection (matches MotionExecutor thresholds)
        double positionError = motionState.getPositionError();
        if (Math.abs(lastPositionError - positionError) < MotionConfig.STALL_VELOCITY_THRESHOLD * 0.02) {
            if (stallTimer.milliseconds() > MotionConfig.STALL_DETECTION_TIME_MS) {
                finish(new MotionExecutor.MotionResult(false, 
                    positionError, 
                    motionState.getHeadingError(), 
                    timer.milliseconds(), 
                    "Stalled"));
                return;
            }
        } else {
            stallTimer.reset();
        }
        lastPositionError = positionError;
        
        // CRITICAL: Match MotionExecutor's exact control law (lines 761-863)
        // MotionExecutor converts reference point target to robot center target, then uses robot center for control
        
        CoordinateTransformer transformer = motionExecutor.getCoordinateTransformer();
        
        // Convert reference point target to robot center target (MotionExecutor line 761-762)
        Pose2D robotCenterTarget = transformer.convertReferencePointToRobotCenter(
            targetX, targetY, targetHeading);
        
        // Get current robot center pose (MotionExecutor line 823)
        // NOTE: getCurrentRobotCenterPose() exists in CoordinateTransformer (line 143)
        // It internally does: getCurrentPose() → convertReferencePointToRobotCenter()
        // This matches exactly how MotionExecutor accesses it
        Pose2D currentRobotCenter = transformer.getCurrentRobotCenterPose();
        
        // Calculate direction vector toward target (field-centric, from deltaX/deltaY)
        double robotCenterTargetX = robotCenterTarget.getX(DistanceUnit.INCH);
        double robotCenterTargetY = robotCenterTarget.getY(DistanceUnit.INCH);
        double currentRobotCenterX = currentRobotCenter.getX(DistanceUnit.INCH);
        double currentRobotCenterY = currentRobotCenter.getY(DistanceUnit.INCH);
        
        double deltaX = robotCenterTargetX - currentRobotCenterX;
        double deltaY = robotCenterTargetY - currentRobotCenterY;
        double distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        
        DriveHardware driveHardware = motionExecutor.getDriveHardware();
        // CRITICAL: Negate distance - PID error = setpoint(0) - (-distance) = +distance (MotionExecutor line 840)
        double desiredLinearSpeed = driveHardware.getDistanceController().calculate(-distanceToTarget);
        
        // Convert speed and direction to velocity components (field-centric)
        double vx, vy;
        if (distanceToTarget > 0.001) {
            double directionX = deltaX / distanceToTarget;  // Normalized direction (lines 845-846)
            double directionY = deltaY / distanceToTarget;
            vx = desiredLinearSpeed * directionX;  // Line 847
            vy = desiredLinearSpeed * directionY;  // Line 848
        } else {
            vx = 0.0;
            vy = 0.0;
        }
        
        // Apply velocity limits (lines 857-858)
        vx = Math.max(-maxVelocity, Math.min(maxVelocity, vx));
        vy = Math.max(-maxVelocity, Math.min(maxVelocity, vy));
        
        // Heading controller uses current heading (not error) - MotionExecutor line 861
        double omega = driveHardware.getHeadingController().calculate(motionState.getHeading());
        omega = Math.max(-MotionConfig.MAX_ANGULAR_VELOCITY,
                Math.min(MotionConfig.MAX_ANGULAR_VELOCITY, omega));
        
        // Apply velocity ramping (prevents jerky motion)
        double[] ramped = applyVelocityRampingWithDt(vx, vy, omega, dt);
        
        // Apply velocity (always FIELD_CENTRIC to match moveToPose())
        motionExecutor.setVelocity(ramped[0], ramped[1], ramped[2], 
                                   MotionState.CoordinateMode.FIELD_CENTRIC);
    }
    
    /**
     * Apply velocity ramping with variable dt (matches MotionExecutor logic but adapted for variable dt).
     * 
     * @param targetVx Target X velocity
     * @param targetVy Target Y velocity
     * @param targetOmega Target angular velocity
     * @param dt Time delta in seconds
     * @return Ramped velocities [vx, vy, omega]
     */
    private double[] applyVelocityRampingWithDt(double targetVx, double targetVy, double targetOmega, double dt) {
        // Calculate maximum allowed velocity change based on acceleration limit
        double maxLinearVelocityChange = MotionConfig.MAX_LINEAR_ACCELERATION * dt;
        double maxAngularVelocityChange = MotionConfig.MAX_ANGULAR_ACCELERATION * dt;
        
        // Apply ramping to X velocity
        double vxChange = targetVx - previousVx;
        if (Math.abs(vxChange) > maxLinearVelocityChange) {
            vxChange = Math.signum(vxChange) * maxLinearVelocityChange;
        }
        double rampedVx = previousVx + vxChange;
        
        // Apply ramping to Y velocity
        double vyChange = targetVy - previousVy;
        if (Math.abs(vyChange) > maxLinearVelocityChange) {
            vyChange = Math.signum(vyChange) * maxLinearVelocityChange;
        }
        double rampedVy = previousVy + vyChange;
        
        // Apply ramping to angular velocity
        double omegaChange = targetOmega - previousOmega;
        if (Math.abs(omegaChange) > maxAngularVelocityChange) {
            omegaChange = Math.signum(omegaChange) * maxAngularVelocityChange;
        }
        double rampedOmega = previousOmega + omegaChange;
        
        // Store for next iteration
        previousVx = rampedVx;
        previousVy = rampedVy;
        previousOmega = rampedOmega;
        
        return new double[]{rampedVx, rampedVy, rampedOmega};
    }
    
    private void finish(MotionExecutor.MotionResult finalResult) {
        state = (finalResult.success) ? AdapterState.COMPLETE : AdapterState.FAILED;
        result = finalResult;
        
        // Stop motion
        motionExecutor.setVelocity(0, 0, 0, MotionState.CoordinateMode.FIELD_CENTRIC);
    }
    
    // ---------- Public Getters ----------
    
    public boolean isComplete() {
        return state == AdapterState.COMPLETE || state == AdapterState.FAILED || state == AdapterState.ABORTED;
    }
    
    public AdapterState getState() {
        return state;
    }
    
    public MotionExecutor.MotionResult getResult() {
        return result;
    }
    
    public MotionExecutor getMotionExecutor() {
        return motionExecutor;
    }
}

