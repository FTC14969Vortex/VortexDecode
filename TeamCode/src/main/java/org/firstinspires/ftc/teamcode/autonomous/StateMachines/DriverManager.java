package org.firstinspires.ftc.teamcode.autonomous.StateMachines;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.calibration.RobotConstants;
import org.firstinspires.ftc.teamcode.calibration.RobotConstants.ComponentPosition;
import org.firstinspires.ftc.teamcode.external.gobilida.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.motion.CoordinateTransformer;
import org.firstinspires.ftc.teamcode.motion.FieldPose;
import org.firstinspires.ftc.teamcode.motion.MotionConfig;
import org.firstinspires.ftc.teamcode.motion.MotionExecutor;
import org.firstinspires.ftc.teamcode.motion.MotionExecutorAdapter;
import org.firstinspires.ftc.teamcode.motion.MotionState;
import org.firstinspires.ftc.teamcode.utils.StateMachineLogger;
import org.firstinspires.ftc.teamcode.subsystems.BaseMotion;

public class DriverManager {

    public enum DriveState {
        IDLE,       // nothing currently running
        MOVING,     // driving toward target position
        DONE        // cycle finished
    }

    public enum DriveResult {
        NONE,           // no cycle yet
        ARRIVED_OK,     // reached target pose successfully
        PATH_FAILED,    // couldn't reach target (timeout, large error)
        ABORTED         // cancelled by GameManager
    }

    // What kind of goal this cycle is about
    public enum DriveGoalKind {
        NONE,
        GOTO_BALL_SPOT,
        GOTO_BALL_FINISH,  // drive to intake finish position while intaking
        GOTO_SHOOT_SPOT,
        GOTO_PARK,
        PURE_ALIGN_TAG,   // e.g. only alignment around current spot
        DRIVE_FORWARD_FOR_INTAKE   // drive forward fixed distance while intaking (deprecated - use GOTO_BALL_FINISH)
    }

    private final Telemetry telemetry;
    private final ElapsedTime timer = new ElapsedTime();
    
    private final BaseMotion baseMotion;
    private final MotionExecutor motionExecutor;  // Extracted from BaseMotion (same instance)
    private final MotionExecutorAdapter motionAdapter;
    
    private StateMachineLogger logger;  // Optional logger for detailed debugging

    private DriveState  state  = DriveState.IDLE;
    private DriveState  previousState = DriveState.IDLE;  // Track previous state for logging
    private DriveResult result = DriveResult.NONE;
    private DriveGoalKind goalKind = DriveGoalKind.NONE;

    // Target parameters for current cycle
    private double timeoutSec = 5.0;

    public DriverManager(OpMode opMode, Telemetry telemetry) {
        this.telemetry = telemetry;
        
        // Create BaseMotion and initialize (handles hardware setup)
        this.baseMotion = new BaseMotion();
        this.baseMotion.init(opMode);  // Creates MotionExecutor internally
        
        // Extract MotionExecutor from BaseMotion (same instance)
        this.motionExecutor = baseMotion.getMotionExecutor();
        
        // Wrap with adapter for non-blocking operation
        this.motionAdapter = new MotionExecutorAdapter(motionExecutor);
    }

    // ---------- Public API for GameManager ----------

    public DriveState getState()        { return state; }
    public DriveResult getResult()      { return result; }
    public DriveGoalKind getGoalKind()  { return goalKind; }

    public boolean isIdle() { return state == DriveState.IDLE; }
    public boolean isDone() { return state == DriveState.DONE; }
    
    /**
     * Set logger for detailed debugging (optional).
     */
    public void setLogger(StateMachineLogger logger) {
        this.logger = logger;
    }

    /** Hard stop + clear result. */
    public void resetCycle() {
        // Track previous state before resetting (for accurate logging)
        previousState = state;
        
        state     = DriveState.IDLE;
        result    = DriveResult.NONE;
        goalKind  = DriveGoalKind.NONE;
        
        // Log the reset transition if logger is attached
        if (logger != null) {
            logger.logStateTransition(previousState, state, DriveState.IDLE, "Cycle reset");
        }
    }

    public void abortCycle() {
        if (state == DriveState.IDLE) return;
        
        // Stop MotionExecutor motion immediately
        motionAdapter.abort();
        
        state  = DriveState.DONE;
        result = DriveResult.ABORTED;
    }

    /**
     * Initialize reference point and set initial position (convenience method with default reference point).
     * This encapsulates BaseMotion calls and prevents double resets.
     * 
     * @param initialPosition FieldPose where reference point should be initially positioned
     */
    public void initReferencePoint(FieldPose initialPosition) {
        // Default to BACK_RIGHT_CORNER (matches FullAutoOperateTest)
        initReferencePoint(RobotConstants.BACK_RIGHT_CORNER, initialPosition);
    }
    
    /**
     * Initialize reference point and set initial position (full control).
     * Allows alliance-specific reference points for future flexibility.
     * 
     * @param refPoint ComponentPosition to use as reference point
     * @param initialPosition FieldPose where reference point should be initially positioned
     */
    public void initReferencePoint(ComponentPosition refPoint, FieldPose initialPosition) {
        baseMotion.setReferencePoint(refPoint);
        baseMotion.setReferencePointToPosition(initialPosition);
        
        telemetry.addData("Drive", "Init reference point: %s at (%.1f, %.1f, %.1f°)",
            refPoint, initialPosition.x, initialPosition.y, initialPosition.heading);
    }
    
    /**
     * Get current robot heading from MotionState (for alignment calculations).
     * 
     * @return Current heading in degrees (field heading from odometry)
     */
    public double getCurrentHeadingDeg() {
        motionExecutor.updateState();
        Pose2D currentPose = motionExecutor.getMotionState().getCurrentPose();
        return currentPose.getHeading(AngleUnit.DEGREES);
    }
    
    /**
     * Get MotionExecutor instance (for CameraServo initialization).
     * 
     * @return MotionExecutor instance
     */
    public MotionExecutor getMotionExecutor() {
        return motionExecutor;
    }
    
    /**
     * Get CoordinateTransformer instance (for CameraServo initialization).
     * 
     * @return CoordinateTransformer instance
     */
    public CoordinateTransformer getCoordinateTransformer() {
        return motionExecutor.getCoordinateTransformer();
    }

    // ------------------------------------------------------------
    // START NEW CYCLES
    // ------------------------------------------------------------

    /** Generalized startCycle() for MOVE goals (ball spot, shoot spot, park). */
    public void startCycle(DriveGoalKind kind, FieldPose target, double timeoutSec) {
        startCycle(kind, target, timeoutSec, null); // Use default velocity (will be determined by goal kind)
    }
    
    /**
     * Generalized startCycle() with explicit velocity.
     * @param kind Goal kind
     * @param target Target pose
     * @param timeoutSec Timeout in seconds
     * @param velocityInchPerSec Velocity in inches/second (null to use default based on goal kind)
     */
    public void startCycle(DriveGoalKind kind, FieldPose target, double timeoutSec, Double velocityInchPerSec) {
        if (state != DriveState.IDLE) return;

        this.goalKind = kind;
        this.timeoutSec = timeoutSec;

        // Target is already FieldPose in reference-point coordinates - use directly!
        double x = target.x;
        double y = target.y;
        double heading = target.heading;
        
        // Determine velocity based on goal kind if not explicitly provided
        double velocity;
        if (velocityInchPerSec != null) {
            velocity = velocityInchPerSec;
        } else {
            // Use velocity profiles matching FullAutoOperateTest
            switch (kind) {
                case GOTO_BALL_FINISH:
                    // Intake movements use slower velocity for stability
                    velocity = 20.0; // INTAKE_VELOCITY
                    break;
                case GOTO_BALL_SPOT:
                case GOTO_SHOOT_SPOT:
                case GOTO_PARK:
                default:
                    // Travel movements use faster velocity
                    velocity = 30.0; // TRAVEL_VELOCITY
                    break;
            }
        }
        
        // Log state transition and settings
        if (logger != null) {
            logger.logStateTransition(previousState, DriveState.MOVING, DriveState.DONE, 
                    String.format("startCycle(%s)", kind));
            logger.logSettings(String.format("goalKind=%s, target=(%.2f, %.2f, %.2f°), timeout=%.2fs, velocity=%.1f",
                    kind, x, y, heading, timeoutSec, velocity));
        }
        
        // Start motion with appropriate velocity
        motionAdapter.startMoveToPose(x, y, heading, velocity, timeoutSec);
        
        previousState = state;
        state = DriveState.MOVING;
        result = DriveResult.NONE;
        timer.reset();

        telemetry.addData("Drive", "Start MOVE cycle: %s to (%.1f, %.1f, %.1f°)",
                kind, x, y, heading);
    }
    
    // Legacy overload for backward compatibility (deprecated - use FieldPose version)
    @Deprecated
    public void startCycle(DriveGoalKind kind, Pose2D target, double timeoutSec) {
        // Convert Pose2D to FieldPose (assumes reference-point frame from MotionState)
        FieldPose fieldPose = new FieldPose(
            target.getX(DistanceUnit.INCH),
            target.getY(DistanceUnit.INCH),
            target.getHeading(AngleUnit.DEGREES)
        );
        startCycle(kind, fieldPose, timeoutSec);
    }

    /**
     * startCycle() for ALIGN-ONLY mode (tag or heading).
     * 
     * EXCEPTION: Robot-center operation (documented exception to FieldPose-only rule).
     * startAlignCycle and startForwardDrive are robot-center operations.
     * They convert ref-point -> robot-center for math and back.
     */
    public void startAlignCycle(double headingDeg, double timeoutSec) {
        if (state != DriveState.IDLE) return;

        this.goalKind = DriveGoalKind.PURE_ALIGN_TAG;
        this.timeoutSec = timeoutSec;
        
        // Log state transition
        if (logger != null) {
            logger.logStateTransition(previousState, DriveState.MOVING, DriveState.DONE, 
                    String.format("startAlignCycle(%.1f°)", headingDeg));
        }

        // Get current reference point pose
        motionExecutor.updateState();
        Pose2D currentRefPointPose = motionExecutor.getMotionState().getCurrentPose();
        
        // Convert reference point to robot center for calculation
        CoordinateTransformer transformer = motionExecutor.getCoordinateTransformer();
        double currentX = currentRefPointPose.getX(DistanceUnit.INCH);
        double currentY = currentRefPointPose.getY(DistanceUnit.INCH);
        double currentHeading = currentRefPointPose.getHeading(AngleUnit.DEGREES);
        
        Pose2D currentRobotCenter = transformer.convertReferencePointToRobotCenter(
            currentX, currentY, currentHeading);
        
        // Create target robot center (same position, new heading)
        Pose2D targetRobotCenter = new Pose2D(
            DistanceUnit.INCH,
            currentRobotCenter.getX(DistanceUnit.INCH),
            currentRobotCenter.getY(DistanceUnit.INCH),
            AngleUnit.DEGREES,
            headingDeg
        );
        
        // Convert robot center target back to reference point coordinates
        FieldPose refPointTarget = CoordinateTransformer.convertRobotCenterToReferencePoint(
            targetRobotCenter);
        
        // FIXED: Use TRAVEL_VELOCITY (30.0) instead of 1.0 to match FullAutoOperateTest behavior
        // FullAutoOperateTest uses rotate() with TRAVEL_VELOCITY and no timeout
        // Higher velocity allows faster alignment and reduces timeout issues
        // Since we're at the same position (pure rotation), linear velocity mainly affects rotation speed
        motionAdapter.startMoveToPose(refPointTarget.x, refPointTarget.y, refPointTarget.heading,
                                  30.0, timeoutSec);  // TRAVEL_VELOCITY for faster alignment (matching FullAutoOperateTest)
        
        previousState = state;
        state = DriveState.MOVING;
        result = DriveResult.NONE;
        timer.reset();
        
        telemetry.addData("Drive", "Start ALIGN cycle to %.1f° at (%.1f, %.1f)",
            headingDeg, currentX, currentY);
    }

    /**
     * Start forward drive for intake sequence - drives forward fixed distance from current position.
     * 
     * EXCEPTION: Robot-center operation (documented exception to FieldPose-only rule).
     * startAlignCycle and startForwardDrive are robot-center operations.
     * They convert ref-point -> robot-center for math and back.
     */
    public void startForwardDrive(double distanceInch, double timeoutSec) {
        if (state != DriveState.IDLE) return;

        this.goalKind = DriveGoalKind.DRIVE_FORWARD_FOR_INTAKE;
        this.timeoutSec = timeoutSec;
        
        // Log state transition
        if (logger != null) {
            logger.logStateTransition(previousState, DriveState.MOVING, DriveState.DONE, 
                    String.format("startForwardDrive(%.1f in)", distanceInch));
        }

        // Get current pose (MotionExecutor returns reference point coordinates)
        motionExecutor.updateState();
        Pose2D currentRefPointPose = motionExecutor.getMotionState().getCurrentPose();
        
        // Convert reference point pose to robot center for calculation
        // (We want to move robot center forward by distanceInch)
        CoordinateTransformer transformer = motionExecutor.getCoordinateTransformer();
        double currentX = currentRefPointPose.getX(DistanceUnit.INCH);
        double currentY = currentRefPointPose.getY(DistanceUnit.INCH);
        double currentHeading = currentRefPointPose.getHeading(AngleUnit.DEGREES);
        
        Pose2D currentRobotCenter = transformer.convertReferencePointToRobotCenter(
            currentX, currentY, currentHeading);
        
        // Calculate target robot center (forward in current heading direction)
        double headingRad = Math.toRadians(currentRobotCenter.getHeading(AngleUnit.DEGREES));
        double targetRobotX = currentRobotCenter.getX(DistanceUnit.INCH) + distanceInch * Math.cos(headingRad);
        double targetRobotY = currentRobotCenter.getY(DistanceUnit.INCH) + distanceInch * Math.sin(headingRad);
        double targetHeading = currentRobotCenter.getHeading(AngleUnit.DEGREES);
        
        // Create target robot center pose
        Pose2D targetRobotCenter = new Pose2D(DistanceUnit.INCH, targetRobotX, targetRobotY, 
                                              AngleUnit.DEGREES, targetHeading);
        
        // Convert robot center target to reference point coordinates
        FieldPose refPointTarget = CoordinateTransformer.convertRobotCenterToReferencePoint(
            targetRobotCenter);
        
        // Start motion using reference point coordinates
        // Use INTAKE_VELOCITY for forward intake drives (matching FullAutoOperateTest)
        double intakeVelocity = 20.0; // INTAKE_VELOCITY
        motionAdapter.startMoveToPose(refPointTarget.x, refPointTarget.y, refPointTarget.heading, 
                                    intakeVelocity, timeoutSec);
        
        previousState = state;
        state = DriveState.MOVING;
        result = DriveResult.NONE;
        timer.reset();

        telemetry.addData("Drive", "Start FORWARD_INTAKE drive: %.1f inches forward", distanceInch);
    }

    // ------------------------------------------------------------
    // UPDATE LOOP (non-blocking)
    // ------------------------------------------------------------

    public void update(double nowSec) {
        // Update adapter (calls MotionExecutor.updateState() internally)
        motionAdapter.update(nowSec);
        
        if (state == DriveState.IDLE || state == DriveState.DONE)
            return;

        // Log current status during movement
        // Note: motionAdapter.update() already calls motionExecutor.updateState() internally,
        // so we can get the current pose from MotionState without calling updateState() again
        if (logger != null && state == DriveState.MOVING) {
            Pose2D currentPose = motionExecutor.getMotionState().getCurrentPose();
            double currentX = currentPose.getX(DistanceUnit.INCH);
            double currentY = currentPose.getY(DistanceUnit.INCH);
            double currentHeading = currentPose.getHeading(AngleUnit.DEGREES);
            double elapsed = timer.seconds();
            double remaining = timeoutSec - elapsed;
            
            logger.logStatus(String.format("state=%s, goalKind=%s, elapsed=%.2fs, remaining=%.2fs, " +
                    "currentPose=(%.2f, %.2f, %.2f°), timeout=%.2fs",
                    state, goalKind, elapsed, remaining, currentX, currentY, currentHeading, timeoutSec));
        }

        // Check completion
        if (motionAdapter.isComplete()) {
            MotionExecutor.MotionResult motionResult = motionAdapter.getResult();
            
            if (motionResult != null && motionResult.success) {
                finish(DriveResult.ARRIVED_OK);
            } else {
                finish(DriveResult.PATH_FAILED);
            }
        } else if (timer.seconds() > timeoutSec) {
            // Backup timeout check (adapter should handle this, but keep as safety)
            finish(DriveResult.PATH_FAILED);
        }
    }

    // ------------------------------------------------------------
    // FINISH HANDLER
    // ------------------------------------------------------------

    private void finish(DriveResult finalResult) {
        // Log state transition
        if (logger != null) {
            logger.logStateTransition(previousState, state, DriveState.DONE, 
                    String.format("finish(%s)", finalResult));
            
            // Log final status
            motionExecutor.updateState();
            Pose2D finalPose = motionExecutor.getMotionState().getCurrentPose();
            double finalX = finalPose.getX(DistanceUnit.INCH);
            double finalY = finalPose.getY(DistanceUnit.INCH);
            double finalHeading = finalPose.getHeading(AngleUnit.DEGREES);
            double elapsed = timer.seconds();
            
            logger.logStatus(String.format("result=%s, elapsed=%.2fs, finalPose=(%.2f, %.2f, %.2f°)",
                    finalResult, elapsed, finalX, finalY, finalHeading));
        }
        
        previousState = state;
        state = DriveState.DONE;
        result = finalResult;

        telemetry.addData("Drive", "Cycle DONE: %s", finalResult);
    }
}
