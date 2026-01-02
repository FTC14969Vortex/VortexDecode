package org.firstinspires.ftc.teamcode.autonomous.StateMachines;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.external.gobilida.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.motion.CoordinateTransformer;
import org.firstinspires.ftc.teamcode.motion.FieldPose;
import org.firstinspires.ftc.teamcode.motion.MotionConfig;
import org.firstinspires.ftc.teamcode.motion.MotionExecutor;
import org.firstinspires.ftc.teamcode.motion.MotionExecutorAdapter;
import org.firstinspires.ftc.teamcode.motion.MotionState;

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
        GOTO_SHOOT_SPOT,
        GOTO_PARK,
        PURE_ALIGN_TAG,   // e.g. only alignment around current spot
        DRIVE_FORWARD_FOR_INTAKE   // drive forward fixed distance while intaking
    }

    private final Telemetry telemetry;
    private final ElapsedTime timer = new ElapsedTime();
    
    private final MotionExecutor motionExecutor;
    private final MotionExecutorAdapter motionAdapter;

    private DriveState  state  = DriveState.IDLE;
    private DriveResult result = DriveResult.NONE;
    private DriveGoalKind goalKind = DriveGoalKind.NONE;

    // Target parameters for current cycle
    private Pose2D targetPose;
    private double timeoutSec = 5.0;

    public DriverManager(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        
        // Get hardware
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeftDrive");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRightDrive");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeftDrive");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRightDrive");
        GoBildaPinpointDriver odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        
        // Create MotionExecutor and adapter
        this.motionExecutor = new MotionExecutor(frontLeft, frontRight, backLeft, backRight, odometry);
        this.motionAdapter = new MotionExecutorAdapter(motionExecutor);
    }

    // ---------- Public API for GameManager ----------

    public DriveState getState()        { return state; }
    public DriveResult getResult()      { return result; }
    public DriveGoalKind getGoalKind()  { return goalKind; }

    public boolean isIdle() { return state == DriveState.IDLE; }
    public boolean isDone() { return state == DriveState.DONE; }

    /** Hard stop + clear result. */
    public void resetCycle() {
        state     = DriveState.IDLE;
        result    = DriveResult.NONE;
        goalKind  = DriveGoalKind.NONE;
    }

    public void abortCycle() {
        if (state == DriveState.IDLE) return;
        
        // Stop MotionExecutor motion immediately
        motionAdapter.abort();
        
        state  = DriveState.DONE;
        result = DriveResult.ABORTED;
    }

    /**
     * Set the starting pose for odometry.
     * Must be called before starting any paths (typically before waitForStart() in OpMode).
     * 
     * @param startingPose Starting pose in robot center coordinates
     */
    public void setStartingPose(Pose2D startingPose) {
        // CRITICAL: MotionExecutor doesn't reset odometry automatically
        // Use resetToFieldOrigin(Pose2D) - this is the correct API
        // startingPose is already in robot center coordinates, convert to reference point first
        FieldPose refPointPose = CoordinateTransformer.convertRobotCenterToReferencePoint(startingPose);
        
        Pose2D fieldOrigin = new Pose2D(
            DistanceUnit.INCH,
            refPointPose.x,
            refPointPose.y,
            AngleUnit.DEGREES,
            refPointPose.heading
        );
        
        motionExecutor.resetToFieldOrigin(fieldOrigin);
        
        telemetry.addData("Drive", "Set starting pose: (%.1f, %.1f, %.1f°)",
            refPointPose.x, refPointPose.y, refPointPose.heading);
    }

    // ------------------------------------------------------------
    // START NEW CYCLES
    // ------------------------------------------------------------

    /** Generalized startCycle() for MOVE goals (ball spot, shoot spot, park). */
    public void startCycle(DriveGoalKind kind, Pose2D target, double timeoutSec) {
        if (state != DriveState.IDLE) return;

        this.goalKind = kind;
        this.targetPose = target;
        this.timeoutSec = timeoutSec;

        // Convert robot center (GameManager) → reference point (MotionExecutor)
        FieldPose refPointTarget = CoordinateTransformer.convertRobotCenterToReferencePoint(target);
        
        // Extract coordinates (already in inches and degrees)
        double x = refPointTarget.x;
        double y = refPointTarget.y;
        double heading = refPointTarget.heading;
        
        // Start motion
        motionAdapter.startMoveToPose(x, y, heading, MotionConfig.MAX_LINEAR_VELOCITY, timeoutSec);
        
        state = DriveState.MOVING;
        result = DriveResult.NONE;
        timer.reset();

        telemetry.addData("Drive", "Start MOVE cycle: %s to (%.1f, %.1f, %.1f°)",
                kind, target.getX(), target.getY(), target.getHeading());
    }

    /** startCycle() for ALIGN-ONLY mode (tag or heading). */
    public void startAlignCycle(double headingDeg, double timeoutSec) {
        if (state != DriveState.IDLE) return;

        this.goalKind = DriveGoalKind.PURE_ALIGN_TAG;
        this.timeoutSec = timeoutSec;

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
        // NOTE: convertRobotCenterToReferencePoint() is a static method (CoordinateTransformer line 79)
        FieldPose refPointTarget = CoordinateTransformer.convertRobotCenterToReferencePoint(
            targetRobotCenter);
        
        // Use moveToPose() with small linear velocity (focus on rotation, minimal translation)
        motionAdapter.startMoveToPose(refPointTarget.x, refPointTarget.y, refPointTarget.heading,
                                  1.0, timeoutSec);  // Small linear velocity for pure alignment
        
        state = DriveState.MOVING;
        result = DriveResult.NONE;
        timer.reset();
        
        telemetry.addData("Drive", "Start ALIGN cycle to %.1f° at (%.1f, %.1f)",
            headingDeg, currentX, currentY);
    }

    /** Start forward drive for intake sequence - drives forward fixed distance from current position. */
    public void startForwardDrive(double distanceInch, double timeoutSec) {
        if (state != DriveState.IDLE) return;

        this.goalKind = DriveGoalKind.DRIVE_FORWARD_FOR_INTAKE;
        this.timeoutSec = timeoutSec;

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
        motionAdapter.startMoveToPose(refPointTarget.x, refPointTarget.y, refPointTarget.heading, 
                                    MotionConfig.MAX_LINEAR_VELOCITY, timeoutSec);
        
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
        state = DriveState.DONE;
        result = finalResult;

        telemetry.addData("Drive", "Cycle DONE: %s", finalResult);
    }
}
