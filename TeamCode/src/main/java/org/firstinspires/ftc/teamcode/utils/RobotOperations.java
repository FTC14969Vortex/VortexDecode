package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Scanner;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.motion.FieldPose;
import org.firstinspires.ftc.teamcode.motion.FieldPositions;
import org.firstinspires.ftc.teamcode.motion.CoordinateTransformer;
import org.firstinspires.ftc.teamcode.motion.MotionExecutor;
import org.firstinspires.ftc.teamcode.motion.OdometryManager;
import org.firstinspires.ftc.teamcode.subsystems.BaseMotion;
import org.firstinspires.ftc.teamcode.subsystems.FlyWheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Flipper;
import org.firstinspires.ftc.teamcode.vision.CameraServo;

/**
 * RobotOperations - Dedicated robot operation utilities for FTC robot
 *
 * Provides high-level operations for both autonomous and teleop modes:
 * 1. getShootingVelocity() - Calculate optimal flywheel velocity for current position
 * 2. moveToShootingPosition() - Move to position with dynamic flywheel adjustment
 * 3. shoot() - Execute shooting sequence with non-blocking flywheel stop
 * 4. alignToShootingAngle() - Align robot to face AprilTag for optimal shooting
 * 5. moveToLocation() - Move to special locations (parking, loading, etc.) with alliance awareness
 *
 * Key Features:
 * - Dynamic flywheel velocity adjustment during movement
 * - Non-blocking operations for responsive robot control
 * - Thread-safe parallel execution of movement and flywheel control
 * - Interruption-ready design for teleop compatibility
 */
public class RobotOperations {

    // ========== SUBSYSTEM REFERENCES ==========
    private BaseMotion baseMotion;
    private FlyWheel flyWheel;
    private Intake intake;
    private Kicker kicker;
    private Flipper flipper;
    private CameraServo cameraServo;
    private CoordinateTransformer coordinateTransformer;
    private OpMode opMode;

    // ========== FLYWHEEL PARAMETERS ==========
    private static final double FLYWHEEL_VELOCITY_SLOPE = 6.17;      // RPM per inch
    private static final double FLYWHEEL_VELOCITY_INTERCEPT = 867; // Base RPM
    private static final double MIN_FLYWHEEL_VELOCITY = 900.0;
    private static final double MAX_FLYWHEEL_VELOCITY = 2000.0;
    private static final double VELOCITY_TOLERANCE_PERCENT = 2.0;   // 2% tolerance for setVelocity switch
    private static final long FLYWHEEL_SPINUP_TIMEOUT = 2000;      // ms

    // ========== ALIGNMENT PARAMETERS ==========
    private static final double MIN_ALIGNMENT_ANGLE = 5.0;        // degrees - only align if > 5 degrees
    private static final double TRAVEL_VELOCITY = 50.0;            // inches/sec for movement
    private static final double ALIGNMENT_ANGULAR_VELOCITY = 30;      // inch/s
    private static final int ALIGNMENT_TIMEOUT = 500;              // ms



    // ========== INTAKE PARAMETERS ==========
    private static final double INTAKE_SHOOTING_POWER = 1.0;       // intake need to full power all the time to prevent kicker push ball downwards

    // ========== ALLIANCE CONFIGURATION ==========
    private boolean isBlueAlliance = true;  // Default to blue alliance
    private boolean visioncorrectionON = false; // turn on auto correction
    private boolean shooting_4th = false;
    private int targetTagId = 20;  // Default to blue alliance (Tag 20), red alliance uses Tag 24

    // ========== DYNAMIC FLYWHEEL CONTROL ==========
    private volatile boolean isDynamicFlywheelActive = false;
    private Thread dynamicFlywheelThread;
    private volatile double currentTargetVelocity = 0.0;
    private final Object flywheelLock = new Object();

    // ========== MOVEMENT RESULT TRACKING ==========
    private MotionExecutor.MotionResult lastMoveResult = null;



    /**
     * Initializes RobotOperations with required subsystems
     *
     * @param baseMotion BaseMotion subsystem for robot movement
     * @param flyWheel FlyWheel subsystem for shooting
     * @param intake Intake subsystem
     * @param kicker Kicker subsystem for gate control
     * @param flipper Flipper subsystem for shooting
     * @param cameraServo CameraServo for AprilTag detection and calculations
     * @param coordinateTransformer CoordinateTransformer for position conversions
     * @param opMode OpMode instance for telemetry and hardware access
     */
    public void init(BaseMotion baseMotion, FlyWheel flyWheel, Intake intake,
                     Kicker kicker, Flipper flipper, CameraServo cameraServo,
                     CoordinateTransformer coordinateTransformer, OpMode opMode) {
        this.baseMotion = baseMotion;
        this.flyWheel = flyWheel;
        this.intake = intake;
        this.kicker = kicker;
        this.flipper = flipper;
        this.cameraServo = cameraServo;
        this.coordinateTransformer = coordinateTransformer;
        this.opMode = opMode;
    }

    /**
     * Sets the target AprilTag for alliance-specific operations
     *
     * @param tagId AprilTag ID (20 for blue alliance, 24 for red alliance)
     */
    public void setTargetTag(int tagId) {
        this.targetTagId = tagId;
        if (cameraServo != null) {
            cameraServo.setTargetTag(tagId);
        }
    }

    /**
     * Sets alliance configuration (convenience method)
     *
     * @param isBlueAlliance true for blue alliance (Tag 20), false for red alliance (Tag 24)
     */
    public void setAlliance(boolean isBlueAlliance) {
        this.isBlueAlliance = isBlueAlliance;
        setTargetTag(isBlueAlliance ? 20 : 24);
    }

    /**
     * Gets the current alliance configuration
     *
     * @return true if blue alliance, false if red alliance
     */
    public boolean isBlueAlliance() {
        return isBlueAlliance;
    }

    /**
     * Gets the current target tag ID
     *
     * @return Current target AprilTag ID (20 for blue, 24 for red)
     */
    public int getCurrentTargetTag() {
        return targetTagId;
    }

    // ========== 1. GET SHOOTING VELOCITY ==========

    /**
     * Calculates optimal flywheel velocity based on current robot position
     *
     * Uses current robot position to calculate distance to AprilTag,
     * then determines optimal flywheel velocity for accurate shooting.
     *
     * @return Optimal flywheel velocity in RPM
     */
    public double getShootingVelocity() {
        // Get current reference point position from BaseMotion
        FieldPose currentRefPointPose = baseMotion.getCurrentPose();

        // Convert reference point to robot center position
        Pose2D robotCenterPose = coordinateTransformer.convertReferencePointToRobotCenter(
                currentRefPointPose.x, currentRefPointPose.y, currentRefPointPose.heading);

        FieldPose robotCenterFieldPose = new FieldPose(
                robotCenterPose.getX(DistanceUnit.INCH),
                robotCenterPose.getY(DistanceUnit.INCH),
                robotCenterPose.getHeading(AngleUnit.DEGREES)
        );

        // Calculate distance from robot center to AprilTag
        FieldPose aprilTagPose = getTargetAprilTagPose();
        double distance = calculateDistance(robotCenterFieldPose, aprilTagPose);

        // Calculate flywheel velocity using same formula as CameraServo
        return calculateFlywheelVelocity(distance);
    }

    // ========== 2. MOVE TO SHOOTING POSITION ==========

    /**
     * Moves robot to shooting position with dynamic flywheel velocity adjustment and intake power
     *
     * Simultaneously moves robot to target position while continuously adjusting
     * flywheel velocity based on real-time distance to AprilTag. Sets intake to
     * shooting power during movement to prepare for shooting sequence.
     * Ensures optimal shooting readiness regardless of where movement stops or is interrupted.
     *
     * @param targetPos Target shooting position
     * @return MotionResult indicating success/failure of movement
     */
    public MotionExecutor.MotionResult moveToShootingPosition(FieldPose targetPos, int timeoutMS) {
        // Set intake to shooting power for movement to shooting position
        if (intake != null) {
            intake.setIntakePower(INTAKE_SHOOTING_POWER);
        }

        // Start dynamic flywheel control
        startDynamicFlywheelControl();

        try {
            // Execute movement (this is blocking, but flywheel adjusts in parallel)
            MotionExecutor.MotionResult result;
            if (timeoutMS > 0) {
                result = baseMotion.moveToPose(targetPos, TRAVEL_VELOCITY, timeoutMS);
            } else{
                result = baseMotion.moveToPose(targetPos, TRAVEL_VELOCITY);
            }
            lastMoveResult = result;  // Store result for teleop smart shooting logic
            return result;
        } finally {
            // Stop dynamic flywheel control when movement completes or is interrupted
            stopDynamicFlywheelControl();
        }
    }

    /**
     * Starts background thread for dynamic flywheel velocity control
     * Continuously monitors robot position and adjusts flywheel velocity
     * Public method for teleop manual shooting preparation
     */
    public void startDynamicFlywheelControl() {
        synchronized (flywheelLock) {
            if (isDynamicFlywheelActive) {
                return; // Already running
            }

            isDynamicFlywheelActive = true;
            dynamicFlywheelThread = new Thread(this::dynamicFlywheelControlLoop, "DynamicFlywheel");
            dynamicFlywheelThread.setDaemon(true);
            dynamicFlywheelThread.start();
        }
    }

    /**
     * Stops dynamic flywheel control thread
     * Public method for teleop control
     */
    public void stopDynamicFlywheelControl() {
        synchronized (flywheelLock) {
            if (!isDynamicFlywheelActive) {
                return; // Already stopped
            }

            isDynamicFlywheelActive = false;

            if (dynamicFlywheelThread != null) {
                try {
                    dynamicFlywheelThread.interrupt();
                    dynamicFlywheelThread.join(200); // Wait up to 500ms for clean shutdown
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
                dynamicFlywheelThread = null;
            }
        }
    }

    /**
     * Main loop for dynamic flywheel velocity control
     * Runs in background thread, continuously adjusting flywheel velocity
     */
    private void dynamicFlywheelControlLoop() {
        while (isDynamicFlywheelActive && !Thread.currentThread().isInterrupted()) {
            try {
                // Calculate current optimal velocity
                double optimalVelocity = getShootingVelocity();

                synchronized (flywheelLock) {
                    // Check if velocity needs adjustment
                    double velocityDifference = Math.abs(optimalVelocity - currentTargetVelocity);
                    double currentActualVelocity = flyWheel.getVelocity();

                    if (velocityDifference > 20){ // Large change needed
                        currentTargetVelocity = optimalVelocity;
                    }

                    if (Math.abs( currentTargetVelocity - currentActualVelocity)/currentTargetVelocity * 100.0 > VELOCITY_TOLERANCE_PERCENT) { // Significant change needed

                        // Use appropriate power direction for large velocity changes (non-blocking)
                        if (currentTargetVelocity > currentActualVelocity) {
                            flyWheel.setPower(1.0);  // Positive power to speed up
                        } else {
                            flyWheel.setPower(-1.0); // Negative power to slow down
                        }

                    } else {
                        // Within 2% tolerance - use precise velocity control
                        flyWheel.setVelocity(currentTargetVelocity);
                    }
                }

                Thread.sleep(50);

            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            } catch (Exception e) {
                // Log error but continue
                if (opMode != null) {
                    opMode.telemetry.addLine("WARNING: Dynamic flywheel error: " + e.getMessage());
                    opMode.telemetry.update();
                }
            }
        }
    }

    // ========== 3. SHOOT ==========
   public void shoot() throws InterruptedException {
        shoot(getShootingVelocity(), true, false); //use odo and do autoalignment - for autoop
    }
    
    public void shoot(boolean useCameraServo) throws InterruptedException {
        if (useCameraServo){
            shoot(cameraServo.getFlywheelVelocity(), true, true);
        } else {
            shoot(getShootingVelocity(), true, false);
        }
    }

    public void shoot(boolean useCameraServo, boolean alignToShootingAngle) throws InterruptedException {
        if (useCameraServo){
            shoot(cameraServo.getFlywheelVelocity(), alignToShootingAngle, true);
        } else {
            shoot(getShootingVelocity(), alignToShootingAngle, false);
        }
    }

    public void shoot(double targetVelocity, boolean alignToShootingAngle, boolean useCameraServo) throws InterruptedException {

        if (alignToShootingAngle) { alignToShootingAngle(useCameraServo); }

        if (visioncorrectionON) {
            cameraServo.setAutoOdometryCorrection(true);
        }

        flyWheel.setToShootingVelocity(targetVelocity, FLYWHEEL_SPINUP_TIMEOUT); // set shooting velocity again - extra insurance

        // Create flywheel maintenance thread to continuously maintain target velocity
        final boolean[] maintainFlywheel = {true};
        Thread flywheelMaintenanceThread = new Thread(() -> {
            while (maintainFlywheel[0] && !Thread.currentThread().isInterrupted()) {
                try {
                    flyWheel.setToShootingVelocity(targetVelocity, FLYWHEEL_SPINUP_TIMEOUT);
                    Thread.sleep(50); // Check every 50ms for responsive maintenance
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                } catch (Exception e) {
                    // Log error but continue
                    if (opMode != null) {
                        opMode.telemetry.addLine("WARNING: Flywheel maintenance error: " + e.getMessage());
                        opMode.telemetry.update();
                    }
                }
            }
        }, "FlywheelMaintenance");

        // Start flywheel maintenance thread
        flywheelMaintenanceThread.start();

        try {
            kicker.setGatePosition(Kicker.GATE_SHOOT);
            Thread.sleep(300); // Wait for gate to open, otherwise flipper will run against

            // 1st - kicker 2 ballers
            if (alignToShootingAngle) { alignToShootingAngle(useCameraServo); }
            flipper.turnFlipper(30); //was 120
            Thread.sleep(50);         //
            flipper.resetFlipper();
            Thread.sleep(300);

            // 2nd
            if (alignToShootingAngle) { alignToShootingAngle(useCameraServo); }
            flipper.turnFlipper(150);
            Thread.sleep(150);
            flipper.resetFlipper();
            Thread.sleep(450);

            // 3rd
            if (alignToShootingAngle) { alignToShootingAngle(useCameraServo); }
            flipper.turnFlipper(150);
            Thread.sleep(150);
            flipper.resetFlipper();
            Thread.sleep(150); // give time for the ball to get out

            if (shooting_4th) {
                flipper.turnFlipper(150);
                Thread.sleep(150);
                flipper.resetFlipper();
                Thread.sleep(150);
            }

        } finally {
            // Stop flywheel maintenance thread
            maintainFlywheel[0] = false;
            flywheelMaintenanceThread.interrupt();

            try {
                flywheelMaintenanceThread.join(100); // Wait for thread to finish
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }

            // Cleanup - close gate
            kicker.setGatePosition(Kicker.GATE_CLOSE);

            if(visioncorrectionON){
                cameraServo.setAutoOdometryCorrection(false); // turn off auto correction
            }

            // NON-BLOCKING flywheel stop - robot can move immediately
            Thread flywheelStopThread = new Thread(() -> {
                try {
                    flyWheel.fastStop();
                } catch (Exception e) {
                    // Log error but don't block main operation
                    if (opMode != null) {
                        opMode.telemetry.addLine("WARNING: Flywheel stop error: " + e.getMessage());
                        opMode.telemetry.update();
                    }
                }
            }, "FlywheelStop");
            flywheelStopThread.setDaemon(true);
            flywheelStopThread.start();

            // Set gate to intake position for next operation
            kicker.setGatePosition(Kicker.GATE_INTAKE);
        }
    }

    // ========== 4. ALIGN TO SHOOTING ANGLE ==========

    public boolean alignToShootingAngle(boolean useCameraServo) {
        double angleDifference;
        double currentHeading = 0;
        double requiredHeading = 0;

        if (!useCameraServo){
            // Get current robot center position
            FieldPose currentRefPointPose = baseMotion.getCurrentPose();

            Pose2D robotCenterPose = coordinateTransformer.convertReferencePointToRobotCenter(
                currentRefPointPose.x,
                currentRefPointPose.y,
                currentRefPointPose.heading);

            FieldPose robotCenterFieldPose = new FieldPose(
                robotCenterPose.getX(DistanceUnit.INCH),
                robotCenterPose.getY(DistanceUnit.INCH),
                robotCenterPose.getHeading(AngleUnit.DEGREES)
            );

            // Calculate angle from robot center to AprilTag
            FieldPose aprilTagPose = getTargetAprilTagPose();
            double angleToTag = calculateAngle(robotCenterFieldPose, aprilTagPose);

            // Calculate required robot heading (robot back should face tag)
            // Robot back faces opposite direction of robot front
            requiredHeading = normalizeAngle(angleToTag + 180.0);

            // Calculate angle difference
            currentHeading = robotCenterFieldPose.heading;         
            angleDifference = normalizeAngle(requiredHeading - currentHeading);
        } else {
            angleDifference = cameraServo.getShootingAngle();
        }

        // alignment timeout based rotation angle needed 
        int alignment_timeoutMS = ALIGNMENT_TIMEOUT * (1 + (int)Math.round(Math.abs(angleDifference) / 45.0));        

        // Only align if difference is significant
        if (Math.abs(angleDifference) > MIN_ALIGNMENT_ANGLE) {
            if (opMode != null) {
                opMode.telemetry.addData("Aligning", "Current: %.1f deg, Target: %.1f deg, Diff: %.1f deg",
                        currentHeading, requiredHeading, angleDifference);
                opMode.telemetry.update();
            }

            // Use baseMotion.rotate() which leverages MotionExecutor's built-in timeout mechanism
            // MotionExecutor automatically calculates appropriate timeout based on angular distance
            MotionExecutor.MotionResult result = baseMotion.rotate(angleDifference, ALIGNMENT_ANGULAR_VELOCITY, alignment_timeoutMS);

            if (opMode != null) {
                if (result.success) {
                    opMode.telemetry.addLine("SUCCESS: Alignment complete");
                } else {
                    opMode.telemetry.addLine("WARNING: Alignment incomplete: " + result.failureReason);
                }
                opMode.telemetry.update();
            }

            return true;
        }

        return false; // Already aligned
    }

    // ========== 5. MOVE TO SPECIAL LOCATION ==========

    /**
     * Moves robot to special field locations with alliance awareness and context-specific behaviors
     *
     * Supported locations:
     * - "parking_near": Near parking position (turns off intake)
     * - "parking_end": End parking position (turns off intake)
     * - "open_gate": Gate opening position
     * - "loading": Loading zone position
     * - "shooting_near": Near shooting position
     * - "shooting_far": Far shooting position
     *
     * @param locationName Name of the target location
     * @return MotionExecutor.MotionResult indicating success/failure of movement
     * @throws IllegalArgumentException if locationName is not recognized
     */

    public MotionExecutor.MotionResult moveToLocation(String locationName){
        return moveToLocation(locationName, 0);
    }
     
    public MotionExecutor.MotionResult moveToLocation(String locationName, int timeoutMS) {
        // Get alliance-aware position
        FieldPose targetPosition = getLocationPosition(locationName);

        if (targetPosition == null) {
            throw new IllegalArgumentException("Unknown location: " + locationName +
                    ". Supported: parking_near, parking_end, open_gate, loading, shooting_near, shooting_far");
        }

        // Check if this is a shooting location - delegate to moveToShootingPosition
        if (locationName.toLowerCase().startsWith("shooting")) {
            if (opMode != null) {
                opMode.telemetry.addData("Moving to", "%s (%.1f, %.1f, %.1f deg)",
                        locationName, targetPosition.x, targetPosition.y, targetPosition.heading);
                opMode.telemetry.addLine("TARGET: Delegating to moveToShootingPosition for optimal flywheel control");
                opMode.telemetry.update();
            }

            // Delegate to moveToShootingPosition for shooting locations
            return moveToShootingPosition(targetPosition, timeoutMS);
        }

        // Handle non-shooting locations with standard behavior system
        // Apply location-specific behaviors before movement
        applyLocationBehaviors(locationName, true); // pre-movement

        if (opMode != null) {
            opMode.telemetry.addData("Moving to", "%s (%.1f, %.1f, %.1f deg)",
                    locationName, targetPosition.x, targetPosition.y, targetPosition.heading);
            opMode.telemetry.update();
        }

        // Execute movement
        MotionExecutor.MotionResult result;        
        if (timeoutMS > 0){
            result = baseMotion.moveToPose(targetPosition, TRAVEL_VELOCITY, timeoutMS);
        } else {
            result = baseMotion.moveToPose(targetPosition, TRAVEL_VELOCITY);
        }
        lastMoveResult = result;  // Store result for teleop smart shooting logic

        // Apply location-specific behaviors after movement
        applyLocationBehaviors(locationName, false); // post-movement

        return result;
    }

    /**
     * Gets the alliance-aware position for a named location
     *
     * @param locationName Name of the location
     * @return FieldPose for the location, or null if not found
     */
    public FieldPose getLocationPosition(String locationName) {
        FieldPose bluePosition;

        // Map location names to blue alliance positions
        switch (locationName.toLowerCase()) {
            case "start_near":
                bluePosition = FieldPositions.START_NEAR;
                break;
            case "start_far":
                bluePosition = FieldPositions.START_FAR;
                break;
            case "parking_near":
                bluePosition = FieldPositions.PARKING_NEAR;
                break;
            case "parking_end":
                bluePosition = FieldPositions.PARKING_END;
                break;
            case "open_gate":
                bluePosition = FieldPositions.OPEN_GATE;
                break;
            case "open_gate_intake":
                bluePosition = FieldPositions.OPEN_GATE_INTAKE;
                break;
            case "loading":
                bluePosition = FieldPositions.LOADING_ZONE;
                break;
            case "shooting_near":
                bluePosition = FieldPositions.SHOOTING_NEAR;
                break;
            case "shooting_far":
                bluePosition = FieldPositions.SHOOTING_FAR;
                break;
            case "intake_loading_start":
                bluePosition = FieldPositions.INTAKE_LOADING_START;
                break;
            case "intake_1_start":
                bluePosition = FieldPositions.INTAKE_1_START;
                break;
            case "intake_2_start":
                bluePosition = FieldPositions.INTAKE_2_START;
                break;
            case "intake_3_start":
                bluePosition = FieldPositions.INTAKE_3_START;
                break;
            default:
                return null; // Unknown location
        }

        // Return alliance-specific position
        if (targetTagId == 24) {
            // Red alliance - mirror blue position
            return FieldPositions.getRedPosition(bluePosition);
        } else {
            // Blue alliance - use blue position directly
            return bluePosition;
        }
    }

    /**
     * Applies location-specific behaviors (e.g., turning off intake for parking)
     *
     * @param locationName Name of the location
     * @param isPreMovement true for pre-movement behaviors, false for post-movement
     */
    private void applyLocationBehaviors(String locationName, boolean isPreMovement) {
        String location = locationName.toLowerCase();

        // Parking locations: turn off intake for safety and power conservation
        if (location.startsWith("parking")) {
            if (isPreMovement && intake != null) {
                if (opMode != null) {
                    opMode.telemetry.addLine("PARKING: Turning off intake");
                    opMode.telemetry.update();
                }
                intake.stopIntake(); // Turn off intake when moving to parking
            }
        }
    }

    // ========== UTILITY METHODS ==========

    private FieldPose getTargetAprilTagPose() {
        // Return appropriate AprilTag position based on targetTagId
        if (targetTagId == 24) {
            // Red alliance - use red goal AprilTag (mirrored blue position)
            return FieldPositions.getRedPosition(FieldPositions.GOAL_APRILTAG);
        } else {
            // Blue alliance (default) - use blue goal AprilTag
            return FieldPositions.GOAL_APRILTAG;
        }
    }

    private double calculateDistance(FieldPose pose1, FieldPose pose2) {
        double deltaX = pose2.x - pose1.x;
        double deltaY = pose2.y - pose1.y;
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }
   
    private double calculateAngle(FieldPose from, FieldPose to) {
        double deltaX = to.x - from.x;
        double deltaY = to.y - from.y;
        return Math.toDegrees(Math.atan2(deltaY, deltaX));
    }

   private double normalizeAngle(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle <= -180.0) angle += 360.0;
        return angle;
    }

    /**
     * Calculates flywheel velocity based on distance (same formula as CameraServo)
     */
    private double calculateFlywheelVelocity(double distance) {
        if (distance <= 0) {
            return MIN_FLYWHEEL_VELOCITY;
        }

        double velocity = FLYWHEEL_VELOCITY_SLOPE * distance + FLYWHEEL_VELOCITY_INTERCEPT;
        return Math.max(MIN_FLYWHEEL_VELOCITY, Math.min(MAX_FLYWHEEL_VELOCITY, velocity));

    }

    public boolean isDynamicFlywheelActive() {
        synchronized (flywheelLock) {
            return isDynamicFlywheelActive;
        }
    }

    // ========== MANUAL SHOOTING CONTROL FOR TELEOP ==========

    /**
     * Starts manual flywheel preparation for teleop
     * Begins dynamic flywheel control and sets intake power
     */
    public void startManualFlywheelPreparation() {
        // Set intake power for shooting
        if (intake != null) {
            intake.setIntakePower(INTAKE_SHOOTING_POWER);
        }
        
        // Start dynamic flywheel control for position-based velocity
        startDynamicFlywheelControl();
        
        if (opMode != null) {
            opMode.telemetry.addLine("🎯 Flywheel prepared - Manual positioning enabled");
            opMode.telemetry.addLine("📍 Move robot to desired position and press right bumper again to shoot");
            opMode.telemetry.update();
        }
    }

    /**
     * Executes manual shoot after flywheel preparation
     * Aligns robot and shoots, then stops dynamic flywheel control
     */
    public void executeManualShoot() throws InterruptedException {
        try {

            // Execute shoot with current velocity and alignment
            shoot(true, true); // velocity, align, use camera servo
            
            if (opMode != null) {
                opMode.telemetry.addLine("✅ Manual shoot completed!");
                opMode.telemetry.update();
            }
        } finally {
            // Always stop dynamic flywheel control after shooting
            stopDynamicFlywheelControl();
        }
    }

    public void setVisionCorrection(boolean visioncorrectionON) {
        this.visioncorrectionON = visioncorrectionON;
    }

    public void setShooting4th(boolean shooting_4th) {
        this.shooting_4th = shooting_4th;
    }


    public double getCurrentTargetVelocity() {
        synchronized (flywheelLock) {
            return currentTargetVelocity;
        }
    }

    /**
     * Emergency stop for all operations
     * Stops dynamic flywheel control, robot movement, and intake
     */
    public void emergencyStop() {
        stopDynamicFlywheelControl();
        if (baseMotion != null) {
            baseMotion.stopRobot();
        }
        if (flyWheel != null) {
            flyWheel.fastStop();  // Use fastStop for quicker emergency stopping
        }
        if (intake != null) {
            intake.stopIntake();
        }
    }

    // ========== TELEOP ENHANCEMENT METHODS ==========

    /**
     * Gets the result of the last movement operation
     * Used for teleop smart shooting logic to determine if movement was successful
     *
     * @return MotionExecutor.MotionResult of the last movement, or null if no movement has been performed
     */
    public MotionExecutor.MotionResult getLastMoveResult() {
        return lastMoveResult;
    }

    /**
     * Checks if the robot is currently in a valid shooting zone for DECODE 2025-2026
     *
     * For DECODE field, shooting zones are defined as:
     * - NEAR zone (both alliances): Triangle with vertices at (0,0), (-72,-72), (-72,72)
     * - FAR zone (both alliances): 12" tolerance around SHOOTING_FAR position
     *
     * Uses robot center position for accurate zone detection (alliance-aware).
     *
     * @return true if robot is in a valid shooting zone, false otherwise
     */
    public boolean isInShootingZone() {
        if (baseMotion == null || coordinateTransformer == null) {
            return false;
        }

        // Get current robot reference point position
        FieldPose currentRefPointPose = baseMotion.getCurrentPose();

        // Convert reference point to robot center position for accurate zone detection
        Pose2D robotCenterPose = coordinateTransformer.convertReferencePointToRobotCenter(
                currentRefPointPose.x, currentRefPointPose.y, currentRefPointPose.heading);

        double robotX = robotCenterPose.getX(DistanceUnit.INCH);
        double robotY = robotCenterPose.getY(DistanceUnit.INCH);

        // Check NEAR shooting zone - Triangle with vertices at (0,0), (-72,-72), (-72,72)
        // This triangle represents the near shooting area for both alliances bounced by y = +/-x

        boolean inNearZone = robotY >= robotX && robotY <= -robotX;
        boolean inFarZone = robotY >= (48-robotX) && robotY <= (robotX-48);

        return inNearZone || inFarZone;
        
    }

    /**
     * Clears the last movement result
     * Useful for resetting state between teleop operations
     */
    public void clearLastMoveResult() {
        lastMoveResult = null;
    }

    /**
     * Gets a human-readable status of the last movement operation
     *
     * @return String describing the last movement result, or "No movement performed" if none
     */
    public String getLastMoveResultStatus() {
        if (lastMoveResult == null) {
            return "No movement performed";
        }

        if (lastMoveResult.success) {
            return String.format("SUCCESS - Position error: %.1f\", Heading error: %.1f°, Time: %.0fms",
                    lastMoveResult.finalPositionError, lastMoveResult.finalHeadingError, lastMoveResult.executionTimeMs);
        } else {
            return String.format("FAILED - %s (Position error: %.1f\", Heading error: %.1f°, Time: %.0fms)",
                    lastMoveResult.failureReason, lastMoveResult.finalPositionError,
                    lastMoveResult.finalHeadingError, lastMoveResult.executionTimeMs);
        }
    }

    /**
     * Execute smart shooting logic for teleop
     *
     * This method implements the intelligent shooting decision-making:
     * 1. Move to shooting location
     * 2. If movement successful → auto shoot
     * 3. If movement interrupted but in zone → auto shoot
     * 4. If movement interrupted and not in zone → manual positioning required
     *
     * @param shootingLocation Location to move to ("shooting_near" or "shooting_far")
     * @return String describing the result of the smart shooting operation
     */
    public String executeSmartShooting(String shootingLocation) {
        return executeSmartShooting(shootingLocation, false, 0);
    }

    public String executeSmartShooting(String shootingLocation, boolean useCameraServo) {
        return executeSmartShooting(shootingLocation, useCameraServo, 0);
    }

    public String executeSmartShooting(String shootingLocation, int timeoutMS) {
        return executeSmartShooting(shootingLocation, false, timeoutMS);
     }

    public String executeSmartShooting(String shootingLocation, boolean useCameraServo, int timeoutMS) {
        try {
            // Move to shooting location
            MotionExecutor.MotionResult result = (timeoutMS > 0)? moveToLocation(shootingLocation, timeoutMS) : moveToLocation(shootingLocation);

            // Smart shooting logic
            if (result.success) {
                // Movement successful → auto shoot
                try {
                    shoot(useCameraServo);  // shoot() already includes alignment
                    return "Smart shooting completed successfully";
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return "Shooting interrupted";
                } catch (Exception e) {
                    return "Shooting failed: " + e.getMessage();
                }

            } else {
                // Movement interrupted → check if in shooting zone
                boolean inZone = isInShootingZone();

                if (inZone) {
                    // In zone → auto shoot
                    try {
                        shoot(useCameraServo);  // shoot() already includes alignment
                        return "Movement interrupted but in zone → Shooting completed";
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                        return "Zone shooting interrupted";
                    } catch (Exception e) {
                        return "Zone shooting failed: " + e.getMessage();
                    }

                } else {
                    // Not in zone → manual positioning required
                    return "Movement failed, not in zone → Manual positioning required";
                }
            }

        } catch (Exception e) {
            return "Smart shooting error: " + e.getMessage();
        }
    }

    // ========== ODOMETRY PERSISTENCE ==========

    private static final String ODOMETRY_FILE_PATH = "/sdcard/FIRST/robot_odometry.txt";

    /**
     * Result class for odometry restore operations
     */
    public static class OdometryRestoreResult {
        public final boolean success;
        public final String message;
        public final FieldPose restoredPose;

        public OdometryRestoreResult(boolean success, String message, FieldPose restoredPose) {
            this.success = success;
            this.message = message;
            this.restoredPose = restoredPose;
        }
    }

  
    /**
     * Saves current robot odometry to persistent storage at the end of autonomous
     *
     * @param hardwareMap Hardware map for accessing robot systems
     * @param baseMotion BaseMotion subsystem to get current position
     * @param autoCompletedSuccessfully Whether autonomous completed successfully or was interrupted/timed out
     * @return true if save was successful, false otherwise
     */
    public static boolean saveOdometryAtAutoEnd(HardwareMap hardwareMap, BaseMotion baseMotion, boolean autoCompletedSuccessfully) {
        try {
            FieldPose currentPose = baseMotion.getCurrentPose();

            File file = new File(ODOMETRY_FILE_PATH);
            file.getParentFile().mkdirs(); // Create directories if they don't exist

            try (FileWriter writer = new FileWriter(file)) {
                // Format: x,y,heading,timestamp,autoCompleted
                writer.write(String.format("%.6f,%.6f,%.6f,%d,%s",
                        currentPose.x, currentPose.y, currentPose.heading,
                        System.currentTimeMillis(), autoCompletedSuccessfully ? "true" : "false"));
            }

            return true;
        } catch (IOException e) {
            return false;
        }
    }

    /**
     * Loads robot odometry from persistent storage and restores position
     *
     * @param hardwareMap Hardware map for accessing robot systems
     * @param baseMotion BaseMotion subsystem to restore position to
     * @return OdometryRestoreResult with success status, message, and restored pose
     */
    public static OdometryRestoreResult loadOdometryFromAuto(HardwareMap hardwareMap, BaseMotion baseMotion) {
        try {
            File file = new File(ODOMETRY_FILE_PATH);
            if (!file.exists()) {
                return new OdometryRestoreResult(false, "No saved odometry file found", null);
            }

            try (Scanner scanner = new Scanner(file)) {
                if (!scanner.hasNextLine()) {
                    return new OdometryRestoreResult(false, "Odometry file is empty", null);
                }

                String line = scanner.nextLine();
                String[] parts = line.split(",");

                if (parts.length < 4) {
                    return new OdometryRestoreResult(false, "Invalid odometry file format", null);
                }

                double x = Double.parseDouble(parts[0]);
                double y = Double.parseDouble(parts[1]);
                double heading = Double.parseDouble(parts[2]);
                long timestamp = Long.parseLong(parts[3]);

                // Check if data is recent (within 5 minutes)
                long currentTime = System.currentTimeMillis();
                long ageMinutes = (currentTime - timestamp) / (1000 * 60);

                if (ageMinutes > 0.5) {
                    return new OdometryRestoreResult(false,
                            String.format("Odometry data too old (%d minutes)", ageMinutes), null);
                }

                FieldPose restoredPose = new FieldPose(x, y, heading);
                baseMotion.setReferencePointInitialPosition(restoredPose);

                // Include auto completion status in message if available
                String completionStatus = "";
                if (parts.length >= 5) {
                    boolean autoCompleted = "true".equals(parts[4]);
                    completionStatus = autoCompleted ? " (Auto completed)" : " (Auto interrupted/timeout)";
                }

                return new OdometryRestoreResult(true,
                        String.format("Restored from %d minutes ago%s", ageMinutes, completionStatus), restoredPose);

            }
        } catch (Exception e) {
            return new OdometryRestoreResult(false, "Error reading odometry: " + e.getMessage(), null);
        }
    }

    /**
     * Clears saved odometry data
     *
     * @return true if clear was successful, false otherwise
     */
    public static boolean clearSavedOdometry() {
        try {
            File file = new File(ODOMETRY_FILE_PATH);
            if (file.exists()) {
                return file.delete();
            }
            return true; // File doesn't exist, consider it cleared
        } catch (Exception e) {
            return false;
        }
    }
}
