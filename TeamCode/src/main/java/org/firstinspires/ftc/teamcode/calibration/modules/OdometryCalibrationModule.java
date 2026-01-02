package org.firstinspires.ftc.teamcode.calibration.modules;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.calibration.BaseCalibration;
import org.firstinspires.ftc.teamcode.calibration.CalibrationCoefficients;
import org.firstinspires.ftc.teamcode.calibration.SmartDashboardManager;
import org.firstinspires.ftc.teamcode.calibration.MotionCalibrationAndDemo;

import org.firstinspires.ftc.teamcode.calibration.RobotConstants;

/**
 * Odometry Calibration Module
 * 
 * This module calibrates the accuracy of Pinpoint odometry readings by comparing
 * Pinpoint measurements with actual physical distances measured manually.
 * 
 * CALIBRATION PROCESS:
 * 1. Place robot at known starting position
 * 2. Use Dashboard to command specific movements (forward/back/strafe)
 * 3. Robot stops when Pinpoint reads target distance
 * 4. Measure actual distance traveled with tape measure
 * 5. Compare Pinpoint readings vs actual measurements
 * 6. Calculate correction factors for each axis
 * 7. Save correction factors to CalibrationCoefficients.java
 * 
 * REQUIRED EQUIPMENT:
 * - Tape measure (25+ feet recommended)
 * - Large open space for robot movement
 * - Markers to mark starting/ending positions
 * 
 * SAFETY NOTES:
 * - Robot moves slowly during calibration (0.3 power max)
 * - Emergency stop available via gamepad
 * - Clear movement area of obstacles
 */
public class OdometryCalibrationModule extends BaseCalibration {
    
    // Movement state
    private MovementState currentMovement = MovementState.IDLE;
    private double movementStartTime = 0;
    private double targetDistance = 0;
    private double targetRotation = 0;
    
    // Odometry tracking for distance-based stopping
    private double startOdometryX = 0;
    private double startOdometryY = 0;
    private double startOdometryHeading = 0;
    private double lastHeading = 0;  // Track last heading for incremental rotation
    private double accumulatedRotation = 0;  // Total rotation accumulated
    
    // Movement types
    private enum MovementState {
        IDLE,
        MOVING_FORWARD,
        MOVING_BACKWARD, 
        MOVING_LEFT,
        MOVING_RIGHT,
        ROTATING_LEFT,
        ROTATING_RIGHT
    }
    
    /**
     * Dashboard parameters for odometry calibration
     */
    @Config
    public static class _4_OdometryScale {
        // ========== MOVEMENT CONFIGURATION ==========
        public static double TARGET_DISTANCE = 24.0;          // Distance to move (user configurable)
        public static double ROTATION_DEGREES = 90.0;         // Rotation angle for heading calibration
        
        // ========== MOVEMENT COMMANDS ==========
        public static boolean MOVE_FORWARD = false;           // Command: Move forward TARGET_DISTANCE
        public static boolean MOVE_BACKWARD = false;          // Command: Move backward TARGET_DISTANCE  
        public static boolean MOVE_LEFT = false;              // Command: Move left TARGET_DISTANCE
        public static boolean MOVE_RIGHT = false;             // Command: Move right TARGET_DISTANCE
        public static boolean ROTATE_LEFT = false;            // Command: Rotate left ROTATION_DEGREES
        public static boolean ROTATE_RIGHT = false;           // Command: Rotate right ROTATION_DEGREES
        
        // ========== ACTUAL MEASUREMENTS (USER INPUT) ==========
        public static double ACTUAL_FORWARD_DISTANCE = 0.0;   // Measured with tape measure
        public static double ACTUAL_BACKWARD_DISTANCE = 0.0;  // Measured with tape measure
        public static double ACTUAL_LEFT_DISTANCE = 0.0;      // Measured with tape measure
        public static double ACTUAL_RIGHT_DISTANCE = 0.0;     // Measured with tape measure
        public static double ACTUAL_ROTATION_DEGREES = 0.0;   // Measured rotation angle
        
        // ========== ODOMETRY READINGS (AUTO-POPULATED) ==========
        public static double ODOMETRY_FORWARD_DISTANCE = 0.0;   // Odometry reading for forward
        public static double ODOMETRY_BACKWARD_DISTANCE = 0.0;  // Odometry reading for backward
        public static double ODOMETRY_LEFT_DISTANCE = 0.0;      // Odometry reading for left
        public static double ODOMETRY_RIGHT_DISTANCE = 0.0;     // Odometry reading for right
        public static double ODOMETRY_ROTATION_DEGREES = 0.0;   // Odometry reading for rotation
        
        // ========== CALCULATED SCALE FACTORS ==========
        public static double CALCULATED_X_SCALE = 1.0;        // X scale = avg(forward,backward) actual/odometry
        public static double CALCULATED_Y_SCALE = 1.0;        // Y scale = avg(left,right) actual/odometry  
        public static double CALCULATED_HEADING_SCALE = 1.0;  // Heading scale = actual_rotation/odometry_rotation
        
        // ========== CALCULATION CONTROL ==========
        public static boolean CALCULATE_SCALES = false;       // Button: Calculate scale factors from measurements
    }
    
    // ========== DASHBOARD MANAGER ==========
    private SmartDashboardManager dashboardManager;
    
    @Override
    protected void initializeCalibration() {
        // Get dashboard manager from MotionCalibrationAndDemo
        if (parentOpMode instanceof MotionCalibrationAndDemo) {
            dashboardManager = ((MotionCalibrationAndDemo) parentOpMode).getDashboardManager();
        }
        
        // CRITICAL: Reset odometry to (0,0,0) at start of calibration
        // This ensures all distance measurements are relative to a known origin
        motionExecutor.resetToFieldOrigin();
        
        // Motors are already initialized in BaseCalibration from MotionCalibrationAndDemo
        // Motor directions are already set from RobotConstants via MotionExecutor
    }
    
    @Override
    protected void startTest() {
        // Odometry calibration uses manual movement commands
        // No continuous test to start
    }
    
    @Override
    protected void updateTest() {
        // Handle movement commands
        handleMovementCommands();
        
        // Update movement state
        updateMovementState();
        
        // Update dashboard config if needed (critical for synchronization)
        if (dashboardManager != null) {
            dashboardManager.updateConfigIfNeeded();
        }
    }
    
    @Override
    protected void stopTest() {
        // Stop all motors
        stopAllMotors();
        currentMovement = MovementState.IDLE;
    }
    
    @Override
    public void displayStatus() {
        telemetry.addLine("🎯 PINPOINT ODOMETRY CALIBRATION");
        telemetry.addLine("Compare Pinpoint readings with actual tape measurements");
        telemetry.addLine("");
        
        // ========== CURRENT SCALE FACTORS ==========
        telemetry.addLine("📊 CURRENT SCALE FACTORS (CalibrationCoefficients.java):");
        telemetry.addData("X Scale", "%.4f", CalibrationCoefficients.ODOMETRY_X_SCALE);
        telemetry.addData("Y Scale", "%.4f", CalibrationCoefficients.ODOMETRY_Y_SCALE);
        telemetry.addData("Heading Scale", "%.4f", CalibrationCoefficients.ODOMETRY_HEADING_SCALE);
        telemetry.addLine("");
        
        // ========== MOVEMENT CONFIGURATION ==========
        telemetry.addLine("⚙️ MOVEMENT CONFIGURATION:");
        telemetry.addData("Target Distance", "%.1f inches", _4_OdometryScale.TARGET_DISTANCE);
        telemetry.addData("Rotation Angle", "%.1f degrees", _4_OdometryScale.ROTATION_DEGREES);
        telemetry.addLine("");
        
        // ========== MOVEMENT COMMANDS ==========
        telemetry.addLine("🎮 MOVEMENT COMMANDS:");
        telemetry.addData("Current State", currentMovement.toString());
        if (currentMovement != MovementState.IDLE) {
            if (currentMovement == MovementState.ROTATING_LEFT || currentMovement == MovementState.ROTATING_RIGHT) {
                telemetry.addData("Target Rotation", "%.1f degrees", targetRotation);
            } else {
                telemetry.addData("Target Distance", "%.1f inches", targetDistance);
            }
            telemetry.addData("Movement Time", "%.1f seconds", 
                (System.currentTimeMillis() - movementStartTime) / 1000.0);
        }
        telemetry.addLine("Set command = true to start movement:");
        telemetry.addData("MOVE_FORWARD", _4_OdometryScale.MOVE_FORWARD ? "READY" : "false");
        telemetry.addData("MOVE_BACKWARD", _4_OdometryScale.MOVE_BACKWARD ? "READY" : "false");
        telemetry.addData("MOVE_LEFT", _4_OdometryScale.MOVE_LEFT ? "READY" : "false");
        telemetry.addData("MOVE_RIGHT", _4_OdometryScale.MOVE_RIGHT ? "READY" : "false");
        telemetry.addData("ROTATE_LEFT", _4_OdometryScale.ROTATE_LEFT ? "READY" : "false");
        telemetry.addData("ROTATE_RIGHT", _4_OdometryScale.ROTATE_RIGHT ? "READY" : "false");
        telemetry.addLine("");
        
        // ========== MEASUREMENT RESULTS ==========
        telemetry.addLine("📏 MEASUREMENT RESULTS:");
        telemetry.addLine("Robot stops when odometry reads target, measure actual:");
        
        // Forward/Backward (X-axis)
        telemetry.addData("Forward - Odometry", "%.1f inches", _4_OdometryScale.ODOMETRY_FORWARD_DISTANCE);
        telemetry.addData("Forward - Actual", "%.1f inches", _4_OdometryScale.ACTUAL_FORWARD_DISTANCE);
        
        telemetry.addData("Backward - Odometry", "%.1f inches", _4_OdometryScale.ODOMETRY_BACKWARD_DISTANCE);
        telemetry.addData("Backward - Actual", "%.1f inches", _4_OdometryScale.ACTUAL_BACKWARD_DISTANCE);
        
        // Left/Right Strafe (Y-axis)
        telemetry.addData("Left - Odometry", "%.1f inches", _4_OdometryScale.ODOMETRY_LEFT_DISTANCE);
        telemetry.addData("Left - Actual", "%.1f inches", _4_OdometryScale.ACTUAL_LEFT_DISTANCE);
        
        telemetry.addData("Right - Odometry", "%.1f inches", _4_OdometryScale.ODOMETRY_RIGHT_DISTANCE);
        telemetry.addData("Right - Actual", "%.1f inches", _4_OdometryScale.ACTUAL_RIGHT_DISTANCE);
        
        // Rotation (Heading)
        telemetry.addData("Rotation - Odometry", "%.1f degrees", _4_OdometryScale.ODOMETRY_ROTATION_DEGREES);
        telemetry.addData("Rotation - Actual", "%.1f degrees", _4_OdometryScale.ACTUAL_ROTATION_DEGREES);
        telemetry.addLine("");
        
        // ========== CALCULATED SCALE FACTORS ==========
        telemetry.addLine("🧮 CALCULATED SCALE FACTORS:");
        telemetry.addData("X Scale (Forward/Back)", "%.4f", _4_OdometryScale.CALCULATED_X_SCALE);
        telemetry.addData("Y Scale (Left/Right)", "%.4f", _4_OdometryScale.CALCULATED_Y_SCALE);
        telemetry.addData("Heading Scale", "%.4f", _4_OdometryScale.CALCULATED_HEADING_SCALE);
        telemetry.addData("CALCULATE_SCALES", _4_OdometryScale.CALCULATE_SCALES ? "CALCULATING..." : "Click to calculate");
        telemetry.addLine("");
        
        // ========== CALIBRATION INSTRUCTIONS ==========
        telemetry.addLine("📋 CALIBRATION PROCEDURE:");
        telemetry.addLine("1. Set TARGET_DISTANCE (e.g., 24 inches)");
        telemetry.addLine("2. Set MOVE_FORWARD = true");
        telemetry.addLine("3. Robot moves TARGET_DISTANCE according to odometry");
        telemetry.addLine("4. Measure ACTUAL distance with tape measure");
        telemetry.addLine("5. Enter measured distance in ACTUAL_FORWARD_DISTANCE");
        telemetry.addLine("6. Repeat for backward, left, right, rotate movements");
        telemetry.addLine("7. Click CALCULATE_SCALES = true");
        telemetry.addLine("8. Copy calculated scale factors to CalibrationCoefficients.java");
        telemetry.addLine("");
        
        // ========== SAFETY REMINDERS ==========
        telemetry.addLine("⚠️ SAFETY:");
        telemetry.addLine("• Clear 6+ feet in all directions");
        telemetry.addLine("• Robot moves at 30% power for safety");
        telemetry.addLine("");
        
    }
    
    // ========== MOVEMENT CONTROL ==========
    
    private void handleMovementCommands() {
        // Handle scale calculation
        if (_4_OdometryScale.CALCULATE_SCALES) {
            calculateScaleFactors();
            _4_OdometryScale.CALCULATE_SCALES = false;
        }
        
        // Only process new commands if not currently moving
        if (currentMovement == MovementState.IDLE) {
            if (_4_OdometryScale.MOVE_FORWARD) {
                startMovement(MovementState.MOVING_FORWARD, _4_OdometryScale.TARGET_DISTANCE);
                _4_OdometryScale.MOVE_FORWARD = false;
            } else if (_4_OdometryScale.MOVE_BACKWARD) {
                startMovement(MovementState.MOVING_BACKWARD, _4_OdometryScale.TARGET_DISTANCE);
                _4_OdometryScale.MOVE_BACKWARD = false;
            } else if (_4_OdometryScale.MOVE_LEFT) {
                startMovement(MovementState.MOVING_LEFT, _4_OdometryScale.TARGET_DISTANCE);
                _4_OdometryScale.MOVE_LEFT = false;
            } else if (_4_OdometryScale.MOVE_RIGHT) {
                startMovement(MovementState.MOVING_RIGHT, _4_OdometryScale.TARGET_DISTANCE);
                _4_OdometryScale.MOVE_RIGHT = false;
            } else if (_4_OdometryScale.ROTATE_LEFT) {
                startRotation(MovementState.ROTATING_LEFT, _4_OdometryScale.ROTATION_DEGREES);
                _4_OdometryScale.ROTATE_LEFT = false;
            } else if (_4_OdometryScale.ROTATE_RIGHT) {
                startRotation(MovementState.ROTATING_RIGHT, _4_OdometryScale.ROTATION_DEGREES);
                _4_OdometryScale.ROTATE_RIGHT = false;
            }
        }
    }
    
    private void startMovement(MovementState movement, double distance) {
        currentMovement = movement;
        targetDistance = distance;
        movementStartTime = System.currentTimeMillis();
        
        // CRITICAL: Reset odometry to (0,0,0) before each test
        // This ensures robot starts at known position and heading=0 for each movement
        motionExecutor.resetToFieldOrigin();
        
        // Record starting odometry position (will be 0,0,0)
        motionExecutor.updateState();
        startOdometryX = 0;
        startOdometryY = 0;
        startOdometryHeading = 0;
        
        // Start movement at safe speed (30% power)
        double power = 0.2;
        
        switch (movement) {
            case MOVING_FORWARD:
                setMotorPowers(power, power, power, power);
                break;
            case MOVING_BACKWARD:
                setMotorPowers(-power, -power, -power, -power);
                break;
            case MOVING_LEFT:
                setMotorPowers(-power, power, power, -power);
                break;
            case MOVING_RIGHT:
                setMotorPowers(power, -power, -power, power);
                break;
        }
    }
    
    private void startRotation(MovementState movement, double degrees) {
        currentMovement = movement;
        targetRotation = degrees;
        movementStartTime = System.currentTimeMillis();
        
        // CRITICAL: Reset odometry to (0,0,0) before each test
        motionExecutor.resetToFieldOrigin();
        
        // Record starting odometry position (will be 0,0,0)
        motionExecutor.updateState();
        startOdometryX = 0;
        startOdometryY = 0;
        startOdometryHeading = 0;
        lastHeading = 0;
        accumulatedRotation = 0;
        
        // Start rotation at safe speed (25% power)
        double power = 0.25;
        
        switch (movement) {
            case ROTATING_LEFT:
                // Left rotation: left wheels backward, right wheels forward
                setMotorPowers(-power, power, -power, power);
                break;
            case ROTATING_RIGHT:
                // Right rotation: left wheels forward, right wheels backward
                setMotorPowers(power, -power, power, -power);
                break;
        }
    }
    
    private void updateMovementState() {
        if (currentMovement == MovementState.IDLE) {
            return;
        }
        
        // Update odometry and MotionState pose
        // CRITICAL: Call updateState() to update MotionState with latest odometry
        motionExecutor.updateState();
        double currentX = motionExecutor.getMotionState().getX();
        double currentY = motionExecutor.getMotionState().getY();
        double currentHeading = motionExecutor.getMotionState().getHeading();
        
        // Calculate distance/rotation traveled based on movement direction
        double distanceTraveled = 0;
        double rotationTraveled = 0;
        boolean isRotation = false;
        
        switch (currentMovement) {
            case MOVING_FORWARD:
            case MOVING_BACKWARD:
                // Forward/backward: measure X displacement (robot starts at heading=0)
                distanceTraveled = Math.abs(currentX - startOdometryX);
                break;
            case MOVING_LEFT:
            case MOVING_RIGHT:
                // Left/right strafe: measure Y displacement (robot starts at heading=0)
                distanceTraveled = Math.abs(currentY - startOdometryY);
                break;
            case ROTATING_LEFT:
            case ROTATING_RIGHT:
                // Accumulate rotation incrementally to handle >360° rotations
                double headingDelta = currentHeading - lastHeading;
                // Normalize delta to [-180, 180] for this increment
                while (headingDelta > 180) headingDelta -= 360;
                while (headingDelta < -180) headingDelta += 360;
                // Add to accumulated rotation
                accumulatedRotation += headingDelta;
                lastHeading = currentHeading;
                rotationTraveled = Math.abs(accumulatedRotation);
                isRotation = true;
                break;
        }
        
        // Stop when target reached (with small tolerance)
        boolean targetReached = false;
        if (isRotation) {
            targetReached = rotationTraveled >= targetRotation * 0.95;  // 95% of target (both in degrees now!)
        } else {
            targetReached = distanceTraveled >= targetDistance * 0.95;  // 95% of target
        }
        
        if (targetReached) {
            completeMovement();
            return;
        }
        
        // Safety timeout: stop after 10 seconds regardless
        double elapsedTime = (System.currentTimeMillis() - movementStartTime) / 1000.0;
        if (elapsedTime > 10.0) {
            if (isRotation) {
                telemetry.addLine("⚠️ WARNING: Rotation timed out after 10 seconds");
                telemetry.addLine(String.format("Rotation: %.1f degrees (target: %.1f)", 
                    rotationTraveled, targetRotation));  // 🔧 FIX: Both already in degrees
            } else {
                telemetry.addLine("⚠️ WARNING: Movement timed out after 10 seconds");
                telemetry.addLine(String.format("Distance traveled: %.1f inches (target: %.1f)", 
                    distanceTraveled, targetDistance));
            }
            completeMovement();
            return;
        }
    }
    
    private void completeMovement() {
        // Stop motors
        stopAllMotors();
        
        // Get final odometry reading from Pinpoint
        // CRITICAL: Call updateState() to get latest pose
        motionExecutor.updateState();
        double currentX = motionExecutor.getMotionState().getX();
        double currentY = motionExecutor.getMotionState().getY();
        double currentHeading = motionExecutor.getMotionState().getHeading();
        
        // Calculate distance/rotation traveled according to odometry
        double odometryDistance = 0;
        double odometryRotation = 0;
        boolean isRotation = false;
        
        switch (currentMovement) {
            case MOVING_FORWARD:
            case MOVING_BACKWARD:
                // Forward/backward: use X displacement (robot started at heading=0)
                odometryDistance = Math.abs(currentX - startOdometryX);
                break;
            case MOVING_LEFT:
            case MOVING_RIGHT:
                // Left/right strafe: use Y displacement (robot started at heading=0)
                odometryDistance = Math.abs(currentY - startOdometryY);
                break;
            case ROTATING_LEFT:
            case ROTATING_RIGHT:
                // Use the accumulated rotation (already calculated incrementally)
                odometryRotation = accumulatedRotation;
                isRotation = true;
                break;
        }
        
        // Validate odometry reading before storing
        if (!isRotation && odometryDistance < 0.1) {
            telemetry.addLine("❌ ERROR: Odometry distance too small (" + String.format("%.2f", odometryDistance) + " inches)");
            telemetry.addLine("Robot may not have moved or odometry may not be updating");
            telemetry.addLine("Check odometry connection and try again");
            currentMovement = MovementState.IDLE;
            return;
        }
        
        if (isRotation && Math.abs(odometryRotation) < 1.0) {  // 🔧 FIX: Already in degrees
            telemetry.addLine("❌ ERROR: Odometry rotation too small (" + String.format("%.1f", Math.abs(odometryRotation)) + " degrees)");
            telemetry.addLine("Robot may not have rotated or odometry may not be updating");
            telemetry.addLine("Check odometry connection and try again");
            currentMovement = MovementState.IDLE;
            return;
        }
        
        // Store odometry reading and provide immediate feedback
        switch (currentMovement) {
            case MOVING_FORWARD:
                _4_OdometryScale.ODOMETRY_FORWARD_DISTANCE = odometryDistance;
                if (dashboardManager != null) {
                    dashboardManager.markConfigChanged();
                }
                telemetry.addLine("✅ Forward movement completed!");
                telemetry.addLine(String.format("📊 Odometry reading: %.2f inches", odometryDistance));
                break;
            case MOVING_BACKWARD:
                _4_OdometryScale.ODOMETRY_BACKWARD_DISTANCE = odometryDistance;
                if (dashboardManager != null) {
                    dashboardManager.markConfigChanged();
                }
                telemetry.addLine("✅ Backward movement completed!");
                telemetry.addLine(String.format("📊 Odometry reading: %.2f inches", odometryDistance));
                break;
            case MOVING_LEFT:
                _4_OdometryScale.ODOMETRY_LEFT_DISTANCE = odometryDistance;
                if (dashboardManager != null) {
                    dashboardManager.markConfigChanged();
                }
                telemetry.addLine("✅ Left strafe completed!");
                telemetry.addLine(String.format("📊 Odometry reading: %.2f inches", odometryDistance));
                break;
            case MOVING_RIGHT:
                _4_OdometryScale.ODOMETRY_RIGHT_DISTANCE = odometryDistance;
                if (dashboardManager != null) {
                    dashboardManager.markConfigChanged();
                }
                telemetry.addLine("✅ Right strafe completed!");
                telemetry.addLine(String.format("📊 Odometry reading: %.2f inches", odometryDistance));
                break;
            case ROTATING_LEFT:
            case ROTATING_RIGHT:
                _4_OdometryScale.ODOMETRY_ROTATION_DEGREES = Math.abs(odometryRotation);  // Store absolute value
                if (dashboardManager != null) {
                    dashboardManager.markConfigChanged();
                }
                telemetry.addLine("✅ Rotation completed!");
                telemetry.addLine(String.format("📊 Odometry reading: %.1f degrees (%s)", 
                    Math.abs(odometryRotation), 
                    odometryRotation > 0 ? "CCW/Left" : "CW/Right"));
                break;
        }
        
        // 🔧 FIX: Force immediate telemetry update to refresh dashboard
        telemetry.addLine("");
        telemetry.addLine("🔄 Dashboard values updated! Check the measurement results above.");
        telemetry.update();
        
        // Reset movement state
        currentMovement = MovementState.IDLE;
    }
    
    private void calculateScaleFactors() {
        // Calculate X scale factor (average of forward and backward)
        double xScaleForward = 0;
        double xScaleBackward = 0;
        int xScaleCount = 0;
        
        if (_4_OdometryScale.ODOMETRY_FORWARD_DISTANCE > 0.1 && _4_OdometryScale.ACTUAL_FORWARD_DISTANCE > 0) {
            xScaleForward = _4_OdometryScale.ACTUAL_FORWARD_DISTANCE / _4_OdometryScale.ODOMETRY_FORWARD_DISTANCE;
            xScaleCount++;
        }
        
        if (_4_OdometryScale.ODOMETRY_BACKWARD_DISTANCE > 0.1 && _4_OdometryScale.ACTUAL_BACKWARD_DISTANCE > 0) {
            xScaleBackward = _4_OdometryScale.ACTUAL_BACKWARD_DISTANCE / _4_OdometryScale.ODOMETRY_BACKWARD_DISTANCE;
            xScaleCount++;
        }
        
        if (xScaleCount > 0) {
            _4_OdometryScale.CALCULATED_X_SCALE = (xScaleForward + xScaleBackward) / xScaleCount;
        }
        
        // Calculate Y scale factor (average of left and right)
        double yScaleLeft = 0;
        double yScaleRight = 0;
        int yScaleCount = 0;
        
        if (_4_OdometryScale.ODOMETRY_LEFT_DISTANCE > 0.1 && _4_OdometryScale.ACTUAL_LEFT_DISTANCE > 0) {
            yScaleLeft = _4_OdometryScale.ACTUAL_LEFT_DISTANCE / _4_OdometryScale.ODOMETRY_LEFT_DISTANCE;
            yScaleCount++;
        }
        
        if (_4_OdometryScale.ODOMETRY_RIGHT_DISTANCE > 0.1 && _4_OdometryScale.ACTUAL_RIGHT_DISTANCE > 0) {
            yScaleRight = _4_OdometryScale.ACTUAL_RIGHT_DISTANCE / _4_OdometryScale.ODOMETRY_RIGHT_DISTANCE;
            yScaleCount++;
        }
        
        if (yScaleCount > 0) {
            _4_OdometryScale.CALCULATED_Y_SCALE = (yScaleLeft + yScaleRight) / yScaleCount;
        }
        
        // Calculate heading scale factor
        if (_4_OdometryScale.ODOMETRY_ROTATION_DEGREES > 1.0 && _4_OdometryScale.ACTUAL_ROTATION_DEGREES > 0) {
            _4_OdometryScale.CALCULATED_HEADING_SCALE = _4_OdometryScale.ACTUAL_ROTATION_DEGREES / _4_OdometryScale.ODOMETRY_ROTATION_DEGREES;
        }
        
        // Force dashboard to update with calculated scale factors
        if (dashboardManager != null) {
            dashboardManager.markConfigChanged();
        }
    }
    
    // ========== HELPER METHODS ==========
    
    private void setMotorPowers(double fl, double fr, double bl, double br) {
        // Use MotionExecutor to handle motor control properly
        motionExecutor.setMotorPowers(fl, fr, bl, br);
    }
    
    protected void stopAllMotors() {
        setMotorPowers(0, 0, 0, 0);
    }
    
    
    @Override
    public void resetParameters() {
        // Reset all parameters to defaults
        _4_OdometryScale.TARGET_DISTANCE = 24.0;
        _4_OdometryScale.ROTATION_DEGREES = 90.0;
        
        _4_OdometryScale.MOVE_FORWARD = false;
        _4_OdometryScale.MOVE_BACKWARD = false;
        _4_OdometryScale.MOVE_LEFT = false;
        _4_OdometryScale.MOVE_RIGHT = false;
        _4_OdometryScale.ROTATE_LEFT = false;
        _4_OdometryScale.ROTATE_RIGHT = false;
        
        _4_OdometryScale.ACTUAL_FORWARD_DISTANCE = 0.0;
        _4_OdometryScale.ACTUAL_BACKWARD_DISTANCE = 0.0;
        _4_OdometryScale.ACTUAL_LEFT_DISTANCE = 0.0;
        _4_OdometryScale.ACTUAL_RIGHT_DISTANCE = 0.0;
        _4_OdometryScale.ACTUAL_ROTATION_DEGREES = 0.0;
        
        _4_OdometryScale.ODOMETRY_FORWARD_DISTANCE = 0.0;
        _4_OdometryScale.ODOMETRY_BACKWARD_DISTANCE = 0.0;
        _4_OdometryScale.ODOMETRY_LEFT_DISTANCE = 0.0;
        _4_OdometryScale.ODOMETRY_RIGHT_DISTANCE = 0.0;
        _4_OdometryScale.ODOMETRY_ROTATION_DEGREES = 0.0;
        
        _4_OdometryScale.CALCULATED_X_SCALE = 1.0;
        _4_OdometryScale.CALCULATED_Y_SCALE = 1.0;
        _4_OdometryScale.CALCULATED_HEADING_SCALE = 1.0;
        
        _4_OdometryScale.CALCULATE_SCALES = false;
        
        // Stop any current movement
        stopAllMotors();
        currentMovement = MovementState.IDLE;
    }
    
    @Override
    public String getCalibrationName() {
        return "Odometry Distance Verification";
    }
    
    @Override
    public String getCalibrationDescription() {
        return "Calibrate odometry accuracy by comparing encoder readings with actual measured distances";
    }
}
