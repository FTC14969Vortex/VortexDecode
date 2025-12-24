package org.firstinspires.ftc.teamcode.calibration.modules;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.calibration.BaseCalibration;
import org.firstinspires.ftc.teamcode.calibration.RobotConstants;
import org.firstinspires.ftc.teamcode.calibration.SmartDashboardManager;
import org.firstinspires.ftc.teamcode.calibration.MotionCalibrationAndDemo;

import org.firstinspires.ftc.teamcode.helper.GoBildaPinpointDriver;

/**
 * Odometry Direction Calibration Module
 * 
 * This module determines the correct encoder directions for the GoBilda Pinpoint
 * odometry sensor by testing actual robot movements against odometry readings.
 * 
 * CALIBRATION PROCESS:
 * 1. Reset odometry to zero position
 * 2. Command robot to move forward - check if X increases or decreases
 * 3. Command robot to move left - check if Y increases or decreases
 * 4. Determine correct encoder directions based on movement results
 * 5. Apply and save correct directions to configuration
 * 
 * ENCODER DIRECTION RULES:
 * - X encoder should INCREASE when robot moves FORWARD
 * - Y encoder should INCREASE when robot moves LEFT
 * - If opposite behavior observed, encoder direction should be REVERSED
 * 
 * REQUIRED EQUIPMENT:
 * - Large open space for robot movement (6+ feet in all directions)
 * - Markers to mark starting position
 * - Clear path for forward and left movements
 * 
 * SAFETY NOTES:
 * - Robot moves slowly during calibration (25% power max)
 * - Emergency stop available via gamepad
 * - Clear movement area of obstacles
 * - Movements are limited to 12 inches for safety
 */
public class OdometryDirectionCalibrationModule extends BaseCalibration {
    
    // Hardware
    private GoBildaPinpointDriver pinpoint;
    
    // Movement state
    private MovementState currentMovement = MovementState.IDLE;
    private double movementStartTime = 0;
    private Pose2D startPose = null;
    private Pose2D endPose = null;
    
    // Test mode selection for dropdown
    public enum TestMode {
        TEST_X_FORWARD,   // Move forward to test X encoder
        TEST_Y_LEFT       // Move left to test Y encoder
    }
    
    // Movement types
    private enum MovementState {
        IDLE,
        TESTING_X_FORWARD,
        TESTING_Y_LEFT
    }
    
    /**
     * Dashboard parameters for odometry direction calibration
     */
    @Config
    public static class _3_OdometryDirection {
        // ========== TEST CONTROL ==========
        public static TestMode TEST_MODE = TestMode.TEST_X_FORWARD;  // Select test to run
        public static boolean START_TEST = false;          // Click to start selected test
        
        // ========== TEST CONFIGURATION ==========
        public static double TEST_DURATION = 2.0;          // How long to move (seconds)
        public static double TEST_POWER = 0.25;            // Power level for test movements (25%)
        
        // ========== MOVEMENT RESULTS (AUTO-POPULATED) ==========
        public static double X_MOVEMENT_MEASURED = 0.0;    // Actual X change from odometry
        public static double Y_MOVEMENT_MEASURED = 0.0;    // Actual Y change from odometry
        
        // ========== DIRECTION ANALYSIS (AUTO-CALCULATED) ==========
        public static String X_DIRECTION_STATUS = "UNKNOWN";           // Current X direction analysis
        public static String Y_DIRECTION_STATUS = "UNKNOWN";           // Current Y direction analysis
        public static String X_DIRECTION_RECOMMENDATION = "UNKNOWN";   // Recommended X direction
        public static String Y_DIRECTION_RECOMMENDATION = "UNKNOWN";   // Recommended Y direction
        
        // ========== CURRENT CONFIGURATION ==========
        public static String CURRENT_X_DIRECTION = "UNKNOWN";          // Currently configured X direction
        public static String CURRENT_Y_DIRECTION = "UNKNOWN";          // Currently configured Y direction
        
        // ========== CALIBRATION ACTIONS ==========
        public static boolean APPLY_RECOMMENDED_DIRECTIONS = false;    // Apply recommended directions
        public static boolean RESET_CALIBRATION = false;               // Reset all calibration data
    }
    
    // ========== DASHBOARD MANAGER ==========
    private SmartDashboardManager dashboardManager;
    
    @Override
    protected void initializeCalibration() {
        // Get dashboard manager from MotionCalibrationAndDemo
        if (parentOpMode instanceof MotionCalibrationAndDemo) {
            dashboardManager = ((MotionCalibrationAndDemo) parentOpMode).getDashboardManager();
        }
        
        // Motors are already initialized in BaseCalibration from MotionCalibrationAndDemo
        // Motor directions are already set from RobotConstants via MotionExecutor
        
        // Initialize odometry sensor
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        
        // Set to run using encoders
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Update current direction status
        updateCurrentDirectionStatus();
    }
    
    @Override
    protected void startTest() {
        // Direction calibration uses manual test commands
        // No continuous test to start
    }
    
    @Override
    protected void updateTest() {
        // Handle test commands
        handleTestCommands();
        
        // Update movement state
        updateMovementState();
        
        // Handle configuration actions
        handleConfigurationActions();
        
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
        telemetry.addLine("🧭 ODOMETRY DIRECTION CALIBRATION");
        telemetry.addLine("Determine correct encoder directions for GoBilda Pinpoint");
        telemetry.addLine("");
        
        // ========== CURRENT CONFIGURATION ==========
        telemetry.addLine("📋 CURRENT CONFIGURATION:");
        telemetry.addData("X Encoder Direction", _3_OdometryDirection.CURRENT_X_DIRECTION);
        telemetry.addData("Y Encoder Direction", _3_OdometryDirection.CURRENT_Y_DIRECTION);
        telemetry.addLine("");
        
        // ========== TEST CONTROL ==========
        telemetry.addLine("🎮 TEST CONTROL:");
        telemetry.addData("Current State", currentMovement.toString());
        if (currentMovement != MovementState.IDLE) {
            telemetry.addData("Test Time", "%.1f / %.1f seconds", 
                (System.currentTimeMillis() - movementStartTime) / 1000.0,
                _3_OdometryDirection.TEST_DURATION);
        }
        telemetry.addLine("");
        telemetry.addLine("1. Select TEST_MODE from dropdown");
        telemetry.addLine("2. Set START_TEST = true");
        telemetry.addLine("3. Robot moves for " + _3_OdometryDirection.TEST_DURATION + " seconds");
        telemetry.addData("Selected Mode", _3_OdometryDirection.TEST_MODE);
        telemetry.addLine("");
        
        // ========== TEST RESULTS ==========
        telemetry.addLine("📊 TEST RESULTS:");
        telemetry.addData("X Movement", "%.1f inches | %s", 
            _3_OdometryDirection.X_MOVEMENT_MEASURED, _3_OdometryDirection.X_DIRECTION_STATUS);
        telemetry.addData("X Recommendation", _3_OdometryDirection.X_DIRECTION_RECOMMENDATION);
        telemetry.addLine("");
        telemetry.addData("Y Movement", "%.1f inches | %s", 
            _3_OdometryDirection.Y_MOVEMENT_MEASURED, _3_OdometryDirection.Y_DIRECTION_STATUS);
        telemetry.addData("Y Recommendation", _3_OdometryDirection.Y_DIRECTION_RECOMMENDATION);
        telemetry.addLine("");
        
        // ========== CALIBRATION ACTIONS ==========
        telemetry.addLine("⚙️ CALIBRATION ACTIONS:");
        telemetry.addData("Apply Recommendations", _3_OdometryDirection.APPLY_RECOMMENDED_DIRECTIONS ? "READY" : "false");
        telemetry.addData("Reset Calibration", _3_OdometryDirection.RESET_CALIBRATION ? "READY" : "false");
        telemetry.addLine("");
        
        // ========== CALIBRATION PROCEDURE ==========
        telemetry.addLine("📝 CALIBRATION PROCEDURE:");
        telemetry.addLine("1. Place robot in open space (6+ feet clear)");
        telemetry.addLine("2. Select TEST_X_FORWARD from TEST_MODE dropdown");
        telemetry.addLine("3. Set START_TEST = true (robot moves 2 sec)");
        telemetry.addLine("4. Check if X increased (✅) or decreased (❌)");
        telemetry.addLine("5. Select TEST_Y_LEFT, set START_TEST = true");
        telemetry.addLine("6. Check if Y increased (✅) or decreased (❌)");
        telemetry.addLine("7. Set APPLY_RECOMMENDED_DIRECTIONS = true");
        telemetry.addLine("8. Write down final values and update MotionConfig.java manually");
        telemetry.addLine("");
        
        // ========== DIRECTION RULES ==========
        telemetry.addLine("📐 DIRECTION RULES:");
        telemetry.addLine("• X encoder should INCREASE when moving FORWARD");
        telemetry.addLine("• Y encoder should INCREASE when moving LEFT");
        telemetry.addLine("• If opposite behavior → direction should be REVERSED");
        telemetry.addLine("");
        
        // ========== SAFETY REMINDERS ==========
        telemetry.addLine("⚠️ SAFETY:");
        telemetry.addLine("• Clear 6+ feet in all directions");
        telemetry.addLine("• Robot moves at 25% power for safety");
        telemetry.addLine("");
        
    }
    
    // ========== TEST COMMAND HANDLING ==========
    
    private void handleTestCommands() {
        // Process START button with selected mode
        if (currentMovement == MovementState.IDLE && _3_OdometryDirection.START_TEST) {
            _3_OdometryDirection.START_TEST = false;  // Reset immediately
            
            switch (_3_OdometryDirection.TEST_MODE) {
                case TEST_X_FORWARD:
                    startXDirectionTest();
                    break;
                case TEST_Y_LEFT:
                    startYDirectionTest();
                    break;
            }
        }
    }
    
    private void startXDirectionTest() {
        currentMovement = MovementState.TESTING_X_FORWARD;
        movementStartTime = System.currentTimeMillis();
        
        // Record starting pose
        pinpoint.update();
        startPose = pinpoint.getPosition();
        
        // Initialize results
        _3_OdometryDirection.X_MOVEMENT_MEASURED = 0.0;
        _3_OdometryDirection.X_DIRECTION_STATUS = "TESTING...";
        
        // Start forward movement
        setMotorPowers(_3_OdometryDirection.TEST_POWER, _3_OdometryDirection.TEST_POWER, 
                      _3_OdometryDirection.TEST_POWER, _3_OdometryDirection.TEST_POWER);
    }
    
    private void startYDirectionTest() {
        currentMovement = MovementState.TESTING_Y_LEFT;
        movementStartTime = System.currentTimeMillis();
        
        // Record starting pose
        pinpoint.update();
        startPose = pinpoint.getPosition();
        
        // Initialize results
        _3_OdometryDirection.Y_MOVEMENT_MEASURED = 0.0;
        _3_OdometryDirection.Y_DIRECTION_STATUS = "TESTING...";
        
        // Start left strafe movement
        double power = _3_OdometryDirection.TEST_POWER;
        setMotorPowers(-power, power, power, -power);
    }
    
    private void updateMovementState() {
        if (currentMovement == MovementState.IDLE) {
            return;
        }
        
        // Stop after specified test duration
        double elapsedTime = (System.currentTimeMillis() - movementStartTime) / 1000.0;
        if (elapsedTime >= _3_OdometryDirection.TEST_DURATION) {
            completeCurrentTest();
            return;
        }
    }
    
    private void completeCurrentTest() {
        // Stop motors
        stopAllMotors();
        
        // Record ending pose
        pinpoint.update();
        endPose = pinpoint.getPosition();
        
        // Calculate movement and analyze direction
        if (currentMovement == MovementState.TESTING_X_FORWARD) {
            completeXDirectionTest();
        } else if (currentMovement == MovementState.TESTING_Y_LEFT) {
            completeYDirectionTest();
        }
        
        // Reset movement state
        currentMovement = MovementState.IDLE;
    }
    
    private void completeXDirectionTest() {
        double actualXChange = endPose.getX(DistanceUnit.INCH) - startPose.getX(DistanceUnit.INCH);
        _3_OdometryDirection.X_MOVEMENT_MEASURED = actualXChange;
        
        // Analyze direction
        if (Math.abs(actualXChange) < 1.0) {
            _3_OdometryDirection.X_DIRECTION_STATUS = "⚠️ NO MOVEMENT";
            _3_OdometryDirection.X_DIRECTION_RECOMMENDATION = "CHECK HARDWARE";
        } else if (actualXChange > 0) {
            _3_OdometryDirection.X_DIRECTION_STATUS = "✅ CORRECT";
            _3_OdometryDirection.X_DIRECTION_RECOMMENDATION = "FORWARD";
        } else {
            _3_OdometryDirection.X_DIRECTION_STATUS = "❌ WRONG";
            _3_OdometryDirection.X_DIRECTION_RECOMMENDATION = "REVERSED";
        }
        
        if (dashboardManager != null) {
            dashboardManager.markConfigChanged();
        }
    }
    
    private void completeYDirectionTest() {
        double actualYChange = endPose.getY(DistanceUnit.INCH) - startPose.getY(DistanceUnit.INCH);
        _3_OdometryDirection.Y_MOVEMENT_MEASURED = actualYChange;
        
        // Analyze direction
        if (Math.abs(actualYChange) < 1.0) {
            _3_OdometryDirection.Y_DIRECTION_STATUS = "⚠️ NO MOVEMENT";
            _3_OdometryDirection.Y_DIRECTION_RECOMMENDATION = "CHECK HARDWARE";
        } else if (actualYChange > 0) {
            _3_OdometryDirection.Y_DIRECTION_STATUS = "✅ CORRECT";
            _3_OdometryDirection.Y_DIRECTION_RECOMMENDATION = "FORWARD";
        } else {
            _3_OdometryDirection.Y_DIRECTION_STATUS = "❌ WRONG";
            _3_OdometryDirection.Y_DIRECTION_RECOMMENDATION = "REVERSED";
        }
        
        if (dashboardManager != null) {
            dashboardManager.markConfigChanged();
        }
    }
    
    // ========== CONFIGURATION ACTIONS ==========
    
    private void handleConfigurationActions() {
        if (_3_OdometryDirection.APPLY_RECOMMENDED_DIRECTIONS) {
            applyRecommendedDirections();
            _3_OdometryDirection.APPLY_RECOMMENDED_DIRECTIONS = false;
        }
        
        if (_3_OdometryDirection.RESET_CALIBRATION) {
            resetCalibration();
            _3_OdometryDirection.RESET_CALIBRATION = false;
        }
    }
    
    private void applyRecommendedDirections() {
        try {
            GoBildaPinpointDriver.EncoderDirection xDir = GoBildaPinpointDriver.EncoderDirection.FORWARD;
            GoBildaPinpointDriver.EncoderDirection yDir = GoBildaPinpointDriver.EncoderDirection.FORWARD;
            
            // Parse recommendations
            if ("REVERSED".equals(_3_OdometryDirection.X_DIRECTION_RECOMMENDATION)) {
                xDir = GoBildaPinpointDriver.EncoderDirection.REVERSED;
            }
            if ("REVERSED".equals(_3_OdometryDirection.Y_DIRECTION_RECOMMENDATION)) {
                yDir = GoBildaPinpointDriver.EncoderDirection.REVERSED;
            }
            
            // Apply to Pinpoint sensor
            pinpoint.setEncoderDirections(xDir, yDir);
            
            // Update status
            updateCurrentDirectionStatus();
            
            telemetry.addLine("✅ Applied recommended encoder directions");
            telemetry.addData("X Direction", _3_OdometryDirection.X_DIRECTION_RECOMMENDATION);
            telemetry.addData("Y Direction", _3_OdometryDirection.Y_DIRECTION_RECOMMENDATION);
            
        } catch (Exception e) {
            telemetry.addLine("❌ Failed to apply encoder directions");
            telemetry.addLine("Error: " + e.getMessage());
        }
    }
    
    
    private void resetCalibration() {
        _3_OdometryDirection.X_MOVEMENT_MEASURED = 0.0;
        _3_OdometryDirection.Y_MOVEMENT_MEASURED = 0.0;
        _3_OdometryDirection.X_DIRECTION_STATUS = "UNKNOWN";
        _3_OdometryDirection.Y_DIRECTION_STATUS = "UNKNOWN";
        _3_OdometryDirection.X_DIRECTION_RECOMMENDATION = "UNKNOWN";
        _3_OdometryDirection.Y_DIRECTION_RECOMMENDATION = "UNKNOWN";
        stopAllMotors();
        currentMovement = MovementState.IDLE;
    }
    
    private void updateCurrentDirectionStatus() {
        // Read current encoder directions from RobotConstants
        _3_OdometryDirection.CURRENT_X_DIRECTION = RobotConstants.ODOMETRY_X_ENCODER_REVERSED ? "REVERSED" : "FORWARD";
        _3_OdometryDirection.CURRENT_Y_DIRECTION = RobotConstants.ODOMETRY_Y_ENCODER_REVERSED ? "REVERSED" : "FORWARD";
    }
    
    // ========== HELPER METHODS ==========
    
    private void setRunMode(DcMotor.RunMode mode) {
        frontLeftDrive.setMode(mode);
        frontRightDrive.setMode(mode);
        backLeftDrive.setMode(mode);
        backRightDrive.setMode(mode);
    }
    
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
        _3_OdometryDirection.TEST_MODE = TestMode.TEST_X_FORWARD;
        _3_OdometryDirection.START_TEST = false;
        
        _3_OdometryDirection.TEST_DURATION = 2.0;
        _3_OdometryDirection.TEST_POWER = 0.25;
        
        resetCalibration();
        
        _3_OdometryDirection.APPLY_RECOMMENDED_DIRECTIONS = false;
        _3_OdometryDirection.RESET_CALIBRATION = false;
    }
    
    @Override
    public String getCalibrationName() {
        return "Odometry Encoder Direction Calibration";
    }
    
    @Override
    public String getCalibrationDescription() {
        return "Determine correct encoder directions for GoBilda Pinpoint odometry sensor";
    }
}
