package org.firstinspires.ftc.teamcode.calibration.modules;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.calibration.BaseCalibration;
import org.firstinspires.ftc.teamcode.calibration.RobotConstants;
import org.firstinspires.ftc.teamcode.calibration.SmartDashboardManager;
import org.firstinspires.ftc.teamcode.calibration.MotionCalibrationAndDemo;
import org.firstinspires.ftc.teamcode.calibration.MotionCalibrationAndDemo.MotorSelection;

/**
 * Simple Motor Direction Test
 * 
 * Verifies that motors and encoders are wired correctly and spinning
 * in the expected directions. Critical for proper robot movement.
 * 
 * Simple workflow:
 * 1. Select motor from dropdown
 * 2. Click RUN TEST → Motor spins for 2 seconds
 * 3. Observe motor behavior and encoder readings
 * 4. Enter what you observed
 * 5. Check analysis results
 */
public class MotorDirectionCalibrationModule extends BaseCalibration {
    
    // ========== DASHBOARD PARAMETERS ==========
    
    @Config
    public static class _1_MotorDirection {
        // Current motor directions from RobotConstants.java
        public static String CURRENT_FRONT_LEFT = "REVERSED";
        public static String CURRENT_FRONT_RIGHT = "FORWARD";
        public static String CURRENT_BACK_LEFT = "REVERSED";
        public static String CURRENT_BACK_RIGHT = "FORWARD";
        
        // Motor selection (dropdown)
        public static MotorSelection MOTOR_SELECTION = MotorSelection.FRONT_LEFT;
        
        // Test command (2-second test)
        public static boolean RUN_TEST = false;
        public static boolean STOP_TEST = false;
        
        // Test results (Auto-detected)
        public static double START_ENCODER = 0.0;
        public static double END_ENCODER = 0.0;
        public static double ENCODER_CHANGE = 0.0;
        public static double TEST_POWER_APPLIED = 0.3;
        
        // User observation (Only set motor spin direction)
        public static boolean MOTOR_SPUN_FORWARD = false;
        
        // Auto-detected encoder direction
        public static String ENCODER_DIRECTION = "NOT_TESTED";
        
        // Direction analysis (Auto-calculated)
        public static String MOTOR_STATUS = "NOT_TESTED";
        public static String ENCODER_STATUS = "NOT_TESTED";
        public static String RECOMMENDATION = "Run a test first";
    }
    
    // ========== TEST STATE ==========
    
    private enum TestState {
        IDLE,
        TESTING
    }
    
    private TestState currentTest = TestState.IDLE;
    private DcMotorEx currentMotor;
    private double testStartTime = 0;
    private double startEncoderReading = 0;
    
    // ========== DASHBOARD MANAGER ==========
    private SmartDashboardManager dashboardManager;
    
    @Override
    protected void initializeCalibration() {
        // Get dashboard manager from MotionCalibrationAndDemo
        if (parentOpMode instanceof MotionCalibrationAndDemo) {
            dashboardManager = ((MotionCalibrationAndDemo) parentOpMode).getDashboardManager();
        }
        
        // Update current motor direction display from RobotConstants
        updateCurrentDirectionDisplay();
        
        // Reset test state
        currentTest = TestState.IDLE;
        currentMotor = null;
    }
    
    @Override
    protected void startTest() {
        // This method is called by the base class, but we handle test commands manually
        // in updateTest() to provide better control over the simple workflow
    }
    
    @Override
    protected void updateTest() {
        // Handle test commands
        handleTestCommands();
        
        // Update current test state
        updateTestState();
        
        // Update direction analysis
        updateDirectionAnalysis();
        
        // Update dashboard config if needed (critical for synchronization)
        if (dashboardManager != null) {
            dashboardManager.updateConfigIfNeeded();
        }
    }
    
    @Override
    protected void stopTest() {
        // Stop any running motor
        if (currentMotor != null) {
            currentMotor.setPower(0);
        }
        
        // Reset test state
        currentTest = TestState.IDLE;
    }
    
    @Override
    public void displayStatus() {
        telemetry.addLine("🔧 SIMPLE MOTOR DIRECTION TEST");
        telemetry.addLine("2-second motor test to verify directions");
        telemetry.addLine("");
        
        telemetry.addLine("📊 CURRENT MOTOR DIRECTIONS (RobotConstants.java):");
        telemetry.addData("Front Left", "%s", _1_MotorDirection.CURRENT_FRONT_LEFT);
        telemetry.addData("Front Right", "%s", _1_MotorDirection.CURRENT_FRONT_RIGHT);
        telemetry.addData("Back Left", "%s", _1_MotorDirection.CURRENT_BACK_LEFT);
        telemetry.addData("Back Right", "%s", _1_MotorDirection.CURRENT_BACK_RIGHT);
        telemetry.addLine("");
        
        telemetry.addLine("🎮 TEST CONTROLS (2-second test):");
        telemetry.addData("Selected Motor", "%s", _1_MotorDirection.MOTOR_SELECTION.displayName);
        telemetry.addData("RUN_TEST", "%s", _1_MotorDirection.RUN_TEST ? "YES" : "NO");
        telemetry.addData("STOP_TEST", "%s", _1_MotorDirection.STOP_TEST ? "YES" : "NO");
        telemetry.addData("Current State", "%s", currentTest.toString());
        telemetry.addLine("");
        
        telemetry.addLine("📏 TEST RESULTS (Auto-detected):");
        telemetry.addData("Start Encoder", "%.0f", _1_MotorDirection.START_ENCODER);
        telemetry.addData("End Encoder", "%.0f", _1_MotorDirection.END_ENCODER);
        telemetry.addData("Encoder Change", "%.0f", _1_MotorDirection.ENCODER_CHANGE);
        telemetry.addData("Encoder Direction", "%s", _1_MotorDirection.ENCODER_DIRECTION);
        telemetry.addData("Power Applied", "%.1f%%", _1_MotorDirection.TEST_POWER_APPLIED * 100);
        telemetry.addLine("");
        
        telemetry.addLine("👁️ YOUR INPUT (Only set motor spin):");
        telemetry.addData("MOTOR_SPUN_FORWARD", "%s", _1_MotorDirection.MOTOR_SPUN_FORWARD ? "YES" : "NO");
        telemetry.addLine("");
        
        telemetry.addLine("🧮 DIRECTION ANALYSIS:");
        telemetry.addData("Motor Status", "%s", _1_MotorDirection.MOTOR_STATUS);
        telemetry.addData("Encoder Status", "%s", _1_MotorDirection.ENCODER_STATUS);
        telemetry.addData("Recommendation", "%s", _1_MotorDirection.RECOMMENDATION);
        telemetry.addLine("");
        
        telemetry.addLine("📋 SIMPLIFIED WORKFLOW:");
        telemetry.addLine("1. Select motor from MOTOR_SELECTION dropdown");
        telemetry.addLine("2. Click RUN_TEST → Motor spins for 2 seconds");
        telemetry.addLine("3. Watch: Did motor spin forward? Set MOTOR_SPUN_FORWARD");
        telemetry.addLine("4. Encoder direction auto-detected from readings!");
        telemetry.addLine("5. Check analysis → Shows if directions are CORRECT");
        telemetry.addLine("6. If needed → Update RobotConstants.java manually");
    }
    
    @Override
    public void resetParameters() {
        // Reset test commands
        _1_MotorDirection.RUN_TEST = false;
        _1_MotorDirection.STOP_TEST = false;
        
        // Reset test results
        _1_MotorDirection.START_ENCODER = 0.0;
        _1_MotorDirection.END_ENCODER = 0.0;
        _1_MotorDirection.ENCODER_CHANGE = 0.0;
        _1_MotorDirection.TEST_POWER_APPLIED = 0.3;
        
        // Reset user observation
        _1_MotorDirection.MOTOR_SPUN_FORWARD = false;
        
        // Reset auto-detected values
        _1_MotorDirection.ENCODER_DIRECTION = "NOT_TESTED";
        
        // Reset analysis
        _1_MotorDirection.MOTOR_STATUS = "NOT_TESTED";
        _1_MotorDirection.ENCODER_STATUS = "NOT_TESTED";
        _1_MotorDirection.RECOMMENDATION = "Run a test first";
        
        // Reset motor selection
        _1_MotorDirection.MOTOR_SELECTION = MotorSelection.FRONT_LEFT;
        
        // Stop any current test
        if (currentMotor != null) {
            currentMotor.setPower(0);
        }
        currentTest = TestState.IDLE;
        
        // Update current direction display
        updateCurrentDirectionDisplay();
    }
    
    @Override
    public String getCalibrationName() {
        return "Simple Motor Direction Test";
    }
    
    @Override
    public String getCalibrationDescription() {
        return "Simple 2-second motor tests to verify directions";
    }
    
    // ========== HELPER METHODS ==========
    
    private void updateCurrentDirectionDisplay() {
        // Update dashboard with current motor directions from RobotConstants
        _1_MotorDirection.CURRENT_FRONT_LEFT = RobotConstants.FRONT_LEFT_REVERSED ? "REVERSED" : "FORWARD";
        _1_MotorDirection.CURRENT_FRONT_RIGHT = RobotConstants.FRONT_RIGHT_REVERSED ? "REVERSED" : "FORWARD";
        _1_MotorDirection.CURRENT_BACK_LEFT = RobotConstants.BACK_LEFT_REVERSED ? "REVERSED" : "FORWARD";
        _1_MotorDirection.CURRENT_BACK_RIGHT = RobotConstants.BACK_RIGHT_REVERSED ? "REVERSED" : "FORWARD";
    }
    
    private void handleTestCommands() {
        // Emergency stop overrides all other commands
        if (_1_MotorDirection.STOP_TEST) {
            if (currentMotor != null) {
                currentMotor.setPower(0);
            }
            currentTest = TestState.IDLE;
            _1_MotorDirection.STOP_TEST = false;
            return;
        }
        
        // Only process new commands if not currently testing
        if (currentTest == TestState.IDLE && _1_MotorDirection.RUN_TEST) {
            startMotorTest();
            _1_MotorDirection.RUN_TEST = false;
        }
    }
    
    private void startMotorTest() {
        // Get the selected motor
        currentMotor = getMotorByIndex(_1_MotorDirection.MOTOR_SELECTION.id);
        if (currentMotor == null) {
            return;
        }
        
        // Reset and start encoder
        currentMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        currentMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Record starting position
        startEncoderReading = currentMotor.getCurrentPosition();
        _1_MotorDirection.START_ENCODER = startEncoderReading;
        _1_MotorDirection.TEST_POWER_APPLIED = 0.3;
        if (dashboardManager != null) {
            dashboardManager.markConfigChanged();
        }
        
        // Start motor at 30% power
        currentMotor.setPower(0.3);
        
        // Start test timer
        testStartTime = System.currentTimeMillis();
        currentTest = TestState.TESTING;
    }
    
    private void updateTestState() {
        if (currentTest != TestState.TESTING || currentMotor == null) {
            return;
        }
        
        // Update current encoder reading
        double currentEncoderReading = currentMotor.getCurrentPosition();
        _1_MotorDirection.END_ENCODER = currentEncoderReading;
        _1_MotorDirection.ENCODER_CHANGE = currentEncoderReading - startEncoderReading;
        if (dashboardManager != null) {
            dashboardManager.markConfigChanged();
        }
        
        // Check if 2 seconds have elapsed
        double elapsedTime = (System.currentTimeMillis() - testStartTime) / 1000.0;
        if (elapsedTime >= 2.0) {
            completeMotorTest();
        }
    }
    
    private void completeMotorTest() {
        // Stop motor
        if (currentMotor != null) {
            currentMotor.setPower(0);
        }
        
        // Record final encoder reading
        if (currentMotor != null) {
            double finalEncoderReading = currentMotor.getCurrentPosition();
            _1_MotorDirection.END_ENCODER = finalEncoderReading;
            _1_MotorDirection.ENCODER_CHANGE = finalEncoderReading - startEncoderReading;
        }
        
        // Reset test state
        currentTest = TestState.IDLE;
    }
    
    private void updateDirectionAnalysis() {
        // Only analyze if we have test results
        if (_1_MotorDirection.ENCODER_CHANGE == 0.0) {
            _1_MotorDirection.MOTOR_STATUS = "NOT_TESTED";
            _1_MotorDirection.ENCODER_STATUS = "NOT_TESTED";
            _1_MotorDirection.ENCODER_DIRECTION = "NOT_TESTED";
            _1_MotorDirection.RECOMMENDATION = "Run a test first";
            return;
        }
        
        // Auto-detect encoder direction from actual readings
        boolean encoderIncreased = _1_MotorDirection.ENCODER_CHANGE > 0;
        _1_MotorDirection.ENCODER_DIRECTION = encoderIncreased ? "INCREASED" : "DECREASED";
        
        // Analyze motor direction based on user observation
        if (_1_MotorDirection.MOTOR_SPUN_FORWARD) {
            _1_MotorDirection.MOTOR_STATUS = "✅ CORRECT (Forward as expected)";
        } else {
            _1_MotorDirection.MOTOR_STATUS = "❌ NEEDS_REVERSE (Spun backward)";
        }
        
        // Analyze encoder direction (should increase when motor spins forward)
        if (encoderIncreased) {
            _1_MotorDirection.ENCODER_STATUS = "✅ CORRECT (Increased)";
        } else {
            _1_MotorDirection.ENCODER_STATUS = "❌ NEEDS_REVERSE (Decreased)";
        }
        
        // Generate recommendation
        boolean motorCorrect = _1_MotorDirection.MOTOR_SPUN_FORWARD;
        boolean encoderCorrect = encoderIncreased;
        
        if (motorCorrect && encoderCorrect) {
            _1_MotorDirection.RECOMMENDATION = "✅ Perfect! Both motor and encoder are correct";
        } else if (!motorCorrect && !encoderCorrect) {
            _1_MotorDirection.RECOMMENDATION = "⚠️ Reverse motor direction in RobotConstants.java";
        } else if (!motorCorrect && encoderCorrect) {
            _1_MotorDirection.RECOMMENDATION = "❌ Motor wrong but encoder correct - check wiring";
        } else {
            _1_MotorDirection.RECOMMENDATION = "❌ Motor correct but encoder wrong - check encoder cable";
        }
        
        if (dashboardManager != null) {
            dashboardManager.markConfigChanged();
        }
    }
}
