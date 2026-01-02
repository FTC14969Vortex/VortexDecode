package org.firstinspires.ftc.teamcode.calibration.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.calibration.BaseCalibration;
import org.firstinspires.ftc.teamcode.calibration.CalibrationCoefficients;
import org.firstinspires.ftc.teamcode.calibration.SmartDashboardManager;
import org.firstinspires.ftc.teamcode.calibration.MotionCalibrationAndDemo;
import org.firstinspires.ftc.teamcode.motion.MecanumKinematics;
import org.firstinspires.ftc.teamcode.motion.MotionConfig;
import org.firstinspires.ftc.teamcode.motion.MotionState;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Motor Velocity PIDF Calibration Module - 8-Factor Velocity Scaling
 * 
 * Calibrates 8 independent velocity scaling factors for each wheel in each direction.
 * This compensates for motor-to-motor variations and asymmetric friction.
 * 
 * Features:
 * - 8-factor velocity scaling calibration (4 wheels × 2 directions)
 * - Individual direction test selection or run all tests
 * - Real-time odometry-based velocity measurement
 * - Automatic scaling factor calculation
 * - Results displayed on FTC Dashboard for easy copying
 */
public class MotorVelocityPIDFCalibrationModule extends BaseCalibration {
    
    // ========== DASHBOARD PARAMETERS ==========
    
    @Config
    public static class _2_MotorVelocityPIDF {
        // ========== MOTOR VELOCITY PIDF TUNING ==========
        // These parameters tune the motor controller's velocity control
        public static double Kp = MotionConfig.MOTOR_VELOCITY_KP;
        public static double Ki = MotionConfig.MOTOR_VELOCITY_KI;
        public static double Kd = MotionConfig.MOTOR_VELOCITY_KD;
        public static double Kf = MotionConfig.MOTOR_VELOCITY_KF;
        
        // ========== 8-FACTOR CALIBRATION CONFIGURATION ==========
        public static boolean RUN_TEST = false;              // Click to start velocity calibration test
        public static MotionCalibrationAndDemo.VelocityTestSelection SELECTED_TEST = MotionCalibrationAndDemo.VelocityTestSelection.RUN_ALL_TESTS;  // Use dropdown to select test
        public static double TARGET_VELOCITY = CalibrationCoefficients.CALIBRATED_MAX_LINEAR_VELOCITY;  // Test velocity (in/s)
        public static double SCALING_TEST_DURATION = 1.0;        // seconds per direction test
        
        // ========== 8-FACTOR CALIBRATION RESULTS (AUTO-POPULATED) ==========
        public static double SCALE_FL_POSITIVE = 1.0;    // Front Left wheel positive direction scaling
        public static double SCALE_FL_NEGATIVE = 1.0;    // Front Left wheel negative direction scaling
        public static double SCALE_FR_POSITIVE = 1.0;    // Front Right wheel positive direction scaling
        public static double SCALE_FR_NEGATIVE = 1.0;    // Front Right wheel negative direction scaling
        public static double SCALE_BL_POSITIVE = 1.0;    // Back Left wheel positive direction scaling
        public static double SCALE_BL_NEGATIVE = 1.0;    // Back Left wheel negative direction scaling
        public static double SCALE_BR_POSITIVE = 1.0;    // Back Right wheel positive direction scaling
        public static double SCALE_BR_NEGATIVE = 1.0;    // Back Right wheel negative direction scaling
        public static double CALIBRATION_ACCURACY = 999.0; // RMS error in inches/second
        
        // ========== CALIBRATION PROGRESS (AUTO-POPULATED) ==========
        public static String CURRENT_TEST = "Ready";      // Current test being performed
        public static double PROGRESS_PERCENT = 0.0;      // Overall calibration progress (0-100%)
        public static double CURRENT_TEST_TIME = 0.0;     // Time elapsed in current test
        public static boolean CALIBRATION_COMPLETE = false; // True when 8-factor calibration is finished
    }
    
    // ========== TEST STATE ==========
    
    private SmartDashboardManager dashboardManager;
    private boolean testInProgress = false;
    
    // ========== 8-FACTOR CALIBRATION STATE ==========
    
    // Calculated 8 scaling factors (display only)
    private double calculatedScaleFL_Positive = 1.0;
    private double calculatedScaleFL_Negative = 1.0;
    private double calculatedScaleFR_Positive = 1.0;
    private double calculatedScaleFR_Negative = 1.0;
    private double calculatedScaleBL_Positive = 1.0;
    private double calculatedScaleBL_Negative = 1.0;
    private double calculatedScaleBR_Positive = 1.0;
    private double calculatedScaleBR_Negative = 1.0;

    // Test data storage
    private Map<String, TestData> testResults = new HashMap<>();
    private double calibrationAccuracy = 999.0;
    
    // Helper class for test data
    private static class TestData {
        double commandedVx, commandedVy;
        List<Double> odometryVelX, odometryVelY;
        List<Double> wheelVelFL, wheelVelFR, wheelVelBL, wheelVelBR;
    }
    
    @Override
    protected void initializeCalibration() {
        // Initialize SmartDashboardManager for real-time feedback
        if (parentOpMode instanceof MotionCalibrationAndDemo) {
            dashboardManager = ((MotionCalibrationAndDemo) parentOpMode).getDashboardManager();
        }
        
        // CRITICAL: Reset odometry to (0,0,0) at start of calibration
        // This ensures velocity measurements start from a known position
        motionExecutor.resetToFieldOrigin();
        
        // Reset progress indicators
        _2_MotorVelocityPIDF.CURRENT_TEST = "Ready";
        _2_MotorVelocityPIDF.PROGRESS_PERCENT = 0.0;
        _2_MotorVelocityPIDF.CURRENT_TEST_TIME = 0.0;
        _2_MotorVelocityPIDF.CALIBRATION_COMPLETE = false;
        testInProgress = false;
    }
    
    @Override
    protected void startTest() {
        // This method is called by the base class, but we handle test commands manually
        // in updateTest() to provide better control over the RUN_TEST button workflow
    }
    
    private void startTestInternal() {
        if (motionExecutor == null || odometryManager == null) {
            if (telemetry != null) {
                telemetry.addLine("❌ Motion system not available for 8-factor calibration");
            }
            return;
        }
        
        if (telemetry == null) {
            return;
        }
        
        // Start calibration based on selected test
        testInProgress = true;
        
        if (_2_MotorVelocityPIDF.SELECTED_TEST == MotionCalibrationAndDemo.VelocityTestSelection.RUN_ALL_TESTS) {
            // Run all 4 tests sequentially
            run8FactorOdometryCalibration();
        } else {
            // Run single selected test
            runSingleTest(_2_MotorVelocityPIDF.SELECTED_TEST);
        }
        
        testInProgress = false;
    }
    
    /**
     * Handle test command buttons (RUN_TEST)
     */
    private void handleTestCommands() {
        // Only process new commands if not currently testing
        if (!testInProgress && _2_MotorVelocityPIDF.RUN_TEST) {
            startTestInternal();
            _2_MotorVelocityPIDF.RUN_TEST = false;
        }
    }
    
    @Override
    protected void updateTest() {
        // Handle test commands
        handleTestCommands();
        
        // Update dashboard config if needed (critical for synchronization)
        if (dashboardManager != null) {
            dashboardManager.updateConfigIfNeeded();
        }
        
        // Display status during test
        if (testInProgress && telemetry != null) {
            telemetry.addLine("⏳ Test in progress...");
            telemetry.addLine("(Robot is executing movement)");
        }
    }
    
    @Override
    protected void stopTest() {
        // Stop all motors
        if (motionExecutor != null) {
            motionExecutor.stop();
        }
        testInProgress = false;
    }
    
    @Override
    public void displayStatus() {
        if (telemetry == null) {
            return;
        }
        
        telemetry.addLine("🎯 8-FACTOR VELOCITY CALIBRATION");
        telemetry.addLine("Calibrate individual wheel velocity scaling factors");
        telemetry.addLine("");
        
        // Motor PIDF configuration
        telemetry.addLine("🔧 MOTOR VELOCITY PIDF (for tuning):");
        telemetry.addData("Kp", "%.6f", _2_MotorVelocityPIDF.Kp);
        telemetry.addData("Ki", "%.6f", _2_MotorVelocityPIDF.Ki);
        telemetry.addData("Kd", "%.6f", _2_MotorVelocityPIDF.Kd);
        telemetry.addData("Kf", "%.6f", _2_MotorVelocityPIDF.Kf);
        telemetry.addLine("");
        
        // Test configuration
        telemetry.addLine("⚙️ TEST CONFIGURATION:");
        telemetry.addData("RUN_TEST", "%s", _2_MotorVelocityPIDF.RUN_TEST ? "YES" : "NO");
        telemetry.addData("Selected Test", _2_MotorVelocityPIDF.SELECTED_TEST != null ? _2_MotorVelocityPIDF.SELECTED_TEST.displayName : "None");
        telemetry.addData("Target Velocity", "%.1f in/s", _2_MotorVelocityPIDF.TARGET_VELOCITY);
        telemetry.addData("Test Duration", "%.1f sec/direction", _2_MotorVelocityPIDF.SCALING_TEST_DURATION);
        
        if (_2_MotorVelocityPIDF.SELECTED_TEST == MotionCalibrationAndDemo.VelocityTestSelection.RUN_ALL_TESTS) {
            telemetry.addData("Total Test Time", "~%.0f seconds", _2_MotorVelocityPIDF.SCALING_TEST_DURATION * 4 + 3);
        }
        telemetry.addLine("");
        
        // Progress tracking
        telemetry.addLine("📊 PROGRESS:");
        telemetry.addData("Current Test", _2_MotorVelocityPIDF.CURRENT_TEST);
        telemetry.addData("Progress", "%.1f%%", _2_MotorVelocityPIDF.PROGRESS_PERCENT);
        if (_2_MotorVelocityPIDF.CURRENT_TEST_TIME > 0) {
            telemetry.addData("Test Time", "%.1f/%.1f sec", 
                _2_MotorVelocityPIDF.CURRENT_TEST_TIME, _2_MotorVelocityPIDF.SCALING_TEST_DURATION);
        }
        telemetry.addLine("");
        
        // Results status
        if (_2_MotorVelocityPIDF.CALIBRATION_COMPLETE) {
            telemetry.addLine("✅ CALIBRATION COMPLETE!");
            telemetry.addLine("📊 8 scaling factors now visible on FTC Dashboard");
            telemetry.addData("Calibration Accuracy", "%.3f in/s RMS", _2_MotorVelocityPIDF.CALIBRATION_ACCURACY);
            telemetry.addLine("");
            
            // Display scaling factors on telemetry
            telemetry.addLine("🔧 SCALING FACTORS:");
            telemetry.addData("FL+", "%.4f", _2_MotorVelocityPIDF.SCALE_FL_POSITIVE);
            telemetry.addData("FL-", "%.4f", _2_MotorVelocityPIDF.SCALE_FL_NEGATIVE);
            telemetry.addData("FR+", "%.4f", _2_MotorVelocityPIDF.SCALE_FR_POSITIVE);
            telemetry.addData("FR-", "%.4f", _2_MotorVelocityPIDF.SCALE_FR_NEGATIVE);
            telemetry.addData("BL+", "%.4f", _2_MotorVelocityPIDF.SCALE_BL_POSITIVE);
            telemetry.addData("BL-", "%.4f", _2_MotorVelocityPIDF.SCALE_BL_NEGATIVE);
            telemetry.addData("BR+", "%.4f", _2_MotorVelocityPIDF.SCALE_BR_POSITIVE);
            telemetry.addData("BR-", "%.4f", _2_MotorVelocityPIDF.SCALE_BR_NEGATIVE);
            telemetry.addLine("");
            
            telemetry.addLine("✏️ COPY VALUES FROM DASHBOARD:");
            telemetry.addLine("1. Open FTC Dashboard configuration");
            telemetry.addLine("2. Find '_2_MotorVelocityPIDF' section");
            telemetry.addLine("3. Copy SCALE_* values to CalibrationCoefficients.java");
            telemetry.addLine("4. Set USE_8_FACTOR_VELOCITY_SCALING = true");
        } else if (testInProgress) {
            telemetry.addLine("🔄 CALIBRATION IN PROGRESS...");
            telemetry.addLine("Robot is measuring velocity in each direction");
            telemetry.addLine("Real-time data on FTC Dashboard");
        } else {
            telemetry.addLine("📋 INSTRUCTIONS:");
            telemetry.addLine("1. Select test from SELECTED_TEST dropdown:");
            telemetry.addLine("   • Individual direction test (Forward/Back/Left/Right)");
            telemetry.addLine("   • Run All 4 Tests (recommended)");
            telemetry.addLine("2. Click RUN_TEST to start calibration");
            telemetry.addLine("3. Results appear on FTC Dashboard");
            telemetry.addLine("4. Copy SCALE_* values to CalibrationCoefficients.java");
        }
    }
    
    @Override
    public void resetParameters() {
        // Reset PIDF parameters to MotionConfig values
        _2_MotorVelocityPIDF.Kp = MotionConfig.MOTOR_VELOCITY_KP;
        _2_MotorVelocityPIDF.Ki = MotionConfig.MOTOR_VELOCITY_KI;
        _2_MotorVelocityPIDF.Kd = MotionConfig.MOTOR_VELOCITY_KD;
        _2_MotorVelocityPIDF.Kf = MotionConfig.MOTOR_VELOCITY_KF;
        
        // Reset test selection
        _2_MotorVelocityPIDF.RUN_TEST = false;
        _2_MotorVelocityPIDF.SELECTED_TEST = MotionCalibrationAndDemo.VelocityTestSelection.RUN_ALL_TESTS;
        
        // Reset 8-factor calibration configuration
        _2_MotorVelocityPIDF.TARGET_VELOCITY = CalibrationCoefficients.CALIBRATED_MAX_LINEAR_VELOCITY;
        _2_MotorVelocityPIDF.SCALING_TEST_DURATION = 2.0;
        
        // Reset 8-factor calibration results
        _2_MotorVelocityPIDF.SCALE_FL_POSITIVE = 1.0;
        _2_MotorVelocityPIDF.SCALE_FL_NEGATIVE = 1.0;
        _2_MotorVelocityPIDF.SCALE_FR_POSITIVE = 1.0;
        _2_MotorVelocityPIDF.SCALE_FR_NEGATIVE = 1.0;
        _2_MotorVelocityPIDF.SCALE_BL_POSITIVE = 1.0;
        _2_MotorVelocityPIDF.SCALE_BL_NEGATIVE = 1.0;
        _2_MotorVelocityPIDF.SCALE_BR_POSITIVE = 1.0;
        _2_MotorVelocityPIDF.SCALE_BR_NEGATIVE = 1.0;
        _2_MotorVelocityPIDF.CALIBRATION_ACCURACY = 999.0;
        
        // Reset progress tracking
        _2_MotorVelocityPIDF.CURRENT_TEST = "Ready";
        _2_MotorVelocityPIDF.PROGRESS_PERCENT = 0.0;
        _2_MotorVelocityPIDF.CURRENT_TEST_TIME = 0.0;
        _2_MotorVelocityPIDF.CALIBRATION_COMPLETE = false;
        testInProgress = false;
        
        // Mark dashboard config as changed
        if (dashboardManager != null) {
            dashboardManager.markConfigChanged();
        }
    }
    
    @Override
    public String getCalibrationName() {
        return "8-Factor Velocity Scaling";
    }
    
    @Override
    public String getCalibrationDescription() {
        return "Calibrate individual wheel velocity scaling factors";
    }
    
    // ========== 8-FACTOR CALIBRATION METHODS ==========
    
    private void runSingleTest(MotionCalibrationAndDemo.VelocityTestSelection test) {
        if (test == null) {
            if (telemetry != null) {
                telemetry.addLine("❌ No test selected");
            }
            return;
        }
        
        if (test == MotionCalibrationAndDemo.VelocityTestSelection.RUN_ALL_TESTS) {
            run8FactorOdometryCalibration();
            return;
        }
        
        if (telemetry == null) {
            return;
        }
        
        // Initialize calibration state
        _2_MotorVelocityPIDF.CALIBRATION_COMPLETE = false;
        _2_MotorVelocityPIDF.PROGRESS_PERCENT = 0.0;
        _2_MotorVelocityPIDF.CURRENT_TEST = "Initializing...";
        
        if (dashboardManager != null) {
            dashboardManager.markConfigChanged();
        }
        
        telemetry.addLine("🔄 Starting " + test.displayName + " test...");
        telemetry.update();
        
        double testVel = getTestVelocityInchesPerSec();
        double duration = _2_MotorVelocityPIDF.SCALING_TEST_DURATION;
        
        // Clear previous results
        testResults.clear();
        
        // Run selected test
        switch (test) {
            case TEST_FORWARD:
                runDirectionalTest("Forward", testVel, 0.0, 0.0, duration, 1, 1);
                break;
            case TEST_BACKWARD:
                runDirectionalTest("Backward", -testVel, 0.0, 0.0, duration, 1, 1);
                break;
            case TEST_STRAFE_LEFT:
                runDirectionalTest("Strafe Left", 0.0, testVel, 0.0, duration, 1, 1);
                break;
            case TEST_STRAFE_RIGHT:
                runDirectionalTest("Strafe Right", 0.0, -testVel, 0.0, duration, 1, 1);
                break;
        }
        
        // Calculate scaling factors from single test
        _2_MotorVelocityPIDF.CURRENT_TEST = "Calculating results...";
        _2_MotorVelocityPIDF.PROGRESS_PERCENT = 90.0;
        if (dashboardManager != null) {
            dashboardManager.markConfigChanged();
        }
        
        calculate8FactorScaling();
        
        // Mark calibration complete and update dashboard
        _2_MotorVelocityPIDF.CALIBRATION_COMPLETE = true;
        _2_MotorVelocityPIDF.CURRENT_TEST = "Complete: " + test.displayName;
        _2_MotorVelocityPIDF.PROGRESS_PERCENT = 100.0;
        
        if (dashboardManager != null) {
            dashboardManager.forceConfigUpdate();  // Force immediate update for results
        }
        
        if (telemetry != null) {
            telemetry.addLine("✅ " + test.displayName + " Test Complete!");
            telemetry.addLine("📊 Results now visible on FTC Dashboard!");
            telemetry.update();
        }
    }
    
    private void run8FactorOdometryCalibration() {
        if (motionExecutor == null || odometryManager == null) {
            if (telemetry != null) {
                telemetry.addLine("❌ Motion system not available for 8-factor calibration");
                telemetry.update();
            }
            return;
        }
        
        if (telemetry == null) {
            return;
        }
        
        // Initialize calibration state
        _2_MotorVelocityPIDF.CALIBRATION_COMPLETE = false;
        _2_MotorVelocityPIDF.PROGRESS_PERCENT = 0.0;
        _2_MotorVelocityPIDF.CURRENT_TEST = "Initializing...";
        
        if (dashboardManager != null) {
            dashboardManager.markConfigChanged();
        }
        
        telemetry.addLine("🔄 Starting 8-Factor Odometry Calibration...");
        telemetry.update();
        
        double testVel = getTestVelocityInchesPerSec();
        double duration = _2_MotorVelocityPIDF.SCALING_TEST_DURATION;
        
        // Clear previous results
        testResults.clear();
        
        // Test all 4 cardinal directions with progress tracking
        runDirectionalTest("Forward", testVel, 0.0, 0.0, duration, 1, 4);      // +vx
        waitForSettling("Forward → Backward", 1000);
        
        runDirectionalTest("Backward", -testVel, 0.0, 0.0, duration, 2, 4);    // -vx  
        waitForSettling("Backward → Strafe Left", 1000);
        
        runDirectionalTest("Strafe Left", 0.0, testVel, 0.0, duration, 3, 4);  // +vy
        waitForSettling("Strafe Left → Strafe Right", 1000);
        
        runDirectionalTest("Strafe Right", 0.0, -testVel, 0.0, duration, 4, 4); // -vy
        
        // Calculate 8 scaling factors from collected data
        _2_MotorVelocityPIDF.CURRENT_TEST = "Calculating results...";
        _2_MotorVelocityPIDF.PROGRESS_PERCENT = 90.0;
        if (dashboardManager != null) {
            dashboardManager.markConfigChanged();
        }
        
        calculate8FactorScaling();
        
        // Mark calibration complete and update dashboard
        _2_MotorVelocityPIDF.CALIBRATION_COMPLETE = true;
        _2_MotorVelocityPIDF.CURRENT_TEST = "Complete: All 4 Tests";
        _2_MotorVelocityPIDF.PROGRESS_PERCENT = 100.0;
        
        if (dashboardManager != null) {
            dashboardManager.forceConfigUpdate();  // Force immediate update for results
        }
        
        if (telemetry != null) {
            telemetry.addLine("✅ 8-Factor Calibration Complete!");
            telemetry.addLine("📊 Results now visible on FTC Dashboard!");
            telemetry.update();
        }
    }
    
    private void waitForSettling(String transitionName, int milliseconds) {
        _2_MotorVelocityPIDF.CURRENT_TEST = transitionName;
        if (dashboardManager != null) {
            dashboardManager.markConfigChanged();
        }
        
        try { 
            Thread.sleep(milliseconds); 
        } catch (InterruptedException e) { 
            Thread.currentThread().interrupt(); 
        }
    }
    
    private void runDirectionalTest(String testName, double vx, double vy, double omega, double duration, int testNumber, int totalTests) {
        if (telemetry == null) {
            return;
        }
        
        // Update progress tracking
        _2_MotorVelocityPIDF.CURRENT_TEST = testName + " (" + testNumber + "/" + totalTests + ")";
        _2_MotorVelocityPIDF.PROGRESS_PERCENT = ((testNumber - 1) * 100.0) / totalTests;
        _2_MotorVelocityPIDF.CURRENT_TEST_TIME = 0.0;
        
        if (dashboardManager != null) {
            dashboardManager.markConfigChanged();
        }
        
        telemetry.addLine("🔄 Running " + testName + " test (" + testNumber + "/" + totalTests + ")...");
        telemetry.update();
        
        // Reset odometry for clean measurement
        odometryManager.resetPosition(0.0, 0.0);
        
        // Start motion using DriveHardware directly (with 8-factor scaling)
        motionExecutor.getDriveHardware().setRobotVelocity(vx, vy, omega, MotionState.CoordinateMode.ROBOT_CENTRIC, 0.0);
        
        // Data collection
        ElapsedTime timer = new ElapsedTime();
        List<Double> odometryVelX = new ArrayList<>();
        List<Double> odometryVelY = new ArrayList<>();
        List<Double> wheelVelFL = new ArrayList<>();
        List<Double> wheelVelFR = new ArrayList<>();
        List<Double> wheelVelBL = new ArrayList<>();
        List<Double> wheelVelBR = new ArrayList<>();
        
        Pose2D lastPose = odometryManager.getCurrentPose();
        double lastTime = timer.seconds();
        
        // Skip first 0.5 seconds for settling
        while (timer.seconds() < 0.5) {
            try { Thread.sleep(50); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
        }
        
        // Collect data for specified duration
        timer.reset();
        lastTime = 0;
        lastPose = odometryManager.getCurrentPose();
        
        while (timer.seconds() < duration) {
            try { Thread.sleep(50); } catch (InterruptedException e) { Thread.currentThread().interrupt(); } // 20Hz sampling
            
            Pose2D currentPose = odometryManager.getCurrentPose();
            double currentTime = timer.seconds();
            double dt = currentTime - lastTime;
            
            // Update progress within current test
            _2_MotorVelocityPIDF.CURRENT_TEST_TIME = currentTime;
            double testProgress = (currentTime / duration) * (100.0 / totalTests);
            _2_MotorVelocityPIDF.PROGRESS_PERCENT = ((testNumber - 1) * 100.0 / totalTests) + testProgress;
            
            if (dt >= 0.05) { // Valid sample
                // Calculate actual odometry velocities (inches/sec)
                double actualVx = (currentPose.getX(DistanceUnit.INCH) - lastPose.getX(DistanceUnit.INCH)) / dt;
                double actualVy = (currentPose.getY(DistanceUnit.INCH) - lastPose.getY(DistanceUnit.INCH)) / dt;
                
                odometryVelX.add(actualVx);
                odometryVelY.add(actualVy);
                
                // Record wheel velocities (in ticks/sec)
                double currentWheelFL = frontLeftDrive.getVelocity();
                double currentWheelFR = frontRightDrive.getVelocity();
                double currentWheelBL = backLeftDrive.getVelocity();
                double currentWheelBR = backRightDrive.getVelocity();
                
                wheelVelFL.add(currentWheelFL);
                wheelVelFR.add(currentWheelFR);
                wheelVelBL.add(currentWheelBL);
                wheelVelBR.add(currentWheelBR);
                
                // Send real-time velocity data to dashboard
                if (dashboardManager != null) {
                    // Use the primary velocity component for graphing
                    double primaryVelocity = Math.abs(vx) > Math.abs(vy) ? actualVx : actualVy;
                    double commandedVelocity = Math.abs(vx) > Math.abs(vy) ? vx : vy;
                    double velocityError = primaryVelocity - commandedVelocity;
                    
                    dashboardManager.updateCalibrationData(currentPose.getX(DistanceUnit.INCH), 
                        currentPose.getY(DistanceUnit.INCH), velocityError, primaryVelocity);
                }
                
                lastPose = currentPose;
                lastTime = currentTime;
            }
        }
        
        motionExecutor.stop();
        
        // Store test results
        TestData data = new TestData();
        data.commandedVx = vx;
        data.commandedVy = vy;
        data.odometryVelX = new ArrayList<>(odometryVelX);
        data.odometryVelY = new ArrayList<>(odometryVelY);
        data.wheelVelFL = new ArrayList<>(wheelVelFL);
        data.wheelVelFR = new ArrayList<>(wheelVelFR);
        data.wheelVelBL = new ArrayList<>(wheelVelBL);
        data.wheelVelBR = new ArrayList<>(wheelVelBR);
        
        testResults.put(testName, data);
    }
    
    private void calculate8FactorScaling() {
        // Accumulate scaling factors across multiple tests
        Map<String, List<Double>> positiveScalingData = new HashMap<>();
        Map<String, List<Double>> negativeScalingData = new HashMap<>();
        
        // Initialize data maps
        for (String wheel : Arrays.asList("FL", "FR", "BL", "BR")) {
            positiveScalingData.put(wheel, new ArrayList<>());
            negativeScalingData.put(wheel, new ArrayList<>());
        }
        
        // Analyze only tests that were actually run (skip null tests)
        if (testResults.containsKey("Forward") && testResults.get("Forward") != null) {
            analyzeTestData("Forward", testResults.get("Forward"), 
                           positiveScalingData, negativeScalingData,
                           true, true, true, true);  // All wheels positive
        }
        
        if (testResults.containsKey("Backward") && testResults.get("Backward") != null) {
            analyzeTestData("Backward", testResults.get("Backward"),
                           positiveScalingData, negativeScalingData, 
                           false, false, false, false); // All wheels negative
        }
        
        if (testResults.containsKey("Strafe Left") && testResults.get("Strafe Left") != null) {
            analyzeTestData("Strafe Left", testResults.get("Strafe Left"),
                           positiveScalingData, negativeScalingData,
                           true, false, false, true);  // FL+, FR-, BL-, BR+
        }
        
        if (testResults.containsKey("Strafe Right") && testResults.get("Strafe Right") != null) {
            analyzeTestData("Strafe Right", testResults.get("Strafe Right"),
                           positiveScalingData, negativeScalingData,
                           false, true, true, false);  // FL-, FR+, BL+, BR-
        }
        
        // Calculate final scaling factors
        calculatedScaleFL_Positive = calculateAverage(positiveScalingData.get("FL"));
        calculatedScaleFL_Negative = calculateAverage(negativeScalingData.get("FL"));
        calculatedScaleFR_Positive = calculateAverage(positiveScalingData.get("FR"));
        calculatedScaleFR_Negative = calculateAverage(negativeScalingData.get("FR"));
        calculatedScaleBL_Positive = calculateAverage(positiveScalingData.get("BL"));
        calculatedScaleBL_Negative = calculateAverage(negativeScalingData.get("BL"));
        calculatedScaleBR_Positive = calculateAverage(positiveScalingData.get("BR"));
        calculatedScaleBR_Negative = calculateAverage(negativeScalingData.get("BR"));
        
        // Calculate calibration accuracy (RMS error)
        calculateCalibrationAccuracy();
        
        // ========== CRITICAL: SYNC RESULTS TO DASHBOARD PARAMETERS ==========
        // This makes the 8 scaling factors visible on FTC Dashboard!
        // Only update values that have valid data (not 1.0 default)
        if (calculatedScaleFL_Positive != 1.0 || !positiveScalingData.get("FL").isEmpty()) {
            _2_MotorVelocityPIDF.SCALE_FL_POSITIVE = calculatedScaleFL_Positive;
        }
        if (calculatedScaleFL_Negative != 1.0 || !negativeScalingData.get("FL").isEmpty()) {
            _2_MotorVelocityPIDF.SCALE_FL_NEGATIVE = calculatedScaleFL_Negative;
        }
        if (calculatedScaleFR_Positive != 1.0 || !positiveScalingData.get("FR").isEmpty()) {
            _2_MotorVelocityPIDF.SCALE_FR_POSITIVE = calculatedScaleFR_Positive;
        }
        if (calculatedScaleFR_Negative != 1.0 || !negativeScalingData.get("FR").isEmpty()) {
            _2_MotorVelocityPIDF.SCALE_FR_NEGATIVE = calculatedScaleFR_Negative;
        }
        if (calculatedScaleBL_Positive != 1.0 || !positiveScalingData.get("BL").isEmpty()) {
            _2_MotorVelocityPIDF.SCALE_BL_POSITIVE = calculatedScaleBL_Positive;
        }
        if (calculatedScaleBL_Negative != 1.0 || !negativeScalingData.get("BL").isEmpty()) {
            _2_MotorVelocityPIDF.SCALE_BL_NEGATIVE = calculatedScaleBL_Negative;
        }
        if (calculatedScaleBR_Positive != 1.0 || !positiveScalingData.get("BR").isEmpty()) {
            _2_MotorVelocityPIDF.SCALE_BR_POSITIVE = calculatedScaleBR_Positive;
        }
        if (calculatedScaleBR_Negative != 1.0 || !negativeScalingData.get("BR").isEmpty()) {
            _2_MotorVelocityPIDF.SCALE_BR_NEGATIVE = calculatedScaleBR_Negative;
        }
        _2_MotorVelocityPIDF.CALIBRATION_ACCURACY = calibrationAccuracy;
    }
    
    private void analyzeTestData(String testName, TestData data,
                               Map<String, List<Double>> positiveData,
                               Map<String, List<Double>> negativeData,
                               boolean flPositive, boolean frPositive, 
                               boolean blPositive, boolean brPositive) {
        
        if (data == null || data.odometryVelX.isEmpty()) return;
        
        // CRITICAL: Use COMMANDED velocity, not measured odometry velocity!
        // The scaling factor should represent: "What scaling is needed to achieve the commanded velocity?"
        // 
        // Flow:
        // 1. Commanded velocity (in/s) → Expected wheel velocities (ticks/s)
        // 2. Measured wheel velocities (ticks/s) from encoders
        // 3. Scaling factor = expected / actual
        //    - If actual < expected → scaling > 1.0 (motor needs boost)
        //    - If actual > expected → scaling < 1.0 (motor needs reduction)
        
        // Convert COMMANDED robot velocity (in/s) to expected wheel velocities (ticks/s)
        MecanumKinematics.WheelVelocities commandedWheelVels = 
            MecanumKinematics.robotVelocitiesToWheelVelocities(data.commandedVx, data.commandedVy, 0.0);  // in/s → in/s per wheel
        MecanumKinematics.WheelVelocities expectedWheelTicks = 
            MecanumKinematics.wheelVelocitiesToTicks(commandedWheelVels);  // in/s per wheel → ticks/s per wheel
        
        // Calculate ACTUAL average wheel velocities (measured in ticks/sec from motor encoders)
        double avgWheelFL = data.wheelVelFL.stream().mapToDouble(Double::doubleValue).average().orElse(0);
        double avgWheelFR = data.wheelVelFR.stream().mapToDouble(Double::doubleValue).average().orElse(0);
        double avgWheelBL = data.wheelVelBL.stream().mapToDouble(Double::doubleValue).average().orElse(0);
        double avgWheelBR = data.wheelVelBR.stream().mapToDouble(Double::doubleValue).average().orElse(0);
        
        // Calculate scaling factors based on contribution direction
        updateScalingFactor("FL", expectedWheelTicks.frontLeft, avgWheelFL, flPositive, positiveData, negativeData);
        updateScalingFactor("FR", expectedWheelTicks.frontRight, avgWheelFR, frPositive, positiveData, negativeData);
        updateScalingFactor("BL", expectedWheelTicks.backLeft, avgWheelBL, blPositive, positiveData, negativeData);
        updateScalingFactor("BR", expectedWheelTicks.backRight, avgWheelBR, brPositive, positiveData, negativeData);
    }
    
    private void updateScalingFactor(String wheel, double expected, double actual, boolean isPositive,
                                   Map<String, List<Double>> positiveData,
                                   Map<String, List<Double>> negativeData) {
        
        if (Math.abs(expected) > 10 && Math.abs(actual) > 10) { // Valid data threshold (lowered for real robot velocities)
            // Scaling factor = (expected ticks/s) / (actual ticks/s)
            // Both expected and actual are in ticks/s, ensuring proper unit comparison
            double scalingFactor = expected / actual;
            
            if (isPositive) {
                positiveData.get(wheel).add(scalingFactor);
            } else {
                negativeData.get(wheel).add(scalingFactor);
            }
        }
    }
    
    private double calculateAverage(List<Double> values) {
        // Return 1.0 as default only if truly no data
        // This preserves partial calibration results
        if (values == null || values.isEmpty()) return 1.0;
        return values.stream().mapToDouble(Double::doubleValue).average().orElse(1.0);
    }
    
    private void calculateCalibrationAccuracy() {
        // Calculate RMS error across all tests
        double totalSquaredError = 0;
        int totalSamples = 0;
        
        for (TestData data : testResults.values()) {
            for (int i = 0; i < data.odometryVelX.size(); i++) {
                double errorX = Math.abs(data.commandedVx - data.odometryVelX.get(i));
                double errorY = Math.abs(data.commandedVy - data.odometryVelY.get(i));
                totalSquaredError += errorX * errorX + errorY * errorY;
                totalSamples += 2; // X and Y components
            }
        }
        
        calibrationAccuracy = Math.sqrt(totalSquaredError / totalSamples);
    }
    
    // Utility methods
    private double getTestVelocityInchesPerSec() {
        return _2_MotorVelocityPIDF.TARGET_VELOCITY;
    }
    
}
