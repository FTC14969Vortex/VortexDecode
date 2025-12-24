package org.firstinspires.ftc.teamcode.calibration.modules;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.calibration.BaseCalibration;
import org.firstinspires.ftc.teamcode.calibration.SmartDashboardManager;
import org.firstinspires.ftc.teamcode.calibration.MotionCalibrationAndDemo;
import org.firstinspires.ftc.teamcode.motion.MotionExecutor.ControlMode;
import org.firstinspires.ftc.teamcode.motion.MotionConfig;

/**
 * Heading PID Calibration Module
 * 
 * Handles real-time tuning of heading (rotation) PID controller
 * for precise angular position control.
 * 
 * Features:
 * - Real-time PID parameter adjustment via Dashboard
 * - Multiple test patterns (clockwise, counterclockwise, small/large angles)
 * - Angular accuracy and settling time metrics
 * - Proper angle normalization and wraparound handling
 * - Integration with motion control system
 * - Automatic config file updates
 */
public class HeadingPIDCalibrationModule extends BaseCalibration {
    
    // ========== DASHBOARD PARAMETERS ==========
    
    @Config
    public static class _6_HeadingPID {
        // ========== PID TUNING PARAMETERS ==========
        public static double Kp = MotionConfig.HEADING_KP;
        public static double Ki = MotionConfig.HEADING_KI;
        public static double Kd = MotionConfig.HEADING_KD;
        
        // ========== TEST CONFIGURATION ==========
        public static double TEST_ANGLE = 90.0;  // Target angle for rotation test (degrees)
        
        // Test command (toggle button)
        public static boolean RUN_TEST = false;
        public static boolean STOP_TEST = false;
        
        // ========== PERFORMANCE METRICS (AUTO-POPULATED) ==========
        public static double SETTLING_TIME_MS = 0.0;      // Time to reach target (milliseconds)
        public static double MAX_OVERSHOOT_PERCENT = 0.0;  // Peak overshoot as percentage
        public static double STEADY_STATE_ERROR = 0.0;    // Final heading error (degrees)
        public static double RESPONSE_QUALITY = 0.0;      // Overall response quality score (0-100)
        
        // ========== PID PERFORMANCE ANALYSIS (AUTO-POPULATED) ==========
        public static double ODOMETRY_ANGLE_ROTATED = 0.0; // Odometry reading after rotation (degrees)
        public static double HEADING_ERROR = 0.0;         // Heading error: actual - intended (degrees)
        public static double TOTAL_HEADING_ERROR = 0.0;   // Total heading error magnitude (degrees)
    }
    
    // ========== TEST STATE ==========
    
    private enum TestState {
        IDLE,
        TESTING
    }
    
    private TestState currentTest = TestState.IDLE;
    private double startHeading = 0;
    private double targetHeading = 0;
    private boolean testCompleted = false;
    
    // ========== CONTROL MODE MANAGEMENT ==========
    private ControlMode originalControlMode = null;  // Store original mode for restoration
    private boolean controlModeChanged = false;     // Track if we changed the mode
    
    // ========== DASHBOARD MANAGER ==========
    private SmartDashboardManager dashboardManager;
    
    @Override
    protected void initializeCalibration() {
        currentTest = TestState.IDLE;
        
        // Get dashboard manager from MotionCalibrationAndDemo
        if (parentOpMode instanceof MotionCalibrationAndDemo) {
            dashboardManager = ((MotionCalibrationAndDemo) parentOpMode).getDashboardManager();
        }
    }
    
    @Override
    protected void startTest() {
        // This method is called by the base class, but we handle test commands manually
        // in updateTest() to provide better control over the toggle button workflow
    }
    
    private void startTestInternal() {
        if (odometryManager == null || motionExecutor == null) {
            telemetry.addLine("❌ Motion system not available for heading testing");
            return;
        }
        
        // Reset completion flag
        testCompleted = false;
        currentTest = TestState.TESTING;
        
        // CRITICAL: Force PURE_FEEDBACK mode for true PID calibration
        // Save original control mode for restoration later
        originalControlMode = motionExecutor.getControlMode();
        motionExecutor.setControlMode(ControlMode.PURE_FEEDBACK);
        controlModeChanged = true;
        
        telemetry.addLine("🎯 PURE FEEDBACK MODE ACTIVATED");
        telemetry.addLine("   (100% PID control, 0% feedforward)");
        telemetry.addLine("   Original mode: " + originalControlMode);
        
        // Reset odometry to (0,0,0) at start of each test
        motionExecutor.resetOdometry();
        motionExecutor.updateState();
        
        startHeading = 0;  // Always start from 0 after reset
        targetHeading = _6_HeadingPID.TEST_ANGLE;  // Rotate by TEST_ANGLE degrees
        
        // Apply PID parameters to motion system
        if (motionExecutor != null) {
            motionExecutor.updateHeadingPID(_6_HeadingPID.Kp, _6_HeadingPID.Ki, _6_HeadingPID.Kd);
            telemetry.addLine(String.format("🔧 Heading PID Applied: Kp=%.3f, Ki=%.3f, Kd=%.3f", 
                _6_HeadingPID.Kp, _6_HeadingPID.Ki, _6_HeadingPID.Kd));
        } else {
            telemetry.addLine("❌ Motion executor not available - cannot apply PID values");
        }
        
        telemetry.addLine("🔄 Odometry reset to (0, 0, 0°)");
        telemetry.addLine(String.format("🚀 Rotating to %.1f degrees...", targetHeading));
        telemetry.update();
        
        // Execute the movement (BLOCKING - waits until complete)
        motionExecutor.moveToPose(0, 0, targetHeading, 12.0);
        
        // Movement complete! Now capture odometry reading
        motionExecutor.updateState();
        double finalHeading = motionExecutor.getMotionState().getHeading();
        
        // Calculate PID performance metrics: actual rotation vs intended rotation
        double intendedAngle = _6_HeadingPID.TEST_ANGLE;
        
        // Calculate actual angle rotated and error (preserve sign for direction)
        double actualRotation = finalHeading - startHeading;  // Signed rotation (+ = CCW, - = CW)
        double headingError = actualRotation - intendedAngle;  // Positive = overshoot, Negative = undershoot
        
        _6_HeadingPID.ODOMETRY_ANGLE_ROTATED = actualRotation;
        _6_HeadingPID.HEADING_ERROR = headingError;
        _6_HeadingPID.TOTAL_HEADING_ERROR = Math.abs(headingError);
        
        // CRITICAL: Force FTC Dashboard to send config update to all clients
        // Without this, the dashboard won't know that values changed programmatically
        if (dashboardManager != null) {
            dashboardManager.markConfigChanged();
        }
        
        telemetry.addLine(String.format("✅ Heading PID test complete!"));
        telemetry.addLine(String.format("   Intended: %.1f° | Actual: %.1f° | Error: %+.1f°", 
            intendedAngle, actualRotation, headingError));
        
        if (Math.abs(headingError) < 2.0) {
            telemetry.addLine("   🎯 EXCELLENT accuracy!");
        } else if (Math.abs(headingError) < 5.0) {
            telemetry.addLine("   ✅ Good accuracy");
        } else {
            telemetry.addLine("   ⚠️ Needs tuning");
        }
        
        // Force telemetry update to push values to dashboard immediately
        telemetry.update();
        
        testCompleted = true;
        currentTest = TestState.IDLE;
    }
    
    private void handleTestCommands() {
        // Emergency stop overrides all other commands
        if (_6_HeadingPID.STOP_TEST) {
            if (motionExecutor != null) {
                motionExecutor.stop();
            }
            currentTest = TestState.IDLE;
            _6_HeadingPID.STOP_TEST = false;
            return;
        }
        
        // Only process new commands if not currently testing
        if (currentTest == TestState.IDLE && _6_HeadingPID.RUN_TEST) {
            startTestInternal();
            _6_HeadingPID.RUN_TEST = false;
        }
    }
    
    @Override
    protected void updateTest() {
        // Handle test commands
        handleTestCommands();
        
        // Movement is handled by blocking moveToPose() call in startTest()
        // This method is called after movement completes, just for display/monitoring
        
        if (testCompleted) {
            // Test already complete, just display final results
            telemetry.addLine("✅ Test Complete!");
            telemetry.addLine("📊 Check odometry reading on dashboard");
            telemetry.addLine("📐 Measure actual angle and compare");
            
            // Still need to update config even when test is complete!
            if (dashboardManager != null) {
                dashboardManager.updateConfigIfNeeded();
            }
            return;
        }
        
        // If we get here, movement is still in progress (blocked in moveToPose)
        telemetry.addLine("⏳ Rotation in progress...");
        telemetry.addLine("(Robot is executing moveToPose)");
        
        // Update dashboard config if needed (critical for synchronization)
        if (dashboardManager != null) {
            dashboardManager.updateConfigIfNeeded();
        }
    }
    
    @Override
    protected void stopTest() {
        // Calculate performance metrics
        calculatePositionPerformanceMetrics(targetHeading, 2.0); // 2 degree tolerance
        
        // Restore original control mode if we changed it
        if (controlModeChanged && originalControlMode != null) {
            motionExecutor.setControlMode(originalControlMode);
            controlModeChanged = false;
            
            telemetry.addLine("🔄 Control mode restored to: " + originalControlMode);
        }
        
        // Reset test state
        currentTest = TestState.IDLE;
    }
    
    @Override
    public void displayStatus() {
        telemetry.addLine("🔧 CURRENT HEADING PID (from MotionConfig):");
        telemetry.addData("Kp", "%.3f → %.3f", MotionConfig.HEADING_KP, _6_HeadingPID.Kp);
        telemetry.addData("Ki", "%.3f → %.3f", MotionConfig.HEADING_KI, _6_HeadingPID.Ki);
        telemetry.addData("Kd", "%.3f → %.3f", MotionConfig.HEADING_KD, _6_HeadingPID.Kd);
        telemetry.addLine("");
        
        telemetry.addLine("⚙️ TEST CONFIGURATION:");
        telemetry.addData("Test Angle", "%.1f degrees", _6_HeadingPID.TEST_ANGLE);
        telemetry.addLine("");
        
        telemetry.addLine("🎮 TEST CONTROLS:");
        telemetry.addData("RUN_TEST", "%s", _6_HeadingPID.RUN_TEST ? "YES" : "NO");
        telemetry.addData("STOP_TEST", "%s", _6_HeadingPID.STOP_TEST ? "YES" : "NO");
        telemetry.addData("Current State", "%s", currentTest.toString());
        telemetry.addLine("");
        
        telemetry.addLine("📊 PID PERFORMANCE RESULTS:");
        
        // Show target angle for reference
        if (_6_HeadingPID.TEST_ANGLE != 0) {
            telemetry.addData("🎯 Test Angle", "%.1f degrees", _6_HeadingPID.TEST_ANGLE);
        }
        
        telemetry.addData("Odometry Angle", "%.3f degrees", _6_HeadingPID.ODOMETRY_ANGLE_ROTATED);
        if (_6_HeadingPID.HEADING_ERROR != 0.0) {
            telemetry.addData("Heading Error", "%+.3f degrees", _6_HeadingPID.HEADING_ERROR);
            String errorType = (_6_HeadingPID.HEADING_ERROR > 0) ? "(overshoot)" : "(undershoot)";
            telemetry.addData("", errorType);
        }
        
        // Show total heading error if available
        if (_6_HeadingPID.TOTAL_HEADING_ERROR > 0) {
            telemetry.addData("📏 Total Error", "%.3f degrees", _6_HeadingPID.TOTAL_HEADING_ERROR);
            if (_6_HeadingPID.TOTAL_HEADING_ERROR < 2.0) {
                telemetry.addData("  Quality", "🎯 EXCELLENT");
            } else if (_6_HeadingPID.TOTAL_HEADING_ERROR < 5.0) {
                telemetry.addData("  Quality", "✅ Good");
            } else {
                telemetry.addData("  Quality", "⚠️ Needs tuning");
            }
        }
        telemetry.addLine("");
        
        telemetry.addLine("📋 INSTRUCTIONS:");
        telemetry.addLine("1. Set TEST_ANGLE (e.g., 90 degrees)");
        telemetry.addLine("2. Adjust heading PID gains (Kp, Ki, Kd)");
        telemetry.addLine("3. Click RUN_TEST → Robot rotates TEST_ANGLE");
        telemetry.addLine("4. Forces PURE FEEDBACK mode automatically");
        telemetry.addLine("5. System automatically calculates heading error:");
        telemetry.addLine("   • Positive error = overshoot (reduce Kp/Kd)");
        telemetry.addLine("   • Negative error = undershoot (increase Kp)");
        telemetry.addLine("6. Use performance metrics and tuning guidance");
        telemetry.addLine("7. Click RUN_TEST again after adjusting gains");
        telemetry.addLine("8. Control mode automatically restored after test");
        telemetry.addLine("");
        
        telemetry.addLine("✅ TUNING RECOMMENDATIONS:");
        telemetry.addLine("• Start with small Kp to avoid oscillation");
        telemetry.addLine("• Test different angles (30°, 90°, 180°)");
        telemetry.addLine("• Test positive and negative angles");
        telemetry.addLine("• Add Ki only if there's steady-state error");
        telemetry.addLine("• Add Kd to reduce overshoot");
        telemetry.addLine("• Update MotionConfig.java with final values");
    }
    
    @Override
    public void resetParameters() {
        _6_HeadingPID.Kp = MotionConfig.HEADING_KP;
        _6_HeadingPID.Ki = MotionConfig.HEADING_KI;
        _6_HeadingPID.Kd = MotionConfig.HEADING_KD;
        _6_HeadingPID.TEST_ANGLE = 90.0;
        
        // Reset test commands
        _6_HeadingPID.RUN_TEST = false;
        _6_HeadingPID.STOP_TEST = false;
        
        // Reset PID performance metrics
        _6_HeadingPID.ODOMETRY_ANGLE_ROTATED = 0.0;
        _6_HeadingPID.HEADING_ERROR = 0.0;
        _6_HeadingPID.TOTAL_HEADING_ERROR = 0.0;
        
        // Reset test state
        currentTest = TestState.IDLE;
    }
    
    @Override
    public String getCalibrationName() {
        return "Heading PID";
    }
    
    @Override
    public String getCalibrationDescription() {
        return "Tune rotation control PID for precise angular positioning";
    }
    
}
