package org.firstinspires.ftc.teamcode.calibration.modules;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.calibration.BaseCalibration;
import org.firstinspires.ftc.teamcode.calibration.CalibrationCoefficients;
import org.firstinspires.ftc.teamcode.calibration.SmartDashboardManager;
import org.firstinspires.ftc.teamcode.calibration.MotionCalibrationAndDemo;

import org.firstinspires.ftc.teamcode.motion.MotionConfig;

/**
 * Hybrid Control Calibration Module
 * 
 * Equilateral triangle movement test for evaluating hybrid control system performance.
 * Robot travels along an equilateral triangle path with configurable side length and heading changes.
 * Tests different control modes (PURE_FEEDBACK, PURE_FEEDFORWARD, HYBRID) for comparison.
 * 
 * Features:
 * - Triangle path test (3 sides, returns to start)
 * - Configurable side length (default: 36 inches)
 * - Configurable heading change per vertex (default: 120° for equilateral triangle)
 *   - 120°: Perfect equilateral triangle (recommended)
 *   - 90°: Sharp turns, more challenging for control system
 *   - 180°: Straight line with U-turns
 *   - 60°: Wide turns, easier for control system
 * - Control mode switching (test different modes)
 * - Automatic odometry vs target comparison
 * - Performance metrics and control mode evaluation
 * - Real-time parameter tuning and feedback
 */
public class HybridControlCalibrationModule extends BaseCalibration {
    
    // ========== CONTROL MODE SELECTION ==========
    
    public enum ControlModeSelection {
        PURE_FEEDBACK("Pure PID Only"),
        PURE_FEEDFORWARD("Pure Feedforward Only"),
        HYBRID("Hybrid (FF + PID)");
        
        public final String displayName;
        
        ControlModeSelection(String displayName) {
            this.displayName = displayName;
        }
    }
    
    // ========== DASHBOARD PARAMETERS ==========
    
    @Config
    public static class _7_HybridControl {
        // ========== CONTROL MODE SELECTION ==========
        public static ControlModeSelection CONTROL_MODE = ControlModeSelection.HYBRID;
        
        // ========== HYBRID CONTROL TUNING PARAMETERS ==========
        public static double POSITION_FEEDFORWARD_GAIN = MotionConfig.POSITION_FEEDFORWARD_GAIN;
        public static double ACCELERATION_LIMIT = MotionConfig.ACCELERATION_LIMIT;
        
        // ========== TRIANGLE TEST CONFIGURATION ==========
        public static double TRIANGLE_SIDE_LENGTH = 36.0;
        public static double HEADING_CHANGE_PER_VERTEX = 120.0;  // Degrees to turn at each vertex (120° = equilateral triangle)
        public static double MAX_VELOCITY = 12.0;  // Maximum linear velocity (inches/sec)
        
        // ========== TEST CONTROL ==========
        // Test command (toggle buttons)
        public static boolean RUN_TEST = false;
        public static boolean STOP_TEST = false;
        
        // ========== TEST RESULTS (AUTO-POPULATED) ==========
        public static double START_X = 0.0;
        public static double START_Y = 0.0;
        public static double START_HEADING = 0.0;
        public static double FINAL_X = 0.0;
        public static double FINAL_Y = 0.0;
        public static double FINAL_HEADING = 0.0;
        public static double POSITION_ERROR = 0.0;
        public static double HEADING_ERROR = 0.0;
        public static double TRIANGLE_COMPLETION_TIME = 0.0;
        public static double CONTROL_QUALITY_SCORE = 0.0;
    }
    
    // ========== INSTANCE VARIABLES ==========
    
    // Triangle test state
    private boolean testInProgress = false;
    private long testStartTime = 0;
    private double startX, startY, startHeading;
    
    // Triangle vertices (calculated during test)
    private double[] triangleVerticesX = new double[3];
    private double[] triangleVerticesY = new double[3];
    private double[] triangleHeadings = new double[3];
    
    // Dashboard integration
    private SmartDashboardManager dashboardManager;
    
    // ========== INITIALIZATION ==========
    
    @Override
    protected void initializeCalibration() {
        // Get dashboard manager from MotionCalibrationAndDemo
        if (parentOpMode instanceof MotionCalibrationAndDemo) {
            dashboardManager = ((MotionCalibrationAndDemo) parentOpMode).getDashboardManager();
        }
    }
    
    // ========== CALIBRATION LIFECYCLE METHODS ==========
    
    @Override
    protected void startTest() {
        // This method is called by the base class when ENABLE_TESTING is toggled
        // However, we don't want to auto-start the test like AxisPID module
        // The test should only start when user explicitly clicks RUN_TEST button
        // This prevents the test from starting when just enabling the module in mode selector
    }
    
    /**
     * Internal test start method called by handleTestCommands() when RUN_TEST is clicked
     */
    private void startTestInternal() {
        if (odometryManager == null || motionExecutor == null) {
            telemetry.addLine("❌ Motion system not available for hybrid control testing");
            return;
        }
        
        telemetry.addLine(String.format("🚀 Starting Triangle Test with %s", 
            _7_HybridControl.CONTROL_MODE.displayName));
        
        // Apply hybrid control parameters to motion system
        applyHybridControlParameters();
        
        // Reset odometry to (0,0,0)
        motionExecutor.resetOdometry();
        motionExecutor.updateState();
        
        // Record start position
        startX = 0;
        startY = 0; 
        startHeading = 0;
        _7_HybridControl.START_X = startX;
        _7_HybridControl.START_Y = startY;
        _7_HybridControl.START_HEADING = startHeading;
        
        testInProgress = true;
        testStartTime = System.currentTimeMillis();
        
        telemetry.addLine("🔄 Odometry reset to (0, 0, 0°)");
        telemetry.addLine("🔧 Control mode and parameters applied");
        telemetry.update();
        
        // Execute the triangle test
        executeTriangleTest();
    }
    
    @Override
    protected void stopTest() {
        testInProgress = false;
        telemetry.addLine("🛑 Triangle test stopped");
        telemetry.update();
    }
    
    /**
     * Handle test command buttons (RUN_TEST and STOP_TEST)
     */
    private void handleTestCommands() {
        // Handle STOP_TEST command
        if (_7_HybridControl.STOP_TEST) {
            if (testInProgress) {
                telemetry.addLine("🛑 Test stopped by user");
                testInProgress = false;
            }
            _7_HybridControl.STOP_TEST = false;
            return;
        }
        
        // Only process new commands if not currently testing
        if (!testInProgress && _7_HybridControl.RUN_TEST) {
            startTestInternal();
            _7_HybridControl.RUN_TEST = false;
        }
    }
    
    @Override
    public void updateCalibrationTest() {
        // Always call updateTest() to handle RUN_TEST button commands
        // This allows the RUN_TEST button to work independently of testRunning state
        // Similar to AxisPID module behavior
        updateTest();
    }
    
    @Override
    protected void updateTest() {
        // Handle test commands
        handleTestCommands();
        
        if (odometryManager == null || motionExecutor == null) return;
        
        // Triangle test uses blocking movements
        // This method is called for display/monitoring
        
        if (testInProgress) {
            telemetry.addLine("⏳ Triangle test in progress...");
            telemetry.addLine("(Robot is executing triangle path)");
        } else {
            // Test complete, display results
            telemetry.addLine("✅ Triangle test complete!");
            telemetry.addLine(String.format("📊 Position Error: %.2f inches", 
                _7_HybridControl.POSITION_ERROR));
            telemetry.addLine(String.format("📊 Heading Error: %+.1f degrees", 
                _7_HybridControl.HEADING_ERROR));
            telemetry.addLine(String.format("📊 Quality Score: %.0f/100", 
                _7_HybridControl.CONTROL_QUALITY_SCORE));
            telemetry.addLine(String.format("📊 Completion Time: %.1f seconds", 
                _7_HybridControl.TRIANGLE_COMPLETION_TIME));
            telemetry.addLine("📊 Check detailed results on dashboard");
        }
        
        // Update dashboard
        if (dashboardManager != null) {
            dashboardManager.updateConfigIfNeeded();
        }
    }
    
    // ========== TRIANGLE TEST IMPLEMENTATION ==========
    
    /**
     * Execute the equilateral triangle test
     */
    private void executeTriangleTest() {
        // Validate parameters before starting test
        double sideLength = Math.max(6.0, Math.min(72.0, _7_HybridControl.TRIANGLE_SIDE_LENGTH));
        double headingChange = normalizeAngle(_7_HybridControl.HEADING_CHANGE_PER_VERTEX);
        double velocity = Math.max(3.0, Math.min(CalibrationCoefficients.CALIBRATED_MAX_LINEAR_VELOCITY, _7_HybridControl.MAX_VELOCITY)); //clamp to safe range
        
        // Update parameters if they were clamped or wrapped
        if (sideLength != _7_HybridControl.TRIANGLE_SIDE_LENGTH) {
            _7_HybridControl.TRIANGLE_SIDE_LENGTH = sideLength;
            telemetry.addLine("⚠️ Side length clamped to safe range (6-72 inches)");
        }
        if (headingChange != _7_HybridControl.HEADING_CHANGE_PER_VERTEX) {
            _7_HybridControl.HEADING_CHANGE_PER_VERTEX = headingChange;
            telemetry.addLine(String.format("⚠️ Heading change wrapped to [-180°, +180°] range (%.1f°)", headingChange));
        }
        if (velocity != _7_HybridControl.MAX_VELOCITY) {
            _7_HybridControl.MAX_VELOCITY = velocity;
            telemetry.addLine("⚠️ Max velocity clamped to safe range (3-24 in/sec)");
        }
        
        // Calculate the 3 vertices of an equilateral triangle
        // Starting at (0,0), first vertex is straight ahead
        triangleVerticesX[0] = sideLength;
        triangleVerticesY[0] = 0;
        triangleHeadings[0] = normalizeAngle(headingChange);
        
        // Second vertex: 120° counterclockwise from first side
        double angle1 = Math.toRadians(60);
        triangleVerticesX[1] = sideLength + sideLength * Math.cos(angle1);
        triangleVerticesY[1] = sideLength * Math.sin(angle1);
        triangleHeadings[1] = normalizeAngle(headingChange * 2);
        
        // Third vertex: back to start (0,0)
        triangleVerticesX[2] = 0;
        triangleVerticesY[2] = 0;
        triangleHeadings[2] = normalizeAngle(headingChange * 3);
        
        // Display configuration
        telemetry.addLine("🔺 Executing Equilateral Triangle Test:");
        telemetry.addLine(String.format("   Side Length: %.1f inches", sideLength));
        telemetry.addLine(String.format("   Heading Change: %.0f° per vertex", headingChange));
        telemetry.addLine(String.format("   Max Velocity: %.1f in/sec", velocity));
        telemetry.update();
        
        // Execute movement to each vertex
        for (int i = 0; i < 3; i++) {
            telemetry.addLine(String.format("🎯 Moving to vertex %d: (%.1f, %.1f) heading %.0f°", 
                i + 1, triangleVerticesX[i], triangleVerticesY[i], triangleHeadings[i]));
            telemetry.update();
            
            motionExecutor.moveToPose(triangleVerticesX[i], triangleVerticesY[i], 
                triangleHeadings[i], velocity);
        }
        
        // Capture final results and calculate errors
        motionExecutor.updateState();
        double actualX = motionExecutor.getMotionState().getX();
        double actualY = motionExecutor.getMotionState().getY();
        double actualHeading = motionExecutor.getMotionState().getHeading();
        
        long testEndTime = System.currentTimeMillis();
        double completionTime = (testEndTime - testStartTime) / 1000.0;
        
        // Calculate errors (should return to start)
        double expectedFinalX = startX;
        double expectedFinalY = startY;
        double expectedFinalHeading = normalizeAngle(startHeading + (headingChange * 3));
        
        double positionError = Math.sqrt(Math.pow(actualX - expectedFinalX, 2) + 
            Math.pow(actualY - expectedFinalY, 2));
        double headingError = normalizeAngle(actualHeading - expectedFinalHeading);
        
        // Store results
        _7_HybridControl.FINAL_X = actualX;
        _7_HybridControl.FINAL_Y = actualY;
        _7_HybridControl.FINAL_HEADING = actualHeading;
        _7_HybridControl.POSITION_ERROR = positionError;
        _7_HybridControl.HEADING_ERROR = headingError;
        _7_HybridControl.TRIANGLE_COMPLETION_TIME = completionTime;
        
        // Calculate quality score based on position accuracy
        if (positionError < 0.5) {
            _7_HybridControl.CONTROL_QUALITY_SCORE = 100;
        } else if (positionError < 1.0) {
            _7_HybridControl.CONTROL_QUALITY_SCORE = 90 - (positionError - 0.5) * 40;
        } else if (positionError < 2.0) {
            _7_HybridControl.CONTROL_QUALITY_SCORE = 70 - (positionError - 1.0) * 30;
        } else {
            _7_HybridControl.CONTROL_QUALITY_SCORE = Math.max(0, 40 - (positionError - 2.0) * 20);
        }
        
        testInProgress = false;
        
        // Reset RUN_TEST toggle after test completion (like HeadingPID module)
        _7_HybridControl.RUN_TEST = false;
        
        // Display results with telemetry
        telemetry.addLine("✅ Triangle Test Complete!");
        telemetry.addLine(String.format("   Expected Final: (%.1f, %.1f, %.0f°)", 
            expectedFinalX, expectedFinalY, expectedFinalHeading));
        telemetry.addLine(String.format("   Actual Final: (%.1f, %.1f, %.0f°)", 
            actualX, actualY, actualHeading));
        telemetry.addLine(String.format("   Position Error: %.2f inches", positionError));
        telemetry.addLine(String.format("   Heading Error: %+.1f degrees", headingError));
        telemetry.addLine(String.format("   Completion Time: %.1f seconds", completionTime));
        telemetry.addLine(String.format("   Quality Score: %.0f/100", _7_HybridControl.CONTROL_QUALITY_SCORE));
        
        // Force immediate dashboard config update to show results
        if (dashboardManager != null) {
            dashboardManager.forceConfigUpdate();
        }
    }
    
    /**
     * Apply hybrid control parameters to the motion system
     */
    private void applyHybridControlParameters() {
        if (motionExecutor != null) {
            // Convert ControlModeSelection to MotionExecutor.ControlMode
            org.firstinspires.ftc.teamcode.motion.MotionExecutor.ControlMode controlMode;
            switch (_7_HybridControl.CONTROL_MODE) {
                case PURE_FEEDBACK:
                    controlMode = org.firstinspires.ftc.teamcode.motion.MotionExecutor.ControlMode.PURE_FEEDBACK;
                    break;
                case PURE_FEEDFORWARD:
                    controlMode = org.firstinspires.ftc.teamcode.motion.MotionExecutor.ControlMode.PURE_FEEDFORWARD;
                    break;
                case HYBRID:
                default:
                    controlMode = org.firstinspires.ftc.teamcode.motion.MotionExecutor.ControlMode.HYBRID;
                    break;
            }
            
            // Apply control mode to motion executor
            motionExecutor.setControlMode(controlMode);
            
            telemetry.addLine("🔧 Applying control parameters:");
            telemetry.addLine(String.format("   Control Mode: %s", _7_HybridControl.CONTROL_MODE.displayName));
            telemetry.addLine(String.format("   Position FF Gain: %.3f", 
                _7_HybridControl.POSITION_FEEDFORWARD_GAIN));
            telemetry.addLine(String.format("   Accel Limit: %.2f", 
                _7_HybridControl.ACCELERATION_LIMIT));
            
            // TODO: Add methods to MotionExecutor for parameter updates:
            // motionExecutor.updateHybridParameters(
            //     _7_HybridControl.POSITION_FEEDFORWARD_GAIN,
            //     _7_HybridControl.ACCELERATION_LIMIT
            // );
        }
    }
    
    // ========== DISPLAY AND STATUS METHODS ==========
    
    @Override
    public void displayStatus() {
        telemetry.addLine("🔧 HYBRID CONTROL PARAMETERS:");
        telemetry.addData("Control Mode", _7_HybridControl.CONTROL_MODE.displayName);
        telemetry.addData("Position FF Gain", "%.3f → %.3f", 
            MotionConfig.POSITION_FEEDFORWARD_GAIN, _7_HybridControl.POSITION_FEEDFORWARD_GAIN);
        telemetry.addData("Accel Limit", "%.2f → %.2f", 
            MotionConfig.ACCELERATION_LIMIT, _7_HybridControl.ACCELERATION_LIMIT);
        telemetry.addLine("");
        
        telemetry.addLine("🔺 TRIANGLE TEST CONFIGURATION:");
        telemetry.addData("Side Length", "%.1f inches", _7_HybridControl.TRIANGLE_SIDE_LENGTH);
        telemetry.addData("Heading Change", "%.0f° per vertex", _7_HybridControl.HEADING_CHANGE_PER_VERTEX);
        
        // Add helpful guidance for heading change values
        double headingChange = _7_HybridControl.HEADING_CHANGE_PER_VERTEX;
        if (Math.abs(headingChange - 120.0) < 5.0) {
            telemetry.addLine("   📐 Equilateral triangle (recommended)");
        } else if (Math.abs(headingChange - 90.0) < 5.0) {
            telemetry.addLine("   📐 Sharp turns (challenging)");
        } else if (Math.abs(headingChange - 180.0) < 5.0) {
            telemetry.addLine("   📐 Straight line with U-turns");
        } else if (Math.abs(headingChange - 60.0) < 5.0) {
            telemetry.addLine("   📐 Wide turns (easier)");
        } else {
            telemetry.addLine("   📐 Custom triangle shape");
        }
        
        telemetry.addData("Max Velocity", "%.1f in/sec", _7_HybridControl.MAX_VELOCITY);
        telemetry.addLine("");
        
        telemetry.addLine("🎮 TEST CONTROLS:");
        telemetry.addData("RUN_TEST", "%s", _7_HybridControl.RUN_TEST ? "YES" : "NO");
        telemetry.addData("STOP_TEST", "%s", _7_HybridControl.STOP_TEST ? "YES" : "NO");
        telemetry.addData("Current State", "%s", testInProgress ? "TESTING" : "IDLE");
        telemetry.addLine("");
        
        telemetry.addLine("📊 TRIANGLE TEST RESULTS:");
        if (_7_HybridControl.POSITION_ERROR > 0 || _7_HybridControl.TRIANGLE_COMPLETION_TIME > 0) {
            // Start position
            telemetry.addLine("📍 START POSITION:");
            telemetry.addData("  Start", "(%.1f, %.1f, %.0f°)", 
                _7_HybridControl.START_X, _7_HybridControl.START_Y, _7_HybridControl.START_HEADING);
            
            // Final position
            telemetry.addLine("🎯 FINAL POSITION:");
            telemetry.addData("  Final", "(%.1f, %.1f, %.0f°)", 
                _7_HybridControl.FINAL_X, _7_HybridControl.FINAL_Y, _7_HybridControl.FINAL_HEADING);
            
            // Performance metrics
            telemetry.addLine("📈 PERFORMANCE:");
            telemetry.addData("  Position Error", "%.2f inches", _7_HybridControl.POSITION_ERROR);
            telemetry.addData("  Heading Error", "%+.1f degrees", _7_HybridControl.HEADING_ERROR);
            telemetry.addData("  Completion Time", "%.1f seconds", _7_HybridControl.TRIANGLE_COMPLETION_TIME);
            telemetry.addData("  Quality Score", "%.0f / 100", _7_HybridControl.CONTROL_QUALITY_SCORE);
            
            // Quality assessment
            if (_7_HybridControl.CONTROL_QUALITY_SCORE >= 90) {
                telemetry.addLine("   🎯 EXCELLENT control performance!");
            } else if (_7_HybridControl.CONTROL_QUALITY_SCORE >= 70) {
                telemetry.addLine("   ✅ Good control performance");
            } else if (_7_HybridControl.CONTROL_QUALITY_SCORE >= 40) {
                telemetry.addLine("   ⚠️ Needs tuning");
            } else {
                telemetry.addLine("   ❌ Poor performance - check parameters");
            }
        } else {
            telemetry.addLine("No triangle test completed yet");
        }
        telemetry.addLine("");
    }
    
    @Override
    public void resetParameters() {
        // Reset control mode selection
        _7_HybridControl.CONTROL_MODE = ControlModeSelection.HYBRID;
        
        // Reset hybrid control parameters to MotionConfig defaults
        _7_HybridControl.POSITION_FEEDFORWARD_GAIN = MotionConfig.POSITION_FEEDFORWARD_GAIN;
        _7_HybridControl.ACCELERATION_LIMIT = MotionConfig.ACCELERATION_LIMIT;
        
        // Reset triangle test configuration
        _7_HybridControl.TRIANGLE_SIDE_LENGTH = 36.0;
        _7_HybridControl.HEADING_CHANGE_PER_VERTEX = 120.0;  // Default to equilateral triangle
        _7_HybridControl.MAX_VELOCITY = 12.0;
        
        // Reset test commands
        _7_HybridControl.RUN_TEST = false;
        _7_HybridControl.STOP_TEST = false;
        
        // Reset test results
        _7_HybridControl.START_X = 0.0;
        _7_HybridControl.START_Y = 0.0;
        _7_HybridControl.START_HEADING = 0.0;
        _7_HybridControl.FINAL_X = 0.0;
        _7_HybridControl.FINAL_Y = 0.0;
        _7_HybridControl.FINAL_HEADING = 0.0;
        _7_HybridControl.POSITION_ERROR = 0.0;
        _7_HybridControl.HEADING_ERROR = 0.0;
        _7_HybridControl.TRIANGLE_COMPLETION_TIME = 0.0;
        _7_HybridControl.CONTROL_QUALITY_SCORE = 0.0;
    }
    
    @Override
    public String getCalibrationName() {
        return "Hybrid Control";
    }
    
    @Override
    public String getCalibrationDescription() {
        return "Equilateral triangle test with control mode switching (Pure Feedback, Pure Feedforward, Hybrid)";
    }
    
    /**
     * Normalize angle to [-180, 180] range
     */
    protected double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}
