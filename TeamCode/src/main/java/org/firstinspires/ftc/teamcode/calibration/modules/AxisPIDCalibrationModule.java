package org.firstinspires.ftc.teamcode.calibration.modules;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.calibration.BaseCalibration;
import org.firstinspires.ftc.teamcode.calibration.SmartDashboardManager;
import org.firstinspires.ftc.teamcode.calibration.MotionCalibrationAndDemo;

import org.firstinspires.ftc.teamcode.motion.MotionConfig;
import org.firstinspires.ftc.teamcode.motion.MotionExecutor.ControlMode;
import org.firstinspires.ftc.teamcode.motion.MotionExecutor.MotionResult;

/**
 * Distance-Based PID Calibration Module (2-PID System)
 * 
 * Handles real-time tuning of distance PID controller for precise linear movement
 * in the new 2-PID architecture where distance and heading are controlled separately.
 * 
 * Features:
 * - Distance-based PID testing using linearMove()
 * - Configurable test distance (default 24 inches)
 * - Configurable test angle (any direction)
 * - Pure feedback mode for accurate PID isolation
 * - Comprehensive distance performance metrics
 * - Real-time parameter adjustment via Dashboard
 */
public class AxisPIDCalibrationModule extends BaseCalibration {
    
    // ========== DASHBOARD PARAMETERS ==========
    
    @Config
    public static class _5_DistancePID {
        // ========== DISTANCE PID CONFIGURATION ==========
        // Distance PID gains for the true 2PID system (distance + heading controllers)
        public static double DISTANCE_KP = MotionConfig.DISTANCE_KP;
        public static double DISTANCE_KI = MotionConfig.DISTANCE_KI;
        public static double DISTANCE_KD = MotionConfig.DISTANCE_KD;
        
        // ========== TEST CONFIGURATION ==========
        public static double TEST_DISTANCE = 24.0;        // Distance to move in inches
        public static double TEST_ANGLE = 0.0;            // Direction to move in degrees (0 = forward)
        public static double MAX_VELOCITY = MotionConfig.MAX_LINEAR_VELOCITY; // Test velocity
        
        // Test commands
        public static boolean RUN_TEST = false;
        public static boolean STOP_TEST = false;
        
        // ========== PERFORMANCE METRICS (AUTO-POPULATED) ==========
        public static double SETTLING_TIME_MS = 0.0;      // Time to reach 98% of target distance
        public static double MAX_OVERSHOOT_PERCENT = 0.0;  // Peak overshoot as percentage
        public static double STEADY_STATE_ERROR = 0.0;    // Final distance error (inches)
        public static double RESPONSE_QUALITY = 0.0;      // Overall quality score (0-100)
        public static double LINEAR_ACCURACY = 0.0;       // Straight-line accuracy score (0-100)
        
        // ========== DISTANCE-BASED ANALYSIS (AUTO-POPULATED) ==========
        public static double ACTUAL_DISTANCE_TRAVELED = 0.0; // Total distance from odometry (inches)
        public static double DISTANCE_ERROR = 0.0;           // Distance error: actual - target (inches)
        public static double PEAK_DISTANCE_REACHED = 0.0;    // Maximum distance during movement (inches)
        public static double TIME_TO_TARGET = 0.0;           // Time to reach target distance (milliseconds)
    }
    
    // ========== TEST STATE ==========
    
    private enum TestState {
        IDLE,
        TESTING,
        COMPLETED
    }
    
    private TestState currentTest = TestState.IDLE;
    private double startX = 0, startY = 0;
    private long testStartTime = 0;
    private double maxDistanceReached = 0;
    private boolean hasSettled = false;
    private boolean testCompleted = false;
    
    // ========== CONTROL MODE MANAGEMENT ==========
    
    private ControlMode originalControlMode = null;  // Store original mode for restoration
    private boolean controlModeChanged = false;     // Track if we changed the mode
    
    // ========== DASHBOARD INTEGRATION ==========
    
    private SmartDashboardManager dashboardManager;
    
    @Override
    protected void initializeCalibration() {
        // Initialize distance-based test state
        currentTest = TestState.IDLE;
        testCompleted = false;
        hasSettled = false;
        maxDistanceReached = 0;
        
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
            return;
        }
        
        // Reset test state
        testCompleted = false;
        hasSettled = false;
        maxDistanceReached = 0;
        currentTest = TestState.TESTING;
        
        // CRITICAL: Force PURE_FEEDBACK mode for true distance PID calibration
        // AxisPID is specifically designed for pure feedback calibration
        originalControlMode = motionExecutor.getControlMode();
        motionExecutor.setControlMode(ControlMode.PURE_FEEDBACK);
        controlModeChanged = true;
        
        telemetry.addLine("🎯 PURE FEEDBACK MODE ACTIVATED");
        telemetry.addLine("   (100% Distance PID control, 0% feedforward)");
        telemetry.addLine("   Original mode: " + originalControlMode);
        
        // CRITICAL: Reset odometry to (0,0) at start of each test
        motionExecutor.resetToFieldOrigin();
        motionExecutor.updateState();
        startX = 0;  // Always start from origin after reset
        startY = 0;
        testStartTime = System.currentTimeMillis();
        
        // Apply distance PID values to the actual distance controller (2-PID architecture)
        motionExecutor.updateDistancePID(_5_DistancePID.DISTANCE_KP, _5_DistancePID.DISTANCE_KI, _5_DistancePID.DISTANCE_KD);
        
        telemetry.addLine("🔄 Odometry reset to (0, 0)");
        telemetry.addLine(String.format("🚀 Starting distance test: %.1f inches at %.1f°", 
            _5_DistancePID.TEST_DISTANCE, _5_DistancePID.TEST_ANGLE));
        telemetry.addLine(String.format("⚙️ Distance PID: Kp=%.3f, Ki=%.3f, Kd=%.3f", 
            _5_DistancePID.DISTANCE_KP, _5_DistancePID.DISTANCE_KI, _5_DistancePID.DISTANCE_KD));
        
        executeDistanceTest();
    }
    
    private void executeDistanceTest() {
        // Execute the distance-based movement using linearMove()
        MotionResult result = motionExecutor.linearMove(
            _5_DistancePID.TEST_ANGLE, 
            _5_DistancePID.TEST_DISTANCE, 
            _5_DistancePID.MAX_VELOCITY
        );
        
        // Movement complete! Now capture odometry reading
        motionExecutor.updateState();
        double finalX = motionExecutor.getMotionState().getX();
        double finalY = motionExecutor.getMotionState().getY();
        
        // Calculate distance-based performance metrics
        calculateDistancePerformanceMetrics();
        
        // Store the result for analysis
        if (result.success) {
            telemetry.addLine("✅ Distance movement completed successfully");
            currentTest = TestState.COMPLETED;
            
            // Display immediate results
            double actualDistance = _5_DistancePID.ACTUAL_DISTANCE_TRAVELED;
            double distanceError = _5_DistancePID.DISTANCE_ERROR;
            
            telemetry.addLine(String.format("📏 Distance Test Results:"));
            telemetry.addLine(String.format("   Target: %.3f\" | Actual: %.3f\" | Error: %+.3f\"", 
                _5_DistancePID.TEST_DISTANCE, actualDistance, distanceError));
            
            // Quality assessment
            if (Math.abs(distanceError) < 0.5) {
                telemetry.addLine("   🎯 EXCELLENT accuracy!");
            } else if (Math.abs(distanceError) < 1.0) {
                telemetry.addLine("   ✅ Good accuracy");
            } else if (Math.abs(distanceError) < 2.0) {
                telemetry.addLine("   ⚠️ Fair accuracy - consider tuning");
            } else {
                telemetry.addLine("   ❌ Poor accuracy - needs tuning");
            }
            
        } else {
            telemetry.addLine("❌ Distance movement failed: " + result.failureReason);
            currentTest = TestState.IDLE;
        }
        
        // CRITICAL: Mark config as changed for batched updates
        if (dashboardManager != null) {
            dashboardManager.markConfigChanged();
        }
        
        // Force telemetry update to push values to dashboard immediately
        telemetry.update();
        
        // Restore original control mode
        if (controlModeChanged && originalControlMode != null) {
            motionExecutor.setControlMode(originalControlMode);
            controlModeChanged = false;
            telemetry.addLine("🔄 Control mode restored to: " + originalControlMode);
        }
        
        testCompleted = true;
        currentTest = TestState.IDLE;
    }
    
    /**
     * Calculate distance-based performance metrics for the 2-PID system
     */
    private void calculateDistancePerformanceMetrics() {
        if (motionExecutor == null) return;
        
        // Get final position from odometry
        motionExecutor.updateState();
        double finalX = motionExecutor.getMotionState().getX();
        double finalY = motionExecutor.getMotionState().getY();
        
        // Calculate actual distance traveled from starting position
        double deltaX = finalX - startX;
        double deltaY = finalY - startY;
        double actualDistance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        
        // Calculate distance error
        double targetDistance = _5_DistancePID.TEST_DISTANCE;
        double distanceError = actualDistance - targetDistance;
        
        // Calculate test duration
        double testDuration = System.currentTimeMillis() - testStartTime;
        
        // Store basic metrics
        _5_DistancePID.ACTUAL_DISTANCE_TRAVELED = actualDistance;
        _5_DistancePID.DISTANCE_ERROR = distanceError;
        _5_DistancePID.TIME_TO_TARGET = testDuration;
        
        // Calculate overshoot percentage
        double overshootPercent = 0.0;
        if (actualDistance > targetDistance) {
            overshootPercent = ((actualDistance - targetDistance) / targetDistance) * 100.0;
        }
        _5_DistancePID.MAX_OVERSHOOT_PERCENT = overshootPercent;
        
        // Calculate steady-state error (final error)
        _5_DistancePID.STEADY_STATE_ERROR = Math.abs(distanceError);
        
        // Calculate settling time (simplified - use total test time for now)
        _5_DistancePID.SETTLING_TIME_MS = testDuration;
        
        // Calculate response quality score (0-100)
        double qualityScore = 100.0;
        double errorPercent = Math.abs(distanceError) / targetDistance * 100.0;
        
        if (errorPercent < 2.0) {
            qualityScore = 100.0; // Excellent
        } else if (errorPercent < 5.0) {
            qualityScore = 85.0;  // Good
        } else if (errorPercent < 10.0) {
            qualityScore = 70.0;  // Fair
        } else {
            qualityScore = 50.0;  // Poor
        }
        
        // Reduce score for excessive overshoot
        if (overshootPercent > 10.0) {
            qualityScore -= 20.0;
        } else if (overshootPercent > 5.0) {
            qualityScore -= 10.0;
        }
        
        _5_DistancePID.RESPONSE_QUALITY = Math.max(0.0, qualityScore);
        
        // Calculate linear accuracy (how straight the path was)
        double expectedAngleRad = Math.toRadians(_5_DistancePID.TEST_ANGLE);
        double expectedX = targetDistance * Math.cos(expectedAngleRad);
        double expectedY = targetDistance * Math.sin(expectedAngleRad);
        
        double pathDeviation = Math.sqrt(Math.pow(deltaX - expectedX, 2) + Math.pow(deltaY - expectedY, 2));
        double linearAccuracy = Math.max(0.0, 100.0 - (pathDeviation / targetDistance * 100.0));
        _5_DistancePID.LINEAR_ACCURACY = linearAccuracy;
        
        // Store peak distance for overshoot analysis (simplified for now)
        _5_DistancePID.PEAK_DISTANCE_REACHED = actualDistance;
    }
    

    
    private void handleTestCommands() {
        // Emergency stop overrides all other commands
        if (_5_DistancePID.STOP_TEST) {
            if (motionExecutor != null) {
                motionExecutor.stop();
            }
            currentTest = TestState.IDLE;
            _5_DistancePID.STOP_TEST = false;
            return;
        }
        
        // Only process new commands if not currently testing
        if (currentTest == TestState.IDLE && _5_DistancePID.RUN_TEST) {
            startTestInternal();
            _5_DistancePID.RUN_TEST = false;
        }
    }
    
    @Override
    protected void updateTest() {
        // Handle test commands
        handleTestCommands();
        
        // Movement is handled by blocking moveToPose() call in executeAxisTest()
        // This method is called after movement completes, just for display/monitoring
        
        if (testCompleted) {
            // Test already complete, just display final results
            telemetry.addLine("✅ Test Complete!");
            telemetry.addLine("📊 Check odometry readings on dashboard");
            telemetry.addLine("📐 Measure actual distance with tape measure");
            
            // Still need to update config even when test is complete!
            if (dashboardManager != null) {
                dashboardManager.updateConfigIfNeeded();
            }
            return;
        }
        
        // Send graph data for real-time monitoring
        if (dashboardManager != null && motionExecutor != null) {
            motionExecutor.updateState();
            double currentX = motionExecutor.getMotionState().getX();
            double currentY = motionExecutor.getMotionState().getY();
            double currentDistance = Math.sqrt(currentX * currentX + currentY * currentY);
            double velocity = Math.sqrt(Math.pow(motionExecutor.getMotionState().getVelocityX(), 2) + Math.pow(motionExecutor.getMotionState().getVelocityY(), 2));
            
            dashboardManager.updateCalibrationData(currentX, currentY, currentDistance, velocity, 
                _5_DistancePID.SETTLING_TIME_MS, _5_DistancePID.MAX_OVERSHOOT_PERCENT);
        }
        
        // If we get here, movement is still in progress (blocked in moveToPose)
        // Display real-time odometry information during test execution
        telemetry.addLine("⏳ Movement in progress...");
        telemetry.addLine("(Robot is executing moveToPose)");
        telemetry.addLine("");
        
        // Show real-time odometry position during test execution
        telemetry.addLine("📍 REAL-TIME ODOMETRY POSITION:");
        if (odometryManager != null) {
            odometryManager.update();
            double currentX = odometryManager.getX();
            double currentY = odometryManager.getY();
            double currentHeading = odometryManager.getHeading();
            
            telemetry.addData("  X Position", String.format("%.3f inches", currentX));
            telemetry.addData("  Y Position", String.format("%.3f inches", currentY));
            telemetry.addData("  Heading", String.format("%.1f degrees", currentHeading));
            
            // Calculate current distance from origin
            double currentDistance = Math.sqrt(currentX * currentX + currentY * currentY);
            telemetry.addData("  Distance from Origin", String.format("%.3f inches", currentDistance));
            
            // Calculate current angle from origin
            double currentAngle = Math.toDegrees(Math.atan2(currentY, currentX));
            telemetry.addData("  Angle from Origin", String.format("%.1f degrees", currentAngle));
            
            // Show target information for comparison
            telemetry.addLine("");
            telemetry.addLine("🎯 TARGET POSITION:");
            double targetX = _5_DistancePID.TEST_DISTANCE * Math.cos(Math.toRadians(_5_DistancePID.TEST_ANGLE));
            double targetY = _5_DistancePID.TEST_DISTANCE * Math.sin(Math.toRadians(_5_DistancePID.TEST_ANGLE));
            telemetry.addData("  Target X", String.format("%.3f inches", targetX));
            telemetry.addData("  Target Y", String.format("%.3f inches", targetY));
            telemetry.addData("  Target Distance", String.format("%.3f inches", _5_DistancePID.TEST_DISTANCE));
            telemetry.addData("  Target Angle", String.format("%.1f degrees", _5_DistancePID.TEST_ANGLE));
            
            // Show error information
            telemetry.addLine("");
            telemetry.addLine("📏 CURRENT ERROR:");
            double errorX = targetX - currentX;
            double errorY = targetY - currentY;
            double errorDistance = Math.sqrt(errorX * errorX + errorY * errorY);
            telemetry.addData("  X Error", String.format("%.3f inches", errorX));
            telemetry.addData("  Y Error", String.format("%.3f inches", errorY));
            telemetry.addData("  Distance Error", String.format("%.3f inches", errorDistance));
        }
    }
    
    private void displayTuningGuidance(double error) {
        telemetry.addLine("");
        telemetry.addLine("🎛️ Distance PID PURE FEEDBACK Tuning:");
        
        // Get current distance PID values
        double kp = _5_DistancePID.DISTANCE_KP;
        double ki = _5_DistancePID.DISTANCE_KI;
        double kd = _5_DistancePID.DISTANCE_KD;
        
        // Analyze performance metrics
        double settlingTime = _5_DistancePID.SETTLING_TIME_MS;
        double overshoot = _5_DistancePID.MAX_OVERSHOOT_PERCENT;
        double steadyStateError = _5_DistancePID.STEADY_STATE_ERROR;
        double quality = _5_DistancePID.RESPONSE_QUALITY;
        
        telemetry.addLine("📊 Performance Analysis:");
        telemetry.addData("  Settling Time", String.format("%.0f ms", settlingTime));
        telemetry.addData("  Overshoot", String.format("%.1f%%", overshoot));
        telemetry.addData("  Steady Error", String.format("%.3f\"", steadyStateError));
        telemetry.addData("  Quality Score", String.format("%.0f/100", quality));
        
        telemetry.addLine("");
        telemetry.addLine("🔬 PID Component Analysis:");
        telemetry.addData("  P Gain", String.format("%.3f", kp));
        telemetry.addData("  I Gain", String.format("%.4f", ki));
        telemetry.addData("  D Gain", String.format("%.3f", kd));
        
        // Analyze gain relationships
        if (ki == 0) {
            telemetry.addLine("  ⚠️ No integral action - may have steady-state error");
        }
        if (kd == 0) {
            telemetry.addLine("  ⚠️ No derivative action - may overshoot");
        }
        if (kp < 0.05) {
            telemetry.addLine("  ⚠️ Low Kp - may be too slow for pure feedback");
        }
        if (kp > 0.5) {
            telemetry.addLine("  ⚠️ High Kp - may cause instability");
        }
        
        telemetry.addLine("");
        telemetry.addLine("🎯 Intelligent Tuning Guidance:");
        
        // Provide specific tuning recommendations based on performance
        if (overshoot > 15.0) {
            telemetry.addLine("  📉 HIGH OVERSHOOT - Reduce Kp by 20-30%");
            if (kd == 0) {
                telemetry.addLine("  📉 Add Kd (try " + String.format("%.3f", kp * 0.1) + ") for damping");
            } else {
                telemetry.addLine("  📉 Increase Kd by 50% for more damping");
            }
        } else if (settlingTime > 3000) {
            telemetry.addLine("  📈 SLOW RESPONSE - Increase Kp by 30-50%");
            telemetry.addLine("  📈 Current Kp=" + String.format("%.3f", kp) + " try " + String.format("%.3f", kp * 1.4));
        } else if (steadyStateError > 1.0) {
            telemetry.addLine("  📈 STEADY-STATE ERROR - Increase Ki");
            if (ki == 0) {
                telemetry.addLine("  📈 Add Ki (try " + String.format("%.4f", kp * 0.01) + ")");
            } else {
                telemetry.addLine("  📈 Increase Ki by 50-100%");
            }
        } else if (quality > 80) {
            telemetry.addLine("  ✅ EXCELLENT PERFORMANCE - Fine-tune for optimization");
            telemetry.addLine("  ✅ Consider small Kd increase for robustness");
        } else if (quality > 60) {
            telemetry.addLine("  🔧 GOOD PERFORMANCE - Minor adjustments needed");
        } else {
            telemetry.addLine("  ⚠️ POOR PERFORMANCE - Major tuning required");
            telemetry.addLine("  ⚠️ Try: Kp=" + String.format("%.3f", 0.15) + ", Ki=" + String.format("%.4f", 0.005) + ", Kd=" + String.format("%.3f", 0.02));
        }
        
        telemetry.addLine("");
        telemetry.addLine("💡 Pure Feedback Tips:");
        telemetry.addLine("  • Higher Kp needed (vs hybrid mode)");
        telemetry.addLine("  • Ki essential for steady-state accuracy");
        telemetry.addLine("  • Kd helps prevent overshoot/oscillation");
        telemetry.addLine("  • Typical ranges: Kp=0.1-0.3, Ki=0.002-0.02, Kd=0.01-0.05");
        
        telemetry.addLine("");
        telemetry.addLine("💾 Write down final values and update MotionConfig.java manually");
    }
    
    @Override
    protected void stopTest() {
        if (motionExecutor != null) {
            motionExecutor.stop();
            
            // CRITICAL: Restore original control mode
            if (controlModeChanged && originalControlMode != null) {
                motionExecutor.setControlMode(originalControlMode);
                controlModeChanged = false;
                
                telemetry.addLine("🔄 Control mode restored to: " + originalControlMode);
            }
        }
        
        // Reset test state
        currentTest = TestState.IDLE;
    }
    
    @Override
    public String getCalibrationName() {
        return "Axis PID Calibration";
    }
    
    @Override
    public String getCalibrationDescription() {
        return "Tune distance PID controller for precise linear movement in the 2-PID system";
    }
    
    @Override
    public void resetParameters() {
        // Reset distance PID values to defaults from MotionConfig
        _5_DistancePID.DISTANCE_KP = MotionConfig.DISTANCE_KP;
        _5_DistancePID.DISTANCE_KI = MotionConfig.DISTANCE_KI;
        _5_DistancePID.DISTANCE_KD = MotionConfig.DISTANCE_KD;
        
        // Reset test parameters
        _5_DistancePID.TEST_DISTANCE = 24.0;
        _5_DistancePID.TEST_ANGLE = 0.0;
        _5_DistancePID.MAX_VELOCITY = MotionConfig.MAX_LINEAR_VELOCITY;
        
        // Reset test commands
        _5_DistancePID.RUN_TEST = false;
        _5_DistancePID.STOP_TEST = false;
        
        // Reset distance performance metrics
        _5_DistancePID.SETTLING_TIME_MS = 0.0;
        _5_DistancePID.MAX_OVERSHOOT_PERCENT = 0.0;
        _5_DistancePID.STEADY_STATE_ERROR = 0.0;
        _5_DistancePID.RESPONSE_QUALITY = 0.0;
        _5_DistancePID.LINEAR_ACCURACY = 0.0;
        _5_DistancePID.ACTUAL_DISTANCE_TRAVELED = 0.0;
        _5_DistancePID.DISTANCE_ERROR = 0.0;
        _5_DistancePID.PEAK_DISTANCE_REACHED = 0.0;
        _5_DistancePID.TIME_TO_TARGET = 0.0;
        
        // Reset test state
        currentTest = TestState.IDLE;
    }
    
    @Override
    public void displayStatus() {
        telemetry.addLine("🎯 Distance-Based PID Testing (2-PID System)");
        telemetry.addLine(String.format("📏 Test Distance: %.1f inches", _5_DistancePID.TEST_DISTANCE));
        telemetry.addLine(String.format("🧭 Test Angle: %.1f° (0° = forward)", _5_DistancePID.TEST_ANGLE));
        telemetry.addLine(String.format("⚡ Max Velocity: %.1f in/s", _5_DistancePID.MAX_VELOCITY));
        telemetry.addLine("🎛️ Pure Feedback: ENABLED (AxisPID is pure feedback only)");
        telemetry.addLine("");
        
        telemetry.addLine("🎮 TEST CONTROLS:");
        telemetry.addData("RUN_TEST", "%s", _5_DistancePID.RUN_TEST ? "YES" : "NO");
        telemetry.addData("STOP_TEST", "%s", _5_DistancePID.STOP_TEST ? "YES" : "NO");
        telemetry.addData("Current State", "%s", currentTest.toString());
        telemetry.addLine("");
        
        telemetry.addLine("📊 DISTANCE PID PERFORMANCE RESULTS:");
        
        // Show target distance for reference
        if (_5_DistancePID.TEST_DISTANCE != 0) {
            telemetry.addData("🎯 Target Distance", String.format("%.3f inches", _5_DistancePID.TEST_DISTANCE));
            telemetry.addData("🧭 Test Angle", String.format("%.1f°", _5_DistancePID.TEST_ANGLE));
        }
        telemetry.addLine("");
        
        // Display distance-based results
        if (_5_DistancePID.ACTUAL_DISTANCE_TRAVELED != 0.0) {
            telemetry.addLine("📏 DISTANCE TEST RESULTS:");
            telemetry.addData("  Target Distance", String.format("%.3f inches", _5_DistancePID.TEST_DISTANCE));
            telemetry.addData("  Actual Distance", String.format("%.3f inches", _5_DistancePID.ACTUAL_DISTANCE_TRAVELED));
            telemetry.addData("  Distance Error", String.format("%+.3f inches", _5_DistancePID.DISTANCE_ERROR));
            
            String errorType = (_5_DistancePID.DISTANCE_ERROR > 0) ? "(overshoot ↗)" : "(undershoot ↘)";
            telemetry.addData("  Error Type", errorType);
            
            // Quality assessment
            double absError = Math.abs(_5_DistancePID.DISTANCE_ERROR);
            if (absError < 0.5) {
                telemetry.addData("  Quality", "🎯 EXCELLENT - Within 0.5\"");
            } else if (absError < 1.0) {
                telemetry.addData("  Quality", "✅ Good - Within 1.0\"");
            } else if (absError < 2.0) {
                telemetry.addData("  Quality", "⚠️ Fair - Needs tuning");
            } else {
                telemetry.addData("  Quality", "❌ Poor - Requires adjustment");
            }
            
            telemetry.addLine("");
            telemetry.addLine("⚡ PERFORMANCE METRICS:");
            telemetry.addData("  Response Quality", String.format("%.1f/100", _5_DistancePID.RESPONSE_QUALITY));
            telemetry.addData("  Linear Accuracy", String.format("%.1f/100", _5_DistancePID.LINEAR_ACCURACY));
            telemetry.addData("  Settling Time", String.format("%.0f ms", _5_DistancePID.SETTLING_TIME_MS));
            telemetry.addData("  Max Overshoot", String.format("%.1f%%", _5_DistancePID.MAX_OVERSHOOT_PERCENT));
            telemetry.addData("  Steady State Error", String.format("%.3f inches", _5_DistancePID.STEADY_STATE_ERROR));
            
        } else {
            telemetry.addLine("  (No test data yet - click RUN_TEST)");
        }
        
        // Add detailed odometry position information
        telemetry.addLine("");
        telemetry.addLine("📍 CURRENT ODOMETRY POSITION:");
        if (odometryManager != null) {
            odometryManager.update();
            double currentX = odometryManager.getX();
            double currentY = odometryManager.getY();
            double currentHeading = odometryManager.getHeading();
            
            telemetry.addData("  X Position", String.format("%.3f inches", currentX));
            telemetry.addData("  Y Position", String.format("%.3f inches", currentY));
            telemetry.addData("  Heading", String.format("%.1f degrees", currentHeading));
            
            // Calculate current distance from origin
            double currentDistance = Math.sqrt(currentX * currentX + currentY * currentY);
            telemetry.addData("  Distance from Origin", String.format("%.3f inches", currentDistance));
            
            // Calculate current angle from origin
            double currentAngle = Math.toDegrees(Math.atan2(currentY, currentX));
            telemetry.addData("  Angle from Origin", String.format("%.1f degrees", currentAngle));
            
            // Show detailed error breakdown if test has been run
            if (_5_DistancePID.ACTUAL_DISTANCE_TRAVELED != 0.0) {
                telemetry.addLine("");
                telemetry.addLine("🔍 DETAILED ERROR ANALYSIS:");
                
                // Calculate expected final position based on test parameters
                double expectedX = _5_DistancePID.TEST_DISTANCE * Math.cos(Math.toRadians(_5_DistancePID.TEST_ANGLE));
                double expectedY = _5_DistancePID.TEST_DISTANCE * Math.sin(Math.toRadians(_5_DistancePID.TEST_ANGLE));
                
                telemetry.addData("  Expected X", String.format("%.3f inches", expectedX));
                telemetry.addData("  Expected Y", String.format("%.3f inches", expectedY));
                telemetry.addData("  Actual X", String.format("%.3f inches", currentX));
                telemetry.addData("  Actual Y", String.format("%.3f inches", currentY));
                
                // Calculate individual axis errors
                double xError = currentX - expectedX;
                double yError = currentY - expectedY;
                
                telemetry.addData("  X Error", String.format("%+.3f inches", xError));
                telemetry.addData("  Y Error", String.format("%+.3f inches", yError));
                
                // Calculate path deviation (perpendicular distance from intended path)
                double pathDeviation = Math.abs(-Math.sin(Math.toRadians(_5_DistancePID.TEST_ANGLE)) * currentX + 
                                                Math.cos(Math.toRadians(_5_DistancePID.TEST_ANGLE)) * currentY);
                telemetry.addData("  Path Deviation", String.format("%.3f inches", pathDeviation));
                
                // Heading error
                double headingError = currentHeading - 0.0; // Assuming target heading is 0
                telemetry.addData("  Heading Error", String.format("%+.1f degrees", headingError));
            }
        } else {
            telemetry.addLine("  (Odometry not available)");
        }
        
        telemetry.addLine("");
        telemetry.addLine("📋 DISTANCE PID INSTRUCTIONS:");
        telemetry.addLine("1. Set TEST_DISTANCE (inches) and TEST_ANGLE (degrees)");
        telemetry.addLine("2. Adjust Distance PID gains (DISTANCE_KP, KI, KD)");
        telemetry.addLine("3. Pure feedback mode is automatically enabled");
        telemetry.addLine("4. Click RUN_TEST → Robot moves using linearMove()");
        telemetry.addLine("5. System calculates distance performance metrics:");
        telemetry.addLine("   • Distance error, overshoot, settling time");
        telemetry.addLine("   • Linear accuracy (path straightness)");
        telemetry.addLine("   • Response quality score (0-100)");
        telemetry.addLine("6. Use tuning guidance for optimization");
        telemetry.addLine("7. Test different angles to verify consistency");
        telemetry.addLine("8. Control mode automatically restored after test");
    }
}
