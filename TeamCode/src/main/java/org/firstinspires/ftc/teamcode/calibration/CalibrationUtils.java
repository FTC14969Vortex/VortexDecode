package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;

/**
 * Utility class for FTC Dashboard-based calibration OpModes.
 * 
 * Provides common functionality for:
 * - Dashboard telemetry formatting
 * - Performance metric calculations
 * - Test pattern generation
 * - Configuration file updates
 * - Data collection and analysis
 * 
 * This eliminates code duplication across calibration OpModes while
 * maintaining their independence.
 */
public class CalibrationUtils {
    
    // ========== TELEMETRY FORMATTING ==========
    
    /**
     * Format a status message with color coding for Dashboard
     */
    public static String formatStatus(String message, StatusType type) {
        switch (type) {
            case SUCCESS:
                return "✅ " + message;
            case WARNING:
                return "⚠️ " + message;
            case ERROR:
                return "❌ " + message;
            case INFO:
                return "ℹ️ " + message;
            case PROGRESS:
                return "🔄 " + message;
            default:
                return message;
        }
    }
    
    public enum StatusType {
        SUCCESS, WARNING, ERROR, INFO, PROGRESS
    }
    
    /**
     * Display a formatted header section in telemetry
     */
    public static void addTelemetryHeader(Telemetry telemetry, String title) {
        telemetry.addLine("");
        telemetry.addLine("═══════════════════════════════════");
        telemetry.addLine("  " + title);
        telemetry.addLine("═══════════════════════════════════");
    }
    
    /**
     * Display parameter with current and target values
     */
    public static void addParameterStatus(Telemetry telemetry, String name, double current, double target, String unit) {
        String status = Math.abs(current - target) < 0.01 ? "✅" : "🔄";
        telemetry.addData(status + " " + name, "%.3f %s (target: %.3f)", current, unit, target);
    }
    
    /**
     * Display a progress bar in telemetry
     */
    public static void addProgressBar(Telemetry telemetry, String label, double progress, double max) {
        int barLength = 20;
        int filled = (int) ((progress / max) * barLength);
        StringBuilder bar = new StringBuilder("[");
        
        for (int i = 0; i < barLength; i++) {
            if (i < filled) {
                bar.append("█");
            } else {
                bar.append("░");
            }
        }
        bar.append("]");
        
        telemetry.addData(label, "%s %.1f%%", bar.toString(), (progress / max) * 100);
    }
    
    // ========== PERFORMANCE METRICS ==========
    
    /**
     * Calculate rise time (time to reach 90% of target)
     */
    public static double calculateRiseTime(List<Double> values, List<Double> times, double target) {
        double threshold = target * 0.9;
        
        for (int i = 0; i < values.size(); i++) {
            if (Math.abs(values.get(i)) >= Math.abs(threshold)) {
                return times.get(i);
            }
        }
        return -1; // Not reached
    }
    
    /**
     * Calculate settling time (time to stay within 5% of target)
     */
    public static double calculateSettlingTime(List<Double> values, List<Double> times, double target) {
        double tolerance = Math.abs(target) * 0.05;
        int settledCount = 0;
        int requiredCount = 10; // Must stay settled for 10 samples
        
        for (int i = values.size() - 1; i >= 0; i--) {
            if (Math.abs(values.get(i) - target) <= tolerance) {
                settledCount++;
                if (settledCount >= requiredCount) {
                    return times.get(i);
                }
            } else {
                settledCount = 0;
            }
        }
        return -1; // Not settled
    }
    
    /**
     * Calculate maximum overshoot percentage
     */
    public static double calculateOvershoot(List<Double> values, double target) {
        double maxValue = values.stream().mapToDouble(Double::doubleValue).max().orElse(0);
        if (target == 0) return 0;
        return Math.max(0, (maxValue - target) / Math.abs(target) * 100);
    }
    
    /**
     * Calculate steady-state error
     */
    public static double calculateSteadyStateError(List<Double> values, double target) {
        if (values.size() < 10) return Double.MAX_VALUE;
        
        // Average last 10 values
        double sum = 0;
        for (int i = values.size() - 10; i < values.size(); i++) {
            sum += values.get(i);
        }
        double steadyState = sum / 10.0;
        
        return Math.abs(steadyState - target);
    }
    
    // ========== TEST PATTERN GENERATORS ==========
    
    /**
     * Generate step response test pattern
     */
    public static class StepResponse {
        public final double amplitude;
        public final double duration;
        
        public StepResponse(double amplitude, double duration) {
            this.amplitude = amplitude;
            this.duration = duration;
        }
        
        public double getValue(double time) {
            return time >= 0 ? amplitude : 0;
        }
    }
    
    /**
     * Generate ramp response test pattern
     */
    public static class RampResponse {
        public final double finalValue;
        public final double rampTime;
        
        public RampResponse(double finalValue, double rampTime) {
            this.finalValue = finalValue;
            this.rampTime = rampTime;
        }
        
        public double getValue(double time) {
            if (time <= 0) return 0;
            if (time >= rampTime) return finalValue;
            return (finalValue / rampTime) * time;
        }
    }
    
    /**
     * Generate square wave test pattern
     */
    public static class SquareWave {
        public final double amplitude;
        public final double period;
        
        public SquareWave(double amplitude, double period) {
            this.amplitude = amplitude;
            this.period = period;
        }
        
        public double getValue(double time) {
            double phase = (time % period) / period;
            return phase < 0.5 ? amplitude : -amplitude;
        }
    }
    
    // ========== DATA COLLECTION ==========
    
    /**
     * Time-series data collector for Dashboard graphs
     */
    public static class DataCollector {
        private List<Double> times = new ArrayList<>();
        private List<Double> values = new ArrayList<>();
        private ElapsedTime timer = new ElapsedTime();
        private int maxSamples;
        
        public DataCollector(int maxSamples) {
            this.maxSamples = maxSamples;
            timer.reset();
        }
        
        public void addSample(double value) {
            times.add(timer.seconds());
            values.add(value);
            
            // Keep only recent samples
            if (times.size() > maxSamples) {
                times.remove(0);
                values.remove(0);
            }
        }
        
        public List<Double> getTimes() { return new ArrayList<>(times); }
        public List<Double> getValues() { return new ArrayList<>(values); }
        
        public void reset() {
            times.clear();
            values.clear();
            timer.reset();
        }
        
        public double getLatestValue() {
            return values.isEmpty() ? 0 : values.get(values.size() - 1);
        }
        
        public double getLatestTime() {
            return times.isEmpty() ? 0 : times.get(times.size() - 1);
        }
    }
    
    // ========== VALIDATION UTILITIES ==========
    
    /**
     * Validate parameter is within reasonable range
     */
    public static boolean validateRange(double value, double min, double max, String paramName) {
        if (value < min || value > max) {
            return false;
        }
        return true;
    }
    
    /**
     * Validate PID gains are reasonable
     */
    public static String validatePIDGains(double kp, double ki, double kd) {
        List<String> issues = new ArrayList<>();
        
        if (kp < 0) issues.add("Kp should be positive");
        if (kp > 1.0) issues.add("Kp seems very high (>1.0)");
        if (ki < 0) issues.add("Ki should be positive");
        if (ki > 0.1) issues.add("Ki seems high (>0.1) - may cause instability");
        if (kd < 0) issues.add("Kd should be positive");
        if (kd > 0.01) issues.add("Kd seems high (>0.01) - may amplify noise");
        
        return issues.isEmpty() ? "✅ PID gains look reasonable" : "⚠️ " + String.join(", ", issues);
    }
    
    /**
     * Safety check for motor power values
     */
    public static double clampMotorPower(double power) {
        return Math.max(-1.0, Math.min(1.0, power));
    }
    
    // ========== DASHBOARD HELPERS ==========
    
    /**
     * Format value for Dashboard display with appropriate precision
     */
    public static String formatValue(double value, String unit) {
        if (Math.abs(value) < 0.001) {
            return String.format("%.6f %s", value, unit);
        } else if (Math.abs(value) < 0.1) {
            return String.format("%.4f %s", value, unit);
        } else if (Math.abs(value) < 10) {
            return String.format("%.3f %s", value, unit);
        } else {
            return String.format("%.1f %s", value, unit);
        }
    }
    
    /**
     * Create a simple instruction display
     */
    public static void displayInstructions(Telemetry telemetry, String[] instructions) {
        addTelemetryHeader(telemetry, "INSTRUCTIONS");
        for (int i = 0; i < instructions.length; i++) {
            telemetry.addLine((i + 1) + ". " + instructions[i]);
        }
        telemetry.addLine("");
    }
    
    /**
     * Display calibration controls
     */
    public static void displayControls(Telemetry telemetry) {
        addTelemetryHeader(telemetry, "CONTROLS");
        telemetry.addLine("🎮 Gamepad Controls:");
        telemetry.addLine("  A - Start/Stop Test");
        telemetry.addLine("  B - Reset Data");
        telemetry.addLine("  X - Save Parameters");
        telemetry.addLine("  Y - Next Test Pattern");
        telemetry.addLine("");
        telemetry.addLine("📊 Use FTC Dashboard to adjust parameters");
        telemetry.addLine("   Connect to: http://192.168.43.1:8080/dash");
        telemetry.addLine("");
    }
    
    // ========== SAFE OPERATIONS ==========
    
    /**
     * Safe sleep that respects OpMode active state
     */
    public static void safeSleep(LinearOpMode opMode, long milliseconds) {
        long endTime = System.currentTimeMillis() + milliseconds;
        while (opMode.opModeIsActive() && System.currentTimeMillis() < endTime) {
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }
    }
    
    /**
     * Check if gamepad button was just pressed (edge detection)
     */
    public static boolean buttonJustPressed(boolean currentState, boolean lastState) {
        return currentState && !lastState;
    }
    
    // ========== CONSTANTS ==========
    
    public static final double INCHES_TO_MM = 25.4;
    public static final double MM_TO_INCHES = 1.0 / 25.4;
    public static final double DEG_TO_RAD = Math.PI / 180.0;
    public static final double RAD_TO_DEG = 180.0 / Math.PI;
    
    // Reasonable parameter ranges for validation
    public static final double MIN_TRACK_WIDTH = 8.0;  // inches
    public static final double MAX_TRACK_WIDTH = 24.0; // inches
    public static final double MIN_WHEELBASE = 8.0;    // inches
    public static final double MAX_WHEELBASE = 24.0;   // inches
    public static final double MIN_WHEEL_DIAMETER = 2.0; // inches
    public static final double MAX_WHEEL_DIAMETER = 6.0; // inches
    
    // PID tuning recommendations
    public static final String[] PID_TUNING_TIPS = {
        "Start with Kp only, set Ki=0, Kd=0",
        "Increase Kp until oscillation, then reduce by 50%",
        "Add small Ki (0.001-0.01) to eliminate steady-state error",
        "Add small Kd (0.0001-0.001) only if needed for stability",
        "Higher gains = faster response but less stability",
        "Lower gains = slower response but more stable"
    };
}
