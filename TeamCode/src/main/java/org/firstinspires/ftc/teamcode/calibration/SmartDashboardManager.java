package org.firstinspires.ftc.teamcode.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Smart Dashboard Manager for 3-Tier Configurable Telemetry System
 * 
 * TIER 1: Fast Internal Updates (10Hz) - Critical for calibration accuracy
 * TIER 2: Configurable Graph Data (5Hz default) - TelemetryPacket for graphs
 * TIER 3: Configurable Config Updates (1Hz default) - Prevent refresh spam
 * 
 * Features:
 * - Real-time configurable update rates via FTC Dashboard
 * - Emergency disable for graph data
 * - Optimized config update batching
 * - Clean separation of text vs graph telemetry
 */
@Config
public class SmartDashboardManager {
    
    // ========== CONFIGURABLE TIMING (Hz for easy tuning) ==========
    
    /**
     * Graph data transmission rate (Hz)
     * Controls how often TelemetryPacket data is sent for graphing
     * Default: 5Hz (good balance of responsiveness and performance)
     */
    public static double GRAPH_UPDATE_HZ = 5.0;
    
    /**
     * Text telemetry update rate (Hz)
     * Controls how often telemetry.update() is called
     * Default: 2Hz (prevents dashboard page refresh issues)
     */
    public static double TEXT_UPDATE_HZ = 2.0;
    
    /**
     * Configuration update rate (Hz)
     * Controls how often updateConfig() is called
     * Default: 1Hz (prevents excessive config refresh)
     */
    public static double CONFIG_UPDATE_HZ = 1.0;
    
    // ========== SIMPLE CONTROLS ==========
    
    /**
     * Emergency disable for graph data transmission
     * Set to false if dashboard becomes unstable
     */
    public static boolean ENABLE_GRAPH_DATA = true;
    
    // ========== INTERNAL STATE ==========
    
    private long lastConfigUpdate = 0;
    private long lastGraphUpdate = 0;
    private long lastTextUpdate = 0;
    private boolean configChanged = false;
    private ElapsedTime timer = new ElapsedTime();
    
    /**
     * Constructor - initializes timer
     */
    public SmartDashboardManager() {
        timer.reset();
    }
    
    /**
     * Update calibration data for graphing at configurable rate
     * 
     * @param posX Current X position
     * @param posY Current Y position  
     * @param error Current position error
     * @param velocity Current velocity command
     */
    public void updateCalibrationData(double posX, double posY, double error, double velocity) {
        long now = (long)(timer.milliseconds());
        
        // TIER 2: Graph data at configurable rate
        if (ENABLE_GRAPH_DATA && now - lastGraphUpdate > (1000.0 / GRAPH_UPDATE_HZ)) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("position_x", posX);
            packet.put("position_y", posY);
            packet.put("position_error", error);
            packet.put("velocity_command", velocity);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            lastGraphUpdate = now;
        }
        
        // TIER 3: Config updates at configurable rate (only when needed)
        if (configChanged && now - lastConfigUpdate > (1000.0 / CONFIG_UPDATE_HZ)) {
            FtcDashboard.getInstance().updateConfig();
            configChanged = false;
            lastConfigUpdate = now;
        }
    }
    
    /**
     * Enhanced version with additional calibration parameters
     * 
     * @param posX Current X position
     * @param posY Current Y position
     * @param error Current position error
     * @param velocity Current velocity command
     * @param settlingTime Current settling time (ms)
     * @param overshoot Current overshoot percentage
     */
    public void updateCalibrationData(double posX, double posY, double error, double velocity, 
                                    double settlingTime, double overshoot) {
        long now = (long)(timer.milliseconds());
        
        // TIER 2: Graph data at configurable rate
        if (ENABLE_GRAPH_DATA && now - lastGraphUpdate > (1000.0 / GRAPH_UPDATE_HZ)) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("position_x", posX);
            packet.put("position_y", posY);
            packet.put("position_error", error);
            packet.put("velocity_command", velocity);
            packet.put("settling_time_ms", settlingTime);
            packet.put("overshoot_percent", overshoot);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            lastGraphUpdate = now;
        }
        
        // TIER 3: Config updates at configurable rate (only when needed)
        if (configChanged && now - lastConfigUpdate > (1000.0 / CONFIG_UPDATE_HZ)) {
            FtcDashboard.getInstance().updateConfig();
            configChanged = false;
            lastConfigUpdate = now;
        }
    }
    
    /**
     * Motor calibration specific data update
     * 
     * @param targetVelocity Target motor velocity
     * @param actualVelocity Actual motor velocity
     * @param motorPower Motor power output
     * @param error Velocity error
     */
    public void updateMotorCalibrationData(double targetVelocity, double actualVelocity, 
                                         double motorPower, double error) {
        long now = (long)(timer.milliseconds());
        
        // TIER 2: Graph data at configurable rate
        if (ENABLE_GRAPH_DATA && now - lastGraphUpdate > (1000.0 / GRAPH_UPDATE_HZ)) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("target_velocity", targetVelocity);
            packet.put("actual_velocity", actualVelocity);
            packet.put("motor_power", motorPower);
            packet.put("velocity_error", error);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            lastGraphUpdate = now;
        }
        
        // TIER 3: Config updates at configurable rate (only when needed)
        if (configChanged && now - lastConfigUpdate > (1000.0 / CONFIG_UPDATE_HZ)) {
            FtcDashboard.getInstance().updateConfig();
            configChanged = false;
            lastConfigUpdate = now;
        }
    }
    
    /**
     * Odometry calibration specific data update
     * 
     * @param encoderX X encoder reading
     * @param encoderY Y encoder reading
     * @param encoderH Heading encoder reading
     * @param calculatedX Calculated X position
     * @param calculatedY Calculated Y position
     */
    public void updateOdometryCalibrationData(double encoderX, double encoderY, double encoderH,
                                            double calculatedX, double calculatedY) {
        long now = (long)(timer.milliseconds());
        
        // TIER 2: Graph data at configurable rate
        if (ENABLE_GRAPH_DATA && now - lastGraphUpdate > (1000.0 / GRAPH_UPDATE_HZ)) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("encoder_x", encoderX);
            packet.put("encoder_y", encoderY);
            packet.put("encoder_heading", encoderH);
            packet.put("calculated_x", calculatedX);
            packet.put("calculated_y", calculatedY);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            lastGraphUpdate = now;
        }
        
        // TIER 3: Config updates at configurable rate (only when needed)
        if (configChanged && now - lastConfigUpdate > (1000.0 / CONFIG_UPDATE_HZ)) {
            FtcDashboard.getInstance().updateConfig();
            configChanged = false;
            lastConfigUpdate = now;
        }
    }
    
    /**
     * Check if text telemetry should be updated based on configurable rate
     * 
     * @return true if telemetry.update() should be called
     */
    public boolean shouldUpdateTextTelemetry() {
        long now = (long)(timer.milliseconds());
        if (now - lastTextUpdate > (1000.0 / TEXT_UPDATE_HZ)) {
            lastTextUpdate = now;
            return true;
        }
        return false;
    }
    
    /**
     * Mark that configuration has changed and needs to be updated
     * Call this when PID values or other config parameters change
     */
    public void markConfigChanged() {
        configChanged = true;
    }
    
    /**
     * Force immediate config update (use sparingly)
     */
    public void forceConfigUpdate() {
        FtcDashboard.getInstance().updateConfig();
        configChanged = false;
        lastConfigUpdate = (long)(timer.milliseconds());
    }
    
    /**
     * Update config if it has been marked as changed and enough time has passed
     * This should be called regularly by modules that don't use updateCalibrationData()
     * 
     * Typical usage: Call this in your module's updateTest() method
     */
    public void updateConfigIfNeeded() {
        long now = (long)(timer.milliseconds());
        if (configChanged && now - lastConfigUpdate > (1000.0 / CONFIG_UPDATE_HZ)) {
            FtcDashboard.getInstance().updateConfig();
            configChanged = false;
            lastConfigUpdate = now;
        }
    }
    
    /**
     * Reset all timers (useful when starting new calibration)
     */
    public void resetTimers() {
        timer.reset();
        lastConfigUpdate = 0;
        lastGraphUpdate = 0;
        lastTextUpdate = 0;
        configChanged = false;
    }
}
