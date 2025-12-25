package org.firstinspires.ftc.teamcode.motion;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.external.gobilida.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.calibration.CalibrationCoefficients;
import org.firstinspires.ftc.teamcode.calibration.RobotConstants;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Centralized odometry management for GoBilda Pinpoint sensor.
 * 
 * Provides:
 * - Optimized sensor data handling
 * - Pose validation and filtering
 * - Sensor health monitoring
 * - Coordinate system management
 * - Performance optimization
 * - CORRECTED incremental calibration (applies scaling to deltas, not absolutes)
 * 
 * COORDINATE SYSTEM:
 * This class is configured to return REFERENCE POINT coordinates directly by calculating
 * the proper offset vector from the odometry sensor to the reference point. The offset
 * is computed as (sensor_position - reference_point_position) using robot center coordinates
 * from RobotConstants. This makes Pinpoint track around the reference point, eliminating
 * coordinate system mismatches throughout the motion control system.
 * 
 * CALIBRATION IMPLEMENTATION:
 * This class implements INCREMENTAL calibration scaling, which applies calibration
 * factors to the change in position (dx, dy, dθ) between updates, rather than to
 * the absolute position values. This prevents error accumulation and provides
 * mathematically correct calibration behavior.
 * 
 * PREVIOUS ISSUE: Applying calibration to absolute positions caused compounding
 * errors over time. FIXED: Now applies calibration to incremental changes only.
 * 
 * This ensures optimal usage of the Pinpoint odometry system and provides
 * reliable pose data for motion control algorithms.
 */
public class OdometryManager {
    
    // ========== HARDWARE REFERENCE ==========
    private final GoBildaPinpointDriver pinpoint;
    
    // ========== STATE TRACKING ==========
    private Pose2D currentPose;
    private Pose2D lastValidPose;
    private ElapsedTime updateTimer;
    private ElapsedTime healthTimer;
    
    // ========== INCREMENTAL CALIBRATION STATE ==========
    private double lastRawX;
    private double lastRawY;
    private double lastRawHeading;
    private boolean firstUpdate;
    
    // ========== VALIDATION PARAMETERS ==========
    private double maxVelocityThreshold;
    private double maxAccelerationThreshold;
    private double maxHeadingChangeRate;
    private int consecutiveInvalidReadings;
    private int maxInvalidReadings;
    
    // ========== PERFORMANCE TRACKING ==========
    private double updateFrequency;
    private long totalUpdates;
    private long validUpdates;
    private long invalidUpdates;
    
    // ========== HEALTH MONITORING ==========
    private boolean sensorHealthy;
    private String lastErrorMessage;
    private ElapsedTime lastValidUpdate;
    
    /**
     * Represents the health status of the odometry system
     */
    public static class OdometryHealth {
        public boolean isHealthy;
        public double updateRate;
        public double validReadingPercentage;
        public String statusMessage;
        public long totalUpdates;
        public long validUpdates;
        
        public OdometryHealth(boolean healthy, double rate, double percentage, 
                             String message, long total, long valid) {
            this.isHealthy = healthy;
            this.updateRate = rate;
            this.validReadingPercentage = percentage;
            this.statusMessage = message;
            this.totalUpdates = total;
            this.validUpdates = valid;
        }
        
        @Override
        public String toString() {
            return String.format("OdometryHealth[%s, %.1fHz, %.1f%% valid, %s]",
                isHealthy ? "HEALTHY" : "UNHEALTHY", updateRate, validReadingPercentage, statusMessage);
        }
    }
    
    /**
     * Creates a new odometry manager
     * 
     * @param pinpointDriver GoBilda Pinpoint odometry driver
     */
    public OdometryManager(GoBildaPinpointDriver pinpointDriver) {
        this.pinpoint = pinpointDriver;
        
        // Initialize state
        this.currentPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        this.lastValidPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        this.updateTimer = new ElapsedTime();
        this.healthTimer = new ElapsedTime();
        this.lastValidUpdate = new ElapsedTime();
        
        // Set validation thresholds
        this.maxVelocityThreshold = MotionConfig.MAX_LINEAR_VELOCITY * 2.0;  // 2x max expected
        this.maxAccelerationThreshold = MotionConfig.MAX_LINEAR_ACCELERATION * 3.0;  // 3x max expected
        this.maxHeadingChangeRate = MotionConfig.MAX_ANGULAR_VELOCITY * 2.0;  // 2x max expected
        this.maxInvalidReadings = 5;  // Allow 5 consecutive invalid readings
        
        // Initialize counters
        this.consecutiveInvalidReadings = 0;
        this.totalUpdates = 0;
        this.validUpdates = 0;
        this.invalidUpdates = 0;
        this.sensorHealthy = true;
        this.lastErrorMessage = "OK";
        
        // Initialize incremental calibration state
        this.lastRawX = 0.0;
        this.lastRawY = 0.0;
        this.lastRawHeading = 0.0;
        this.firstUpdate = true;
        
        // Configure Pinpoint with inch offsets
        configurePinpoint();
    }
    
    /**
     * Configures the Pinpoint sensor with proper settings
     */
    private void configurePinpoint() {
        try {
            // Calculate the offset vector from odometry sensor to reference point
            // This makes Pinpoint track around the reference point, returning reference point coordinates
            // Mathematical formula: offset = sensor_position - reference_point_position (both in robot center coords)
            double offsetX = RobotConstants.ODOMETRY_SENSOR.x - MotionConfig.ACTIVE_REFERENCE_POINT.x;
            double offsetY = RobotConstants.ODOMETRY_SENSOR.y - MotionConfig.ACTIVE_REFERENCE_POINT.y;
            pinpoint.setOffsets(offsetX, offsetY, DistanceUnit.INCH);
            
            // Set encoder resolution for GoBilda 4-bar pods
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            
            // Set encoder directions from RobotConstants
            pinpoint.setEncoderDirections(
                RobotConstants.ODOMETRY_X_ENCODER_DIRECTION,
                RobotConstants.ODOMETRY_Y_ENCODER_DIRECTION
            );
            
            // NOTE: Do NOT call setYawScalar() - the GoBilda Pinpoint comes pre-calibrated from factory
            // Each device has a per-device tuned yaw offset already applied
            // Only set a custom yaw scalar if you have specifically calibrated it and found the factory
            // calibration to be inaccurate (which is rare - GoBilda tests each unit before shipping)
            // Uncommenting the line below will OVERRIDE the factory calibration:
            // pinpoint.setYawScalar(CalibrationCoefficients.ODOMETRY_HEADING_SCALE);
            
            // CRITICAL: Reset position to 0,0,0 and calibrate IMU
            // This establishes the heading zero baseline and clears any accumulated position
            // Robot MUST be stationary during this ~250ms calibration period
            // This is called automatically during initialization - user should not need to manually reset
            pinpoint.resetPosAndIMU();
            
            // Wait for calibration to complete (Pinpoint documentation: takes ~250ms)
            // Check device status to ensure it's ready before proceeding
            ElapsedTime calibrationTimer = new ElapsedTime();
            while (calibrationTimer.milliseconds() < 300) {
                pinpoint.update();
                GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
                if (status == GoBildaPinpointDriver.DeviceStatus.READY) {
                    break; // Calibration complete
                }
                try {
                    Thread.sleep(10); // Small delay to avoid busy-waiting
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
            
            lastErrorMessage = "Pinpoint configured and calibrated successfully";
            
        } catch (Exception e) {
            sensorHealthy = false;
            lastErrorMessage = "Pinpoint configuration failed: " + e.getMessage();
        }
    }
    
    // ========== POSE MANAGEMENT ==========
    
    /**
     * Updates the current pose from the Pinpoint sensor
     * Includes validation and filtering for reliable data
     * 
     * @return true if update was successful and valid
     */
    public boolean update() {
        totalUpdates++;
        
        try {
            // === DEBUG: Odometry Update ===

            
            // Update Pinpoint sensor

            pinpoint.update();

            
            // Get new pose from Pinpoint
            // CRITICAL: Pinpoint returns position in MM, must convert to INCH
            Pose2D rawPose = pinpoint.getPosition();
            
            // Convert from MM (Pinpoint's native unit) to INCH (our system unit)
            double rawX = rawPose.getX(DistanceUnit.MM) / 25.4;  // MM to INCH
            double rawY = rawPose.getY(DistanceUnit.MM) / 25.4;  // MM to INCH
            double rawHeading = rawPose.getHeading(AngleUnit.DEGREES);
            
            Pose2D newPose;
            
            if (firstUpdate) {
                // First update: Initialize pose directly (no calibration needed for initial position)
                newPose = new Pose2D(DistanceUnit.INCH, rawX, rawY, AngleUnit.DEGREES, rawHeading);
                
                // Store raw values for next incremental update
                lastRawX = rawX;
                lastRawY = rawY;
                lastRawHeading = rawHeading;
                firstUpdate = false;
                
            } else {
                // Subsequent updates: Apply calibration to INCREMENTAL changes (CORRECT METHOD)
                
                // Calculate incremental changes since last update
                double deltaX = rawX - lastRawX;
                double deltaY = rawY - lastRawY;
                double deltaHeading = rawHeading - lastRawHeading;
                
                // Handle heading wraparound (e.g., 359° to 1° = +2°, not -358°)
                if (deltaHeading > 180.0) {
                    deltaHeading -= 360.0;
                } else if (deltaHeading < -180.0) {
                    deltaHeading += 360.0;
                }
                
                // Apply calibration scaling factors to INCREMENTAL changes
                // This corrects systematic measurement errors without compounding
                double calibratedDeltaX = deltaX * CalibrationCoefficients.ODOMETRY_X_SCALE;
                double calibratedDeltaY = deltaY * CalibrationCoefficients.ODOMETRY_Y_SCALE;
                // CRITICAL: DO NOT scale heading here! Pinpoint already applies ODOMETRY_HEADING_SCALE internally via setYawScalar()
                // Scaling again here would result in double-scaling and incorrect heading values
                double calibratedDeltaHeading = deltaHeading;  // No additional scaling needed
                
                // Update pose incrementally with calibrated deltas
                double newX = currentPose.getX(DistanceUnit.INCH) + calibratedDeltaX;
                double newY = currentPose.getY(DistanceUnit.INCH) + calibratedDeltaY;
                double newHeading = currentPose.getHeading(AngleUnit.DEGREES) + calibratedDeltaHeading;
                
                // Normalize heading to [-180, 180] range
                while (newHeading > 180.0) newHeading -= 360.0;
                while (newHeading < -180.0) newHeading += 360.0;
                
                // Create new pose with calibrated incremental updates
                newPose = new Pose2D(DistanceUnit.INCH, newX, newY, AngleUnit.DEGREES, newHeading);
                
                // Store current raw values for next incremental update
                lastRawX = rawX;
                lastRawY = rawY;
                lastRawHeading = rawHeading;
            }
            
            // Validate the new pose

            if (isValidPose(newPose)) {
                // Valid pose - update state

                lastValidPose = currentPose;
                currentPose = newPose;
                validUpdates++;
                consecutiveInvalidReadings = 0;
                lastValidUpdate.reset();
                
                // Update frequency calculation
                if (updateTimer.seconds() > 0) {
                    updateFrequency = 1.0 / updateTimer.seconds();
                }
                updateTimer.reset();
                

                return true;
                
            } else {
                // Invalid pose - increment counter
                System.out.println("[OdometryManager] [WARNING] Pose validation failed");
                invalidUpdates++;
                consecutiveInvalidReadings++;
                
                // Check if sensor is becoming unhealthy
                if (consecutiveInvalidReadings >= maxInvalidReadings) {
                    sensorHealthy = false;
                    lastErrorMessage = "Too many consecutive invalid readings";
                    System.out.println("[OdometryManager] [ERROR] Sensor marked as unhealthy");
                }
                
                return false;
            }
            
        } catch (Exception e) {
            invalidUpdates++;
            consecutiveInvalidReadings++;
            sensorHealthy = false;
            lastErrorMessage = "Sensor update failed: " + e.getMessage();
            return false;
        }
    }
    
    /**
     * Validates a pose reading for reasonableness
     * 
     * @param pose Pose to validate
     * @return true if pose appears valid
     */
    private boolean isValidPose(Pose2D pose) {
        if (pose == null) {
            return false;
        }
        
        // Check for NaN or infinite values
        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);
        double heading = pose.getHeading(AngleUnit.DEGREES);
        
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(heading)) {
            return false;
        }
        
        // Check for reasonable position bounds (within field size + margin)
        double maxFieldSize = 200.0;  // inches (generous margin for 12x12 foot field)
        if (Math.abs(x) > maxFieldSize || Math.abs(y) > maxFieldSize) {
            return false;
        }
        
        // Check velocity and acceleration limits (if we have previous data)
        if (totalUpdates > 0 && updateTimer.seconds() > 0) {
            double deltaTime = updateTimer.seconds();
            
            // Calculate velocity
            double deltaX = x - currentPose.getX(DistanceUnit.INCH);
            double deltaY = y - currentPose.getY(DistanceUnit.INCH);
            double velocity = Math.hypot(deltaX, deltaY) / deltaTime;
            
            if (velocity > maxVelocityThreshold) {
                return false;
            }
            
            // Calculate heading change rate
            double deltaHeading = heading - currentPose.getHeading(AngleUnit.DEGREES);
            // Normalize to [-180, 180] range
            while (deltaHeading > 180) deltaHeading -= 360;
            while (deltaHeading <= -180) deltaHeading += 360;
            double headingRate = Math.abs(deltaHeading) / deltaTime;
            
            if (headingRate > maxHeadingChangeRate) {
                return false;
            }
        }
        
        return true;
    }
    
    /**
     * Gets the current robot pose
     * 
     * @return Current pose (validated and filtered)
     */
    public Pose2D getCurrentPose() {
        return currentPose;
    }
    
    /**
     * Gets the last known valid pose (fallback for invalid readings)
     * 
     * @return Last valid pose
     */
    public Pose2D getLastValidPose() {
        return lastValidPose;
    }
    
    /**
     * Gets the current X position in inches
     * 
     * @return Current X position
     */
    public double getX() {
        return currentPose.getX(DistanceUnit.INCH);
    }
    
    /**
     * Gets the current Y position in inches
     * 
     * @return Current Y position
     */
    public double getY() {
        return currentPose.getY(DistanceUnit.INCH);
    }
    
    /**
     * Gets the current heading in degrees
     * 
     * @return Current heading
     */
    public double getHeading() {
        return currentPose.getHeading(AngleUnit.DEGREES);
    }
    
    // ========== COORDINATE SYSTEM MANAGEMENT ==========
    
    /**
     * Resets the odometry to field origin position
     * Sets the robot's current position to the configured field origin
     */
    public void resetToFieldOrigin() {
        try {
            // Reset hardware odometry - this resets Pinpoint to (0,0,0) and recalibrates IMU
            // Takes approximately 0.25 seconds for IMU calibration
            pinpoint.resetPosAndIMU();
            
            // Wait for IMU recalibration to complete -- very important!
            try {
                Thread.sleep(300);  // 300ms to ensure IMU calibration finishes
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            

            // Update Pinpoint to get current position
            pinpoint.update();
            Pose2D actualPose = pinpoint.getPosition();
            
            // Set our logical position to field origin
            Pose2D fieldOrigin = new Pose2D(
                DistanceUnit.INCH, 
                MotionConfig.FIELD_ORIGIN_X, 
                MotionConfig.FIELD_ORIGIN_Y,
                AngleUnit.DEGREES, 
                MotionConfig.FIELD_ORIGIN_HEADING
            );
            
            currentPose = fieldOrigin;
            lastValidPose = fieldOrigin;
            
            // CRITICAL: Initialize incremental tracking with the ACTUAL raw values from Pinpoint
            // This ensures the next incremental update calculates deltas correctly
            lastRawX = actualPose.getX(DistanceUnit.MM) / 25.4;  // Convert MM to INCH
            lastRawY = actualPose.getY(DistanceUnit.MM) / 25.4;  // Convert MM to INCH  
            lastRawHeading = actualPose.getHeading(AngleUnit.DEGREES);
            firstUpdate = false;  // We have a valid starting position
            
            // Reset health tracking
            consecutiveInvalidReadings = 0;
            sensorHealthy = true;
            lastErrorMessage = "Reset to field origin";
            lastValidUpdate.reset();
            
        } catch (Exception e) {
            sensorHealthy = false;
            lastErrorMessage = "Reset failed: " + e.getMessage();
        }
    }
    
    /**
     * Resets odometry to a specific pose
     * 
     * @param pose Target pose to reset to
     */
    public void resetToPose(Pose2D pose) {
        try {
            // Reset hardware - this resets pose and recalibrates IMU (~0.25s)
            pinpoint.resetPosAndIMU();
            
            // Wait for IMU recalibration
            try {
                Thread.sleep(300);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            
            pinpoint.setPosition(pose);
            
            // Update and read actual position from Pinpoint
            pinpoint.update();
            Pose2D actualPose = pinpoint.getPosition();
            
            currentPose = pose;
            lastValidPose = pose;
            
            // Initialize incremental tracking with ACTUAL raw values from Pinpoint
            lastRawX = actualPose.getX(DistanceUnit.MM) / 25.4;
            lastRawY = actualPose.getY(DistanceUnit.MM) / 25.4;
            lastRawHeading = actualPose.getHeading(AngleUnit.DEGREES);
            firstUpdate = false;  // We have a valid starting position
            
            consecutiveInvalidReadings = 0;
            sensorHealthy = true;
            lastErrorMessage = "Reset to custom pose";
            lastValidUpdate.reset();
            
        } catch (Exception e) {
            sensorHealthy = false;
            lastErrorMessage = "Custom reset failed: " + e.getMessage();
        }
    }
    
    /**
     * Resets only position, keeping current heading
     * 
     * @param x New X position (inches)
     * @param y New Y position (inches)
     */
    public void resetPosition(double x, double y) {
        double currentHeading = currentPose.getHeading(AngleUnit.DEGREES);
        Pose2D newPose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, currentHeading);
        resetToPose(newPose);
    }
    
    /**
     * Resets only heading, keeping current position
     * 
     * @param heading New heading (degrees)
     */
    public void resetHeading(double heading) {
        double currentX = currentPose.getX(DistanceUnit.INCH);
        double currentY = currentPose.getY(DistanceUnit.INCH);
        Pose2D newPose = new Pose2D(DistanceUnit.INCH, currentX, currentY, AngleUnit.DEGREES, heading);
        resetToPose(newPose);
    }
    
    // ========== HEALTH MONITORING ==========
    
    /**
     * Gets the current health status of the odometry system
     * 
     * @return Health status information
     */
    public OdometryHealth getHealth() {
        double validPercentage = (totalUpdates > 0) ? (100.0 * validUpdates / totalUpdates) : 100.0;
        
        // Determine overall health
        boolean healthy = sensorHealthy && 
                         (validPercentage > 80.0) && 
                         (lastValidUpdate.seconds() < 1.0);  // Recent valid update
        
        String statusMessage = lastErrorMessage;
        if (!healthy && lastValidUpdate.seconds() > 1.0) {
            statusMessage = "No valid updates for " + String.format("%.1f", lastValidUpdate.seconds()) + "s";
        }
        
        return new OdometryHealth(healthy, updateFrequency, validPercentage, 
                                 statusMessage, totalUpdates, validUpdates);
    }
    
    /**
     * Checks if the odometry system is healthy
     * 
     * @return true if system is operating normally
     */
    public boolean isHealthy() {
        return getHealth().isHealthy;
    }
    
    /**
     * Gets diagnostic information for troubleshooting
     * 
     * @return Diagnostic string
     */
    public String getDiagnostics() {
        OdometryHealth health = getHealth();
        return String.format("Odometry Diagnostics:\n" +
                           "  Health: %s\n" +
                           "  Update Rate: %.1f Hz\n" +
                           "  Valid Readings: %d/%d (%.1f%%)\n" +
                           "  Consecutive Invalid: %d\n" +
                           "  Last Valid Update: %.1fs ago\n" +
                           "  Status: %s\n" +
                           "  Current Pose: (%.2f, %.2f, %.1f degrees)\n" +
                           "  Calibration Mode: Incremental (CORRECTED)\n" +
                           "  Last Raw Position: (%.2f, %.2f, %.1f degrees)\n" +
                           "  Calibration Factors: X=%.4f, Y=%.4f, H=%.4f",
                           health.isHealthy ? "HEALTHY" : "UNHEALTHY",
                           health.updateRate,
                           validUpdates, totalUpdates, health.validReadingPercentage,
                           consecutiveInvalidReadings,
                           lastValidUpdate.seconds(),
                           health.statusMessage,
                           currentPose.getX(DistanceUnit.INCH),
                           currentPose.getY(DistanceUnit.INCH),
                           currentPose.getHeading(AngleUnit.DEGREES),
                           lastRawX, lastRawY, lastRawHeading,
                           CalibrationCoefficients.ODOMETRY_X_SCALE,
                           CalibrationCoefficients.ODOMETRY_Y_SCALE,
                           CalibrationCoefficients.ODOMETRY_HEADING_SCALE);
    }
    
    // ========== CONFIGURATION ==========
    
    /**
     * Sets validation thresholds for pose filtering
     * 
     * @param maxVelocity Maximum reasonable velocity (inches/sec)
     * @param maxAcceleration Maximum reasonable acceleration (inches/sec^2)
     * @param maxHeadingRate Maximum heading change rate (degrees/sec)
     */
    public void setValidationThresholds(double maxVelocity, double maxAcceleration, double maxHeadingRate) {
        this.maxVelocityThreshold = Math.abs(maxVelocity);
        this.maxAccelerationThreshold = Math.abs(maxAcceleration);
        this.maxHeadingChangeRate = Math.abs(maxHeadingRate);
    }
    
    /**
     * Sets the maximum number of consecutive invalid readings before marking sensor unhealthy
     * 
     * @param maxInvalid Maximum consecutive invalid readings
     */
    public void setMaxInvalidReadings(int maxInvalid) {
        this.maxInvalidReadings = Math.max(1, maxInvalid);
    }
    
    // ========== UTILITY METHODS ==========
    
    /**
     * Gets the underlying Pinpoint driver (for advanced usage)
     * 
     * @return GoBilda Pinpoint driver
     */
    public GoBildaPinpointDriver getPinpointDriver() {
        return pinpoint;
    }
    
    /**
     * Forces a sensor health reset (clears error conditions)
     */
    public void resetHealth() {
        sensorHealthy = true;
        consecutiveInvalidReadings = 0;
        lastErrorMessage = "Health reset";
        lastValidUpdate.reset();
    }
    
    /**
     * Gets performance statistics
     * 
     * @return Performance summary string
     */
    public String getPerformanceStats() {
        return String.format("Odometry Performance: %.1f Hz, %d valid/%d total (%.1f%%)",
                           updateFrequency, validUpdates, totalUpdates,
                           (totalUpdates > 0) ? (100.0 * validUpdates / totalUpdates) : 100.0);
    }
    
    @Override
    public String toString() {
        return String.format("OdometryManager[pose=(%.2f, %.2f, %.1f degrees), health=%s, rate=%.1fHz]",
                           currentPose.getX(DistanceUnit.INCH),
                           currentPose.getY(DistanceUnit.INCH),
                           currentPose.getHeading(AngleUnit.DEGREES),
                           isHealthy() ? "OK" : "ERROR",
                           updateFrequency);
    }
}
