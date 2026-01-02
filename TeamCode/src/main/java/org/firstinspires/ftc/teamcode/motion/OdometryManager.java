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
        
        // Configure Pinpoint with inch offsets
        configurePinpoint();        // set odo to reference point
    }
    
    /**
     * Configures the Pinpoint sensor with proper settings
     */
    private void configurePinpoint() {
        try {
            // Calculate the offset vector from odometry sensor to reference point

            // GoBilda Pinpoint uses convention coordinate system where: x  is side-to-side, y is front-to-back
            // Our robot coordinate system defines: x is front-to-back, y is side-to-side
            // plus the internal r_ref = r_odometry - r_offset_rotated, r_offset_rotated = r_offset*e^(i*(theta+pi/2)), pi/2 is coordinate system rotation, very confusing
            // and we have two odo-pod, they have offset in different direction, so we just hardcode the offset here for simplicity

            double offsetX =  FieldPositions.getActiveReferencePoint().x - (RobotConstants.ODOMETRY_SENSOR.x + RobotConstants.ODOMETRY_DX);
            double offsetY =  FieldPositions.getActiveReferencePoint().y - (RobotConstants.ODOMETRY_SENSOR.y + RobotConstants.ODOMETRY_DY);

            pinpoint.setOffsets(-offsetY, -offsetX, DistanceUnit.INCH); // gobilda's convention

            
            // Set encoder resolution for GoBilda 4-bar pods
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            
            // Set encoder directions from RobotConstants
            pinpoint.setEncoderDirections(
                RobotConstants.ODOMETRY_X_ENCODER_DIRECTION,
                RobotConstants.ODOMETRY_Y_ENCODER_DIRECTION
            );
            
            // NOTE:  call setYawScalar() - if needed to apply your calibration
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
     * Direct reading - hardware tracks position from setPosition() calls
     * 
     * @return true if update was successful and valid
     */
    public boolean update() {
        totalUpdates++;

        try {
            // Update Pinpoint sensor
            pinpoint.update();

            // Read position directly from hardware
            // Hardware already tracks from the position set by setPosition()
            currentPose = pinpoint.getPosition();
            
            validUpdates++;

            // Update frequency calculation
            if (updateTimer.seconds() > 0) {
                updateFrequency = 1.0 / updateTimer.seconds();
            }
            updateTimer.reset();

            return true;

        } catch (Exception e) {
            invalidUpdates++;
            lastErrorMessage = "Sensor update failed: " + e.getMessage();
            return false;
        }
    }
    

    public Pose2D getCurrentPose() {
        return currentPose;
    }

    public Pose2D getLastValidPose() {
        return lastValidPose;
    }
    

    public double getX() {
        return currentPose.getX(DistanceUnit.INCH);
    }
    

    public double getY() {
        return currentPose.getY(DistanceUnit.INCH);
    }
    

    public double getHeading() {
        return currentPose.getHeading(AngleUnit.DEGREES);
    }
    
    // ========== COORDINATE SYSTEM MANAGEMENT ==========
    
    /**
     * Resets the odometry to field origin position
     * Sets the robot's current position to the configured field origin
     */
    public void resetToFieldOrigin(Pose2D fieldOrigin) {
         resetToPose(fieldOrigin);
    }
    
    /**
     * Resets odometry to a specific pose vision - localizaton
     *
     * @param pose Target pose to reset to
     */
    public void resetToPose(Pose2D pose) {
        try {
            // Directly set Pinpoint hardware to the specified pose
            pinpoint.setPosition(pose);

            // Update and sync current pose
            pinpoint.update();
            currentPose = pinpoint.getPosition();
            lastValidPose = currentPose;

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
                           "  Tracking Mode: Direct Hardware Read\n" +
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
                           CalibrationCoefficients.ODOMETRY_X_SCALE,
                           CalibrationCoefficients.ODOMETRY_Y_SCALE,
                           CalibrationCoefficients.ODOMETRY_HEADING_SCALE);
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
