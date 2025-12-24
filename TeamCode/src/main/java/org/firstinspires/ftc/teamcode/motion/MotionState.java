package org.firstinspires.ftc.teamcode.motion;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Represents the current motion state of the robot.
 * 
 * Includes:
 * - Current pose (position and heading)
 * - Current velocity (linear and angular)
 * - Target pose (goal position)
 * - Motion parameters (constraints and modes)
 */
public class MotionState {
    
    // ========== COORDINATE MODE ENUM ==========
    
    /**
     * Coordinate system mode for motion commands
     */
    public enum CoordinateMode {
        /** Velocities and positions relative to robot's current orientation */
        ROBOT_CENTRIC,
        
        /** Velocities and positions relative to fixed field coordinate system */
        FIELD_CENTRIC
    }
    
    
    // ========== STATE VARIABLES ==========
    
    /** Current robot pose (x, y in inches, heading in degrees) - REFERENCE POINT coordinates from odometry */
    private Pose2D currentPose;
    
    /** Current linear velocity in X direction (inches/sec) */
    private double velocityX;
    
    /** Current linear velocity in Y direction (inches/sec) */
    private double velocityY;
    
    /** Current angular velocity (degrees/sec) */
    private double angularVelocity;
    
    /** Target pose for navigation (x, y in inches, heading in degrees) */
    private Pose2D targetPose;
    
    /** Current coordinate mode */
    private CoordinateMode coordinateMode;
    
    
    /** Timestamp of last update (nanoseconds) */
    private long lastUpdateTime;
    
    /** Odometry manager for pose tracking and validation */
    private OdometryManager odometryManager;
    
    // ========== CONSTRUCTOR ==========
    
    /**
     * Creates a new MotionState with default values
     */
    public MotionState() {
        this.currentPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        this.velocityX = 0.0;
        this.velocityY = 0.0;
        this.angularVelocity = 0.0;
        this.targetPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        this.coordinateMode = CoordinateMode.FIELD_CENTRIC;
        this.lastUpdateTime = System.nanoTime();
        this.odometryManager = null;
    }
    
    /**
     * Creates a new MotionState with odometry integration
     * @param odometryManager The odometry manager for pose tracking
     */
    public MotionState(OdometryManager odometryManager) {
        this();
        this.odometryManager = odometryManager;
    
    }
    // ========== POSE ACCESSORS ==========
    
    /**
     * Updates the current pose from odometry
     * @param pose New pose from odometry system
     */
    public void updateCurrentPose(Pose2D pose) {
        this.currentPose = pose;
        this.lastUpdateTime = System.nanoTime();
    }
    
    /**
     * Gets the current robot pose
     * @return Current pose (x, y in inches, heading in degrees) - REFERENCE POINT coordinates
     */
    public Pose2D getCurrentPose() {
        return currentPose;
    }
    
    /**
     * Gets current X position
     * @return X position in inches (REFERENCE POINT coordinates)
     */
    public double getX() {
        return currentPose.getX(DistanceUnit.INCH);
    }
    
    /**
     * Gets current Y position
     * @return Y position in inches (REFERENCE POINT coordinates)
     */
    public double getY() {
        return currentPose.getY(DistanceUnit.INCH);
    }
    
    /**
     * Gets current heading
     * @return Heading in degrees
     */
    public double getHeading() {
        return currentPose.getHeading(AngleUnit.DEGREES);
    }
    
    // ========== VELOCITY ACCESSORS ==========
    
    /**
     * Sets the current velocity
     * @param vx Linear velocity in X direction (inches/sec)
     * @param vy Linear velocity in Y direction (inches/sec)
     * @param omega Angular velocity (degrees/sec)
     */
    public void setVelocity(double vx, double vy, double omega) {
        this.velocityX = vx;
        this.velocityY = vy;
        this.angularVelocity = omega;
    }
    
    /**
     * Gets X velocity
     * @return Velocity in X direction (inches/sec)
     */
    public double getVelocityX() {
        return velocityX;
    }
    
    /**
     * Gets Y velocity
     * @return Velocity in Y direction (inches/sec)
     */
    public double getVelocityY() {
        return velocityY;
    }
    
    /**
     * Gets angular velocity
     * @return Angular velocity (degrees/sec)
     */
    public double getAngularVelocity() {
        return angularVelocity;
    }
    
    /**
     * Gets total linear velocity magnitude
     * @return Speed in inches/sec
     */
    public double getSpeed() {
        return Math.hypot(velocityX, velocityY);
    }
    
    // ========== TARGET ACCESSORS ==========
    
    /**
     * Sets the target pose
     * @param x Target X position (inches)
     * @param y Target Y position (inches)
     * @param heading Target heading (degrees)
     */
    public void setTarget(double x, double y, double heading) {
        this.targetPose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading);
    }
    
    /**
     * Sets the target pose
     * @param pose Target pose
     */
    public void setTarget(Pose2D pose) {
        this.targetPose = pose;
    }
    
    /**
     * Gets the target pose
     * @return Target pose
     */
    public Pose2D getTargetPose() {
        return targetPose;
    }
    
    /**
     * Gets target X position
     * @return Target X in inches
     */
    public double getTargetX() {
        return targetPose.getX(DistanceUnit.INCH);
    }
    
    /**
     * Gets target Y position
     * @return Target Y in inches
     */
    public double getTargetY() {
        return targetPose.getY(DistanceUnit.INCH);
    }
    
    /**
     * Gets target heading
     * @return Target heading in degrees
     */
    public double getTargetHeading() {
        return targetPose.getHeading(AngleUnit.DEGREES);
    }
    
    // ========== ERROR CALCULATIONS ==========
    
    /**
     * Calculates position error to target
     * @return Distance to target in inches
     */
    public double getPositionError() {
        double dx = getTargetX() - getX();
        double dy = getTargetY() - getY();
        return Math.hypot(dx, dy);
    }
    
    /**
     * Calculates X position error to target
     * @return X error in inches
     */
    public double getErrorX() {
        return getTargetX() - getX();
    }
    
    /**
     * Calculates Y position error to target
     * @return Y error in inches
     */
    public double getErrorY() {
        return getTargetY() - getY();
    }
    
    /**
     * Calculates heading error to target (normalized to [-180, 180])
     * @return Heading error in degrees
     */
    public double getHeadingError() {
        double error = getTargetHeading() - getHeading();
        // Normalize to [-180, 180]
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        return error;
    }
    
    /**
     * Calculates angle to target position
     * @return Angle to target in degrees
     */
    public double getAngleToTarget() {
        double dx = getTargetX() - getX();
        double dy = getTargetY() - getY();
        return Math.toDegrees(Math.atan2(dy, dx));
    }
    
    // ========== MODE ACCESSORS ==========
    
    /**
     * Sets the coordinate mode
     * @param mode Coordinate mode (ROBOT_CENTRIC or FIELD_CENTRIC)
     */
    public void setCoordinateMode(CoordinateMode mode) {
        this.coordinateMode = mode;
    }
    
    /**
     * Gets the current coordinate mode
     * @return Current coordinate mode
     */
    public CoordinateMode getCoordinateMode() {
        return coordinateMode;
    }
    
    /**
     * Checks if in field-centric mode
     * @return true if field-centric
     */
    public boolean isFieldCentric() {
        return coordinateMode == CoordinateMode.FIELD_CENTRIC;
    }
    
    /**
     * Checks if in robot-centric mode
     * @return true if robot-centric
     */
    public boolean isRobotCentric() {
        return coordinateMode == CoordinateMode.ROBOT_CENTRIC;
    }
    
    
    // ========== TIME ACCESSORS ==========
    
    /**
     * Gets the last update timestamp
     * @return Timestamp in nanoseconds
     */
    public long getLastUpdateTime() {
        return lastUpdateTime;
    }
    
    /**
     * Calculates time since last update
     * @return Elapsed time in seconds
     */
    public double getTimeSinceUpdate() {
        return (System.nanoTime() - lastUpdateTime) / 1e9;
    }
    
    // ========== ODOMETRY ACCESSORS ==========
    
    /**
     * Gets the odometry manager
     * @return The odometry manager (can be null)
     */
    public OdometryManager getOdometryManager() {
        return odometryManager;
    }
    
    /**
     * Sets the odometry manager
     * @param odometryManager The odometry manager to use (can be null)
     */
    public void setOdometryManager(OdometryManager odometryManager) {
        this.odometryManager = odometryManager;
    }
    
    /**
     * Updates the current pose from the odometry manager if available
     */
    public void updateFromOdometry() {
        if (odometryManager != null) {
            this.currentPose = odometryManager.getCurrentPose();
            this.lastUpdateTime = System.nanoTime();
        }
    }
    
    // ========== UTILITY METHODS ==========
    
    /**
     * Resets velocities to zero
     */
    public void stopMotion() {
        this.velocityX = 0.0;
        this.velocityY = 0.0;
        this.angularVelocity = 0.0;
    }
    
    /**
     * Checks if robot is stopped (velocity below threshold)
     * @return true if stopped
     */
    public boolean isStopped() {
        return getSpeed() < MotionConfig.VELOCITY_TOLERANCE && 
               Math.abs(angularVelocity) < MotionConfig.VELOCITY_TOLERANCE;
    }
    
    /**
     * Checks if robot has reached target position
     * @return true if within position tolerance
     */
    public boolean atTargetPosition() {
        return getPositionError() <= MotionConfig.POSITION_TOLERANCE;
    }
    
    /**
     * Checks if robot has reached target heading
     * @return true if within heading tolerance
     */
    public boolean atTargetHeading() {
        return Math.abs(getHeadingError()) <= MotionConfig.HEADING_TOLERANCE;
    }
    
    /**
     * Checks if robot has reached target pose (position and heading)
     * @return true if at target
     */
    public boolean atTarget() {
        return atTargetPosition() && atTargetHeading();
    }
    
    /**
     * Creates a string representation of the motion state
     * @return String with current state information
     */
    @Override
    public String toString() {
        return String.format("MotionState[pos=(%.2f, %.2f), hdg=%.2f degrees, vel=(%.2f, %.2f), omega=%.2f, mode=%s]",
            getX(), getY(), getHeading(),
            velocityX, velocityY, angularVelocity,
            coordinateMode);
    }
}
