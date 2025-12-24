package org.firstinspires.ftc.teamcode.motion;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.calibration.RobotConstants.ComponentPosition;

/**
 * Centralized coordinate system management and transformations.
 * 
 * Responsibilities:
 * - Field origin management and odometry reset
 * - Reference point ↔ robot center conversions
 * - Coordinate mode utilities and transformations
 * - Integration with odometry system
 * 
 * This class handles all coordinate system transformations to keep them
 * separate from motion control logic and make them easily testable.
 */
public class CoordinateTransformer {
    
    // ========== DEPENDENCIES ==========
    private final OdometryManager odometry;
    private final MotionState motionState;
    
    // ========== CONSTRUCTOR ==========
    
    /**
     * Creates a new coordinate transformer
     * 
     * @param odometry Odometry manager for pose tracking
     * @param motionState Motion state for position updates
     */
    public CoordinateTransformer(OdometryManager odometry, MotionState motionState) {
        this.odometry = odometry;
        this.motionState = motionState;
    }
    
    // ========== FIELD ORIGIN MANAGEMENT ==========
    
    /**
     * Resets odometry to field origin position
     * 
     * Sets the robot's current position to the configured field origin values
     * from MotionConfig (FIELD_ORIGIN_X, FIELD_ORIGIN_Y, FIELD_ORIGIN_HEADING).
     * This establishes the coordinate system for all subsequent field-centric motion.
     * 
     * Call this at the start of autonomous with the robot positioned at a known
     * starting location on the field.
     */
    public void resetToFieldOrigin() {
        // Reset odometry to configured field origin
        odometry.resetToFieldOrigin();
        
        // Update motion state with new position
        motionState.updateFromOdometry();
    }
    
    /**
     * Set field origin using reference point coordinates
     * 
     * @param refPointX Reference point X coordinate (inches)
     * @param refPointY Reference point Y coordinate (inches)
     * @param robotHeading Robot heading (degrees)
     */
    public void setFieldOrigin(double refPointX, double refPointY, double robotHeading) {
        // Convert reference point position to robot center position
        Pose2D robotCenterOrigin = convertReferencePointToRobotCenter(refPointX, refPointY, robotHeading);
        
        // Reset odometry and set robot center position
        odometry.resetToPose(robotCenterOrigin);
        
        // Update motion state with new position
        motionState.updateFromOdometry();
    }
    
    /**
     * Set field origin using reference point position
     * 
     * @param referencePose Where the reference point is positioned at start
     */
    public void setFieldOrigin(FieldPose referencePose) {
        setFieldOrigin(referencePose.x, referencePose.y, referencePose.heading);
    }
    
    // ========== REFERENCE POINT CONVERSIONS ==========
    
    /**
     * Converts reference point field coordinates to robot center field coordinates.
     * 
     * This method takes a position where the reference point should be located and
     * calculates where the robot center needs to be positioned to achieve that.
     * 
     * @param refPointX Reference point X coordinate in field frame (inches)
     * @param refPointY Reference point Y coordinate in field frame (inches)
     * @param robotHeading Robot heading (degrees)
     * @return Robot center position in field frame
     */
    public Pose2D convertReferencePointToRobotCenter(double refPointX, double refPointY, double robotHeading) {
        ComponentPosition refPoint = MotionConfig.ACTIVE_REFERENCE_POINT;
        
        // Rotate reference point offset by robot heading
        double headingRad = Math.toRadians(robotHeading);
        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);
        
        // Transform reference point offset from robot frame to field frame
        double offsetX = refPoint.x * cos - refPoint.y * sin;
        double offsetY = refPoint.x * sin + refPoint.y * cos;
        
        // Robot center = reference point position - rotated offset
        double robotX = refPointX - offsetX;
        double robotY = refPointY - offsetY;
        
        return new Pose2D(DistanceUnit.INCH, robotX, robotY, AngleUnit.DEGREES, robotHeading);
    }
    
    /**
     * Converts robot center field coordinates to reference point field coordinates.
     * 
     * This method takes the robot center position and calculates where the
     * reference point is located in field coordinates.
     * 
     * @param robotCenterPose Robot center position in field frame
     * @return Reference point position as FieldPose
     */
    public FieldPose convertRobotCenterToReferencePoint(Pose2D robotCenterPose) {
        ComponentPosition refPoint = MotionConfig.ACTIVE_REFERENCE_POINT;
        
        double robotX = robotCenterPose.getX(DistanceUnit.INCH);
        double robotY = robotCenterPose.getY(DistanceUnit.INCH);
        double robotHeading = robotCenterPose.getHeading(AngleUnit.DEGREES);
        
        // Rotate reference point offset by robot heading
        double headingRad = Math.toRadians(robotHeading);
        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);
        
        // Transform reference point offset from robot frame to field frame
        double offsetX = refPoint.x * cos - refPoint.y * sin;
        double offsetY = refPoint.x * sin + refPoint.y * cos;
        
        // Reference point = robot center + rotated offset
        double refPointX = robotX + offsetX;
        double refPointY = robotY + offsetY;
        
        return new FieldPose(refPointX, refPointY, robotHeading);
    }
    
    // ========== COORDINATE MODE UTILITIES ==========
    

    /**
     * Calculates absolute field angle from relative angle and coordinate mode
     * 
     * @param angle Input angle (degrees)
     * @param coordinateMode ROBOT_CENTRIC or FIELD_CENTRIC
     * @param robotHeading Current robot heading (degrees)
     * @return Absolute field angle (degrees)
     */
    public double calculateFieldAngle(double angle, MotionState.CoordinateMode coordinateMode, double robotHeading) {
        if (coordinateMode == MotionState.CoordinateMode.ROBOT_CENTRIC) {
            return robotHeading + angle;  // Convert robot-relative to field-absolute
        } else {
            return angle;  // Already field-absolute
        }
    }
    
    // ========== CURRENT POSITION UTILITIES ==========
    
    /**
     * Gets current reference point position in field coordinates
     * 
     * @return Current reference point pose
     */
    public FieldPose getCurrentReferencePointPose() {
        // motionState.getCurrentPose() returns REFERENCE POINT coordinates from odometry
        Pose2D refPointPose = motionState.getCurrentPose();
        return new FieldPose(
            refPointPose.getX(DistanceUnit.INCH),
            refPointPose.getY(DistanceUnit.INCH),
            refPointPose.getHeading(AngleUnit.DEGREES)
        );
    }
    
    /**
     * Gets current robot center position in field coordinates
     * 
     * @return Current robot center pose
     */
    public Pose2D getCurrentRobotCenterPose() {
        // motionState.getCurrentPose() returns REFERENCE POINT coordinates from odometry
        // Convert to robot center coordinates
        Pose2D refPointPose = motionState.getCurrentPose();
        return convertReferencePointToRobotCenter(
            refPointPose.getX(DistanceUnit.INCH),
            refPointPose.getY(DistanceUnit.INCH),
            refPointPose.getHeading(AngleUnit.DEGREES)
        );
    }
    
    // ========== COORDINATE VALIDATION ==========
    
    /**
     * Validates that a field position is within reasonable bounds
     * 
     * @param x X coordinate (inches)
     * @param y Y coordinate (inches)
     * @return true if position is valid
     */
    public boolean isValidFieldPosition(double x, double y) {
        // FTC field is typically 12' x 12' (144" x 144")
        // Allow some margin for positioning outside field boundaries
        final double MAX_FIELD_COORDINATE = 180.0;  // inches
        final double MIN_FIELD_COORDINATE = -36.0;  // inches
        
        return x >= MIN_FIELD_COORDINATE && x <= MAX_FIELD_COORDINATE &&
               y >= MIN_FIELD_COORDINATE && y <= MAX_FIELD_COORDINATE;
    }
    
    /**
     * Validates that a heading is within valid range
     * 
     * @param heading Heading in degrees
     * @return true if heading is valid
     */
    public boolean isValidHeading(double heading) {
        // Headings should be normalized to [-180, 180] or [0, 360]
        // We'll accept any reasonable range
        return heading >= -720.0 && heading <= 720.0;
    }
}
