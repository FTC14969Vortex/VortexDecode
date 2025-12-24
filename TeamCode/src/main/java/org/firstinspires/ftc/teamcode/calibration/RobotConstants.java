package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.helper.GoBildaPinpointDriver;

/**
 * Single source of truth for all robot constants including geometry, hardware specifications,
 * and component positioning.
 * 
 * This consolidated file contains:
 * 1. Physical dimensions and geometry
 * 2. Motor specifications and directions
 * 3. Component positions and coordinate system definitions
 * 4. ComponentPosition utility class (consolidated from separate file)
 * 
 * REFERENCE COORDINATE SYSTEM:
 * - Origin: Geometric center of drivetrain at floor level
 * - +X: Forward (toward scoring mechanism)
 * - +Y: Left (driver's left when facing forward)
 * - +Z: Up (away from floor)
 * - Units: inches for distance, degrees for angles
 */
public class RobotConstants {
    
    // ========== COMPONENT POSITION UTILITY CLASS ==========
    
    /**
     * Represents the position and orientation of a robot component relative to the robot's reference point.
     * 
     * COORDINATE SYSTEM:
     * - Reference Point: Geometric center of drivetrain at floor level
     * - +X: Forward (toward scoring mechanism)
     * - +Y: Left (driver's left when facing forward)
     * - +Z: Up (away from floor)
     * - Units: inches for position, degrees for orientation
     * 
     * ORIENTATION:
     * - Roll: Rotation around X-axis (positive = right side down)
     * - Pitch: Rotation around Y-axis (positive = nose up)
     * - Yaw: Rotation around Z-axis (positive = counter-clockwise from above)
     */
    public static class ComponentPosition {
        
        // ========== POSITION COORDINATES ==========
        
        /** X coordinate relative to robot reference point (inches, +X = forward) */
        public final double x;
        
        /** Y coordinate relative to robot reference point (inches, +Y = left) */
        public final double y;
        
        /** Z coordinate relative to robot reference point (inches, +Z = up) */
        public final double z;
        
        // ========== ORIENTATION ANGLES ==========
        
        /** Roll angle - rotation around X-axis (degrees, positive = right side down) */
        public final double roll;
        
        /** Pitch angle - rotation around Y-axis (degrees, positive = nose up) */
        public final double pitch;
        
        /** Yaw angle - rotation around Z-axis (degrees, positive = counter-clockwise) */
        public final double yaw;
        
        // ========== CONSTRUCTORS ==========
        
        /**
         * Creates a component position with full 6-DOF specification
         * 
         * @param x X coordinate (inches, +X = forward)
         * @param y Y coordinate (inches, +Y = left)
         * @param z Z coordinate (inches, +Z = up)
         * @param roll Roll angle (degrees, rotation around X-axis)
         * @param pitch Pitch angle (degrees, rotation around Y-axis)
         * @param yaw Yaw angle (degrees, rotation around Z-axis)
         */
        public ComponentPosition(double x, double y, double z, double roll, double pitch, double yaw) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.roll = roll;
            this.pitch = pitch;
            this.yaw = yaw;
        }        
    
        public ComponentPosition(double x, double y, double z) {
            this(x, y, z, 0.0, 0.0, 0.0);
        }        
        
        public ComponentPosition(double x, double y) {
            this(x, y, 0.0, 0.0, 0.0, 0.0);
        }
        
       
        public double getDistance2D() {
            return Math.sqrt(x * x + y * y);
        }
        
       
        public double getDistance3D() {
            return Math.sqrt(x * x + y * y + z * z);
        }
        
        
        public double getAngleFromReference() {
            return Math.toDegrees(Math.atan2(y, x));
        }
                
        public ComponentPosition translate(double deltaX, double deltaY, double deltaZ) {
            return new ComponentPosition(x + deltaX, y + deltaY, z + deltaZ, roll, pitch, yaw);
        }        
        
        public ComponentPosition rotate(double deltaRoll, double deltaPitch, double deltaYaw) {
            return new ComponentPosition(x, y, z, roll + deltaRoll, pitch + deltaPitch, yaw + deltaYaw);
        }
        
        
        public boolean isValid(double maxDistance) {
            // Check for NaN or infinite values
            if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(z) ||
                !Double.isFinite(roll) || !Double.isFinite(pitch) || !Double.isFinite(yaw)) {
                return false;
            }
            
            // Check distance from reference point
            if (getDistance3D() > maxDistance) {
                return false;
            }
            
            // Check angle ranges
            if (Math.abs(roll) > 180 || Math.abs(pitch) > 180 || Math.abs(yaw) > 180) {
                return false;
            }
            
            return true;
        }        
      
        public boolean isValid() {
            return isValid(24.0); // 24" max distance for typical FTC robot
        }
        
        // ========== STRING REPRESENTATION ==========
        
        @Override
        public String toString() {
            return String.format("ComponentPosition[x=%.2f, y=%.2f, z=%.2f, roll=%.1f degrees, pitch=%.1f degrees, yaw=%.1f degrees]",
                               x, y, z, roll, pitch, yaw);
        }
        
        /**
         * Returns a compact string representation showing only position
         * 
         * @return Compact position string
         */
        public String toPositionString() {
            return String.format("(%.2f, %.2f, %.2f)", x, y, z);
        }
              
        public String toOrientationString() {
            return String.format("(%.1f degrees, %.1f degrees, %.1f degrees)", roll, pitch, yaw);
        }
        
        // ========== EQUALITY AND COMPARISON ==========
        
        @Override
        public boolean equals(Object obj) {
            if (this == obj) return true;
            if (obj == null || getClass() != obj.getClass()) return false;
            
            ComponentPosition other = (ComponentPosition) obj;
            
            final double EPSILON = 1e-6; // Tolerance for floating point comparison
            
            return Math.abs(x - other.x) < EPSILON &&
                   Math.abs(y - other.y) < EPSILON &&
                   Math.abs(z - other.z) < EPSILON &&
                   Math.abs(roll - other.roll) < EPSILON &&
                   Math.abs(pitch - other.pitch) < EPSILON &&
                   Math.abs(yaw - other.yaw) < EPSILON;
        }
        
        @Override
        public int hashCode() {
            // Simple hash based on rounded values
            return (int) (Math.round(x * 1000) + Math.round(y * 1000) * 31 + Math.round(z * 1000) * 961);
        }
        
        /**
         * Checks if this position is approximately equal to another within tolerance
         * 
         * @param other Other ComponentPosition to compare
         * @param positionTolerance Position tolerance in inches
         * @param angleTolerance Angle tolerance in degrees
         * @return true if positions are within tolerance
         */
        public boolean isNear(ComponentPosition other, double positionTolerance, double angleTolerance) {
            return Math.abs(x - other.x) <= positionTolerance &&
                   Math.abs(y - other.y) <= positionTolerance &&
                   Math.abs(z - other.z) <= positionTolerance &&
                   Math.abs(roll - other.roll) <= angleTolerance &&
                   Math.abs(pitch - other.pitch) <= angleTolerance &&
                   Math.abs(yaw - other.yaw) <= angleTolerance;
        }
    }
    
    // ========== PHYSICAL DIMENSIONS ==========
    
    /**
     * Actual wheel diameter (inches)
     * MEASURE: Roll robot exactly 10 wheel rotations, measure distance
     * Calculate: actual_diameter = distance / (10 * pi)
     * CRITICAL: This directly affects all distance calculations!
     */
    public static final double WHEEL_DIAMETER = 4.096;  // TODO: MEASURE THIS VALUE!
    
    /**
     * Gear reduction ratio (motor revolutions per wheel revolution)
     * For direct drive: 1.0, For geared systems: gear_ratio
     */
    public static final double GEAR_REDUCTION = 1.0;  // Yes
    
    // ========== DRIVETRAIN WHEEL POSITIONS ==========
    // REFERENCE POINT: Back-right wheel center is used as the origin (0,0,0)
    // This provides a stable, easily accessible, and clearly defined reference point
    // for goBILDA mecanum wheel systems. Using back-right makes most coordinates positive.
    //
    // MEASUREMENT INSTRUCTIONS:
    // 1. Mark the center of the back-right wheel hub clearly
    // 2. Use calipers or ruler to measure from back-right wheel center to other wheel centers
    // 3. Positive X = forward, Positive Y = left, Positive Z = up
    // 4. Measure to 1/8 inch accuracy for best results
    
    /**
     * Back right wheel center position - THIS IS THE REFERENCE POINT (origin)
     * Always (0,0,0) since this is our coordinate system origin
     */

    public static final double TRACK_WIDTH = 11.97;  // 304/25.4, measured from wheel centers. - Y direction
    public static final double WHEELBASE = 9.45;  // 240/25.4, measured from wheel centers - X direction
    public static final double ROBOT_LENGTH = 17.0;  // 18U of gobilda chassis height.
    public static final double ROBOT_WIDTH = 14.2;  // 60mm + 240mm + 60mm = 360mm = 14.1732 inches    
    public static final double ROBOT_HEIGHT = 15.0;  
    
    // ========== FUNCTIONAL COMPONENT POSITIONS ==========

    public static final ComponentPosition BACK_RIGHT_WHEEL_REF = 
        new ComponentPosition(0.0, 0.0, 0.0, 0, 0, 0);  // REFERENCE POINT - DO NOT CHANGE
    
    public static final ComponentPosition BACK_LEFT_WHEEL_REF = BACK_RIGHT_WHEEL_REF.translate(0, TRACK_WIDTH, 0); // 304/25.4
    public static final ComponentPosition FRONT_RIGHT_WHEEL_REF = BACK_RIGHT_WHEEL_REF.translate(WHEELBASE, 0.0, 0); // 240/25.4        
    public static final ComponentPosition FRONT_LEFT_WHEEL_REF = FRONT_RIGHT_WHEEL_REF.translate(0, TRACK_WIDTH, 0); // 304/25.4)

    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;

    public static final double DRIVETRAIN_RADIUS = Math.sqrt(
        (TRACK_WIDTH / 2.0) * (TRACK_WIDTH / 2.0) + 
        (WHEELBASE / 2.0) * (WHEELBASE / 2.0)
    );

     // other positions relative to back-right wheel reference point
     // ODOMETRY SENSOR, CAMERAS, INTAKE POINT, SCORING POINT, Front/Back/Left/Right references, DISTANCE SENSOR  
  
    public static final ComponentPosition ODOMETRY_SENSOR_REF = BACK_RIGHT_WHEEL_REF.translate(-1, 7.87, 0);  // -24/25.4, 200/25.4
    public static final ComponentPosition BACK_CAMERA_REF = BACK_RIGHT_WHEEL_REF.translate(3.78, 5.98,0); // 96/25.4, 152/25.4        
    public static final ComponentPosition FRONT_CAMERA_REF = FRONT_RIGHT_WHEEL_REF.translate(1.0, -TRACK_WIDTH/2,0); // To be implemented in the middle of the front side.         
    
    public static final ComponentPosition INTAKE_POINT_REF = 
        new ComponentPosition(11.34, 5.98,  0.0, 0, 0, 0);  // y= 152/25.4, x=288/25.4
    
    
    public static final ComponentPosition SCORING_POINT_REF = BACK_RIGHT_WHEEL_REF.translate(-3.78, TRACK_WIDTH/2, 0);  // x= -96/25.4, y= 152/25.4
    
    public static final ComponentPosition ROBOT_FRONT_REF = INTAKE_POINT_REF;
    public static final ComponentPosition ROBOT_BACK_REF = SCORING_POINT_REF;
        
    public static final ComponentPosition ROBOT_LEFT_REF = 
        new ComponentPosition(0.0, +9.0, 0.0, 0, 0, 0);  // TODO: MEASURE THESE VALUES!
    
    public static final ComponentPosition ROBOT_RIGHT_REF = 
        new ComponentPosition(4.72, -1.0, 0.0, 0, 0, 0);  // TODO: MEASURE THESE VALUES!    
         
    public static final ComponentPosition DISTANCE_SENSOR_REF = 
        new ComponentPosition(+8.0, +6.0, 4.0, 0, 0, 45);  // TODO: MEASURE THESE VALUES!
    

    
    // ========== COORDINATE TRANSFORMATION UTILITIES ==========


    
    /**
     * Converts a position from reference point coordinates to robot center coordinates
     * 
     * @param refPosition Position relative to physical reference point
     * @return Position relative to robot center (geometric center of wheels)
     */
    public static ComponentPosition convertToRobotCenter(ComponentPosition refPosition) {
        ComponentPosition robotCenter = calculateRobotCenter();
        return new ComponentPosition(
            refPosition.x - robotCenter.x,
            refPosition.y - robotCenter.y,
            refPosition.z - robotCenter.z,
            refPosition.roll,
            refPosition.pitch,
            refPosition.yaw
        );
    }
    
    /**
     * Converts a position from robot center coordinates to reference point coordinates
     * 
     * @param robotPosition Position relative to robot center
     * @return Position relative to physical reference point
     */
    public static ComponentPosition convertToReferencePoint(ComponentPosition robotPosition) {
        ComponentPosition robotCenter = calculateRobotCenter();
        return new ComponentPosition(
            robotPosition.x + robotCenter.x,
            robotPosition.y + robotCenter.y,
            robotPosition.z + robotCenter.z,
            robotPosition.roll,
            robotPosition.pitch,
            robotPosition.yaw
        );
    }
    
    
    /**
     * Calculates the geometric center of the robot (center of 4 wheels) from wheel measurements
     */
    public static ComponentPosition calculateRobotCenter() {
        double centerX = (FRONT_LEFT_WHEEL_REF.x + FRONT_RIGHT_WHEEL_REF.x + 
                         BACK_LEFT_WHEEL_REF.x + BACK_RIGHT_WHEEL_REF.x) / 4.0;
        double centerY = (FRONT_LEFT_WHEEL_REF.y + FRONT_RIGHT_WHEEL_REF.y + 
                         BACK_LEFT_WHEEL_REF.y + BACK_RIGHT_WHEEL_REF.y) / 4.0;
        double centerZ = (FRONT_LEFT_WHEEL_REF.z + FRONT_RIGHT_WHEEL_REF.z + 
                         BACK_LEFT_WHEEL_REF.z + BACK_RIGHT_WHEEL_REF.z) / 4.0;
        
        return new ComponentPosition(centerX, centerY, centerZ, 0, 0, 0);
    }
    
    // ========== CALCULATED ROBOT CENTER ==========
    
    /**
     * Robot center position (geometric center of 4 wheels) relative to reference point
     * This is calculated from the measured wheel positions
     */
    public static final ComponentPosition ROBOT_CENTER_FROM_REF = calculateRobotCenter();
    
    // ========== WHEEL POSITIONS RELATIVE TO ROBOT CENTER ==========

    public static final ComponentPosition Robot_CENTER =
        convertToRobotCenter(ROBOT_CENTER_FROM_REF);
    
    public static final ComponentPosition BACK_RIGHT_WHEEL = 
        convertToRobotCenter(BACK_RIGHT_WHEEL_REF);    
    
    public static final ComponentPosition BACK_LEFT_WHEEL = 
        convertToRobotCenter(BACK_LEFT_WHEEL_REF);    
   
    public static final ComponentPosition FRONT_RIGHT_WHEEL = 
        convertToRobotCenter(FRONT_RIGHT_WHEEL_REF);    
    
    public static final ComponentPosition FRONT_LEFT_WHEEL = 
        convertToRobotCenter(FRONT_LEFT_WHEEL_REF);

    public static final ComponentPosition ODOMETRY_SENSOR = 
        convertToRobotCenter(ODOMETRY_SENSOR_REF);    
  
    public static final ComponentPosition BACK_CAMERA = 
        convertToRobotCenter(BACK_CAMERA_REF);    
   
    public static final ComponentPosition FRONT_CAMERA = 
        convertToRobotCenter(FRONT_CAMERA_REF);

    public static final ComponentPosition INTAKE_POINT = 
        convertToRobotCenter(INTAKE_POINT_REF);
    
    public static final ComponentPosition SCORING_POINT =
        convertToRobotCenter(SCORING_POINT_REF);
    
    public static final ComponentPosition ROBOT_FRONT =
        convertToRobotCenter(ROBOT_FRONT_REF);

    public static final ComponentPosition ROBOT_BACK =
        convertToRobotCenter(ROBOT_BACK_REF);

    public static final ComponentPosition ROBOT_LEFT =
        convertToRobotCenter(ROBOT_LEFT_REF);

    public static final ComponentPosition ROBOT_RIGHT =
        convertToRobotCenter(ROBOT_RIGHT_REF);

    public static final ComponentPosition DISTANCE_SENSOR =
        convertToRobotCenter(DISTANCE_SENSOR_REF);
    
    // ========== MOTOR SPECIFICATIONS ==========
    
    /**
     * Motor maximum RPM at 12V (no load) - from goBILDA 5203 datasheet
     */
    public static final double MOTOR_MAX_RPM_DATASHEET = 312.0;
    
    /**
     * Encoder counts per motor revolution - from goBILDA 5203 datasheet
     */
    public static final double ENCODER_COUNTS_PER_REV = 537.7;
    
    public static final double MAX_THEORETICAL_LINEAR_VELOCITY =
        (MOTOR_MAX_RPM_DATASHEET / 60.0) * WHEEL_CIRCUMFERENCE;
    
    
    public static final double MAX_THEORETICAL_ANGULAR_VELOCITY =
        Math.toDegrees(MAX_THEORETICAL_LINEAR_VELOCITY / DRIVETRAIN_RADIUS);
    
        
    /**
     * Motor stall torque at 12V - from datasheet (kg*cm)
     * Used for theoretical calculations and safety limits
     */
    public static final double MOTOR_STALL_TORQUE = 3.2;
    
    /**
     * Motor stall current at 12V - from datasheet (Amps)
     * Used for current limiting and thermal protection
     */
    public static final double MOTOR_STALL_CURRENT = 9.8;
    
    // ========== MOTOR DIRECTION CONFIGURATION ==========
    
    /**
     * IMPORTANT: Motor Direction and Encoder Reversal are INDEPENDENT settings!
     * 
     * MOTOR DIRECTION controls software power inversion:
     * - FORWARD: setPower(0.5) sends +0.5 to motor
     * - REVERSE: setPower(0.5) sends -0.5 to motor (software inverts)
     * 
     * Test each motor: Positive power should contribute to +X (forward) motion
     */
    
    /**
     * Front left motor direction
     * VERIFY: Positive power should contribute to +X (forward) and +Y (left) motion
     */
    public static final DcMotorSimple.Direction FRONT_LEFT_DIRECTION = 
        DcMotorSimple.Direction.REVERSE;  // TODO: VERIFY ON YOUR ROBOT!
    
    /**
     * Front right motor direction
     * VERIFY: Positive power should contribute to +X (forward) and -Y (right) motion
     */
    public static final DcMotorSimple.Direction FRONT_RIGHT_DIRECTION = 
        DcMotorSimple.Direction.FORWARD;  // TODO: VERIFY ON YOUR ROBOT!
    
    /**
     * Back left motor direction
     * VERIFY: Positive power should contribute to +X (forward) and +Y (left) motion
     */
    public static final DcMotorSimple.Direction BACK_LEFT_DIRECTION = 
        DcMotorSimple.Direction.REVERSE;  // TODO: VERIFY ON YOUR ROBOT!
    
    /**
     * Back right motor direction
     * VERIFY: Positive power should contribute to +X (forward) and -Y (right) motion
     */
    public static final DcMotorSimple.Direction BACK_RIGHT_DIRECTION = 
        DcMotorSimple.Direction.FORWARD;  // TODO: VERIFY ON YOUR ROBOT!
    
    // ========== ENCODER DIRECTION CONFIGURATION ==========
    
    /**
     * ENCODER REVERSAL is INDEPENDENT of MOTOR DIRECTION!
     * 
     * These settings control whether encoder readings are negated:
     * - false: Use encoder readings as-is (normal A/B channel wiring)
     * - true: Negate encoder readings (swapped A/B channels or reversed mounting)
     * 
     * Test each encoder: Move robot forward and check if encoder counts INCREASE.
     * If encoder counts DECREASE during forward motion, set ENCODER_REVERSED = true.
     */
    
    /**
     * Front left encoder direction reversed
     * VERIFY: Encoder should increase when motor contributes to forward motion
     */
    public static final boolean FRONT_LEFT_ENCODER_REVERSED = false;  // TODO: VERIFY ON YOUR ROBOT!
    
    /**
     * Front right encoder direction reversed
     * VERIFY: Encoder should increase when motor contributes to forward motion
     */
    public static final boolean FRONT_RIGHT_ENCODER_REVERSED = true;  // TODO: VERIFY ON YOUR ROBOT!
    
    /**
     * Back left encoder direction reversed
     * VERIFY: Encoder should increase when motor contributes to forward motion
     */
    public static final boolean BACK_LEFT_ENCODER_REVERSED = false;  // TODO: VERIFY ON YOUR ROBOT!
    
    /**
     * Back right encoder direction reversed
     * VERIFY: Encoder should increase when motor contributes to forward motion
     */
    public static final boolean BACK_RIGHT_ENCODER_REVERSED = true;  // TODO: VERIFY ON YOUR ROBOT!
    
    // ========== MOTOR DIRECTION BOOLEAN FLAGS ==========
    
    /**
     * Boolean flags indicating if motor direction is REVERSE (for calibration modules)
     */
    public static final boolean FRONT_LEFT_REVERSED = (FRONT_LEFT_DIRECTION == DcMotorSimple.Direction.REVERSE);
    public static final boolean FRONT_RIGHT_REVERSED = (FRONT_RIGHT_DIRECTION == DcMotorSimple.Direction.REVERSE);
    public static final boolean BACK_LEFT_REVERSED = (BACK_LEFT_DIRECTION == DcMotorSimple.Direction.REVERSE);
    public static final boolean BACK_RIGHT_REVERSED = (BACK_RIGHT_DIRECTION == DcMotorSimple.Direction.REVERSE);
    
    // ========== ODOMETRY ENCODER DIRECTION CONFIGURATION ==========
    
    /**
     * GoBilda Pinpoint X-encoder direction (forward/backward encoder)    
     * Applied in OdometryManager.setEncoderDirections()
     */
    public static final GoBildaPinpointDriver.EncoderDirection ODOMETRY_X_ENCODER_DIRECTION = 
        GoBildaPinpointDriver.EncoderDirection.REVERSED;  // TODO: VERIFY ON YOUR ROBOT!
    
    /**
     * GoBilda Pinpoint Y-encoder direction (left/rigth encoder)   
     */
    public static final GoBildaPinpointDriver.EncoderDirection ODOMETRY_Y_ENCODER_DIRECTION = 
        GoBildaPinpointDriver.EncoderDirection.REVERSED;  // TODO: VERIFY ON YOUR ROBOT!
    
    /**
     * Convenience boolean flags for odometry encoder direction checking (for calibration modules)
     */
    public static final boolean ODOMETRY_X_ENCODER_REVERSED = 
        (ODOMETRY_X_ENCODER_DIRECTION == GoBildaPinpointDriver.EncoderDirection.REVERSED);
    public static final boolean ODOMETRY_Y_ENCODER_REVERSED = 
        (ODOMETRY_Y_ENCODER_DIRECTION == GoBildaPinpointDriver.EncoderDirection.REVERSED);
    
   
    
    // ========== SUMMARY METHODS ==========
    
    /**
     * Returns a summary of the robot constants for documentation
     * 
     * @return Formatted string with key measurements
     */
    public static String getConstantsSummary() {
        StringBuilder summary = new StringBuilder();
        summary.append("=== ROBOT CONSTANTS SUMMARY ===\n");
        summary.append(String.format("Robot Center Offset from Reference: %s\n", ROBOT_CENTER_FROM_REF.toPositionString()));
        summary.append(String.format("Track Width: %.2f\"\n", TRACK_WIDTH));
        summary.append(String.format("Wheelbase: %.2f\"\n", WHEELBASE));
        summary.append(String.format("Drivetrain Radius: %.2f\"\n", DRIVETRAIN_RADIUS));
        summary.append(String.format("Robot Dimensions: %.1f\" x %.1f\" x %.1f\"\n", 
                                    ROBOT_LENGTH, ROBOT_WIDTH, ROBOT_HEIGHT));
        
        summary.append("\nPhysical Dimensions:\n");
        summary.append(String.format("  Wheel Diameter: %.3f inches\n", WHEEL_DIAMETER));
        summary.append(String.format("  Wheel Circumference: %.3f inches\n", WHEEL_CIRCUMFERENCE));
        
        summary.append("\nMotor Specifications:\n");
        summary.append(String.format("  Datasheet Max RPM: %.0f\n", MOTOR_MAX_RPM_DATASHEET));
        summary.append(String.format("  Encoder Counts/Rev: %.1f\n", ENCODER_COUNTS_PER_REV));
        summary.append(String.format("  Gear Reduction: %.1f:1\n", GEAR_REDUCTION));
        
        summary.append("\nWheel Positions (from Reference Point):\n");
        summary.append(String.format("  Front Left:  %s\n", FRONT_LEFT_WHEEL_REF.toPositionString()));
        summary.append(String.format("  Front Right: %s\n", FRONT_RIGHT_WHEEL_REF.toPositionString()));
        summary.append(String.format("  Back Left:   %s\n", BACK_LEFT_WHEEL_REF.toPositionString()));
        summary.append(String.format("  Back Right:  %s\n", BACK_RIGHT_WHEEL_REF.toPositionString()));
        
        summary.append("\nWheel Positions (from Robot Center):\n");
        summary.append(String.format("  Front Left:  %s\n", FRONT_LEFT_WHEEL.toPositionString()));
        summary.append(String.format("  Front Right: %s\n", FRONT_RIGHT_WHEEL.toPositionString()));
        summary.append(String.format("  Back Left:   %s\n", BACK_LEFT_WHEEL.toPositionString()));
        summary.append(String.format("  Back Right:  %s\n", BACK_RIGHT_WHEEL.toPositionString()));
        
        summary.append("\nSensor Positions (from Robot Center):\n");
        summary.append(String.format("  Odometry:     %s\n", ODOMETRY_SENSOR.toPositionString()));
        summary.append(String.format("  Back Camera:  %s\n", BACK_CAMERA.toPositionString()));
        summary.append(String.format("  Front Camera: %s\n", FRONT_CAMERA.toPositionString()));        
        
        return summary.toString();
    }
    

}
