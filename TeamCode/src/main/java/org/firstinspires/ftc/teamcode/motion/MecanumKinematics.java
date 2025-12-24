package org.firstinspires.ftc.teamcode.motion;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.calibration.CalibrationCoefficients;

/**
 * Pure mathematical functions for mecanum wheel kinematics.
 * 
 * Layer 1: Low-level motion primitives
 * 
 * Responsibilities:
 * - Convert robot velocities to wheel velocities (forward kinematics)
 * - Convert wheel velocities to robot velocities (inverse kinematics)
 * - Transform vectors between robot-centric and field-centric coordinate frames
 * - Normalize and scale wheel powers/velocities
 * 
 * This class contains only pure functions - no state, no hardware access.
 * All functions are thread-safe by design.
 */
public class MecanumKinematics {
    
    /**
     * Applies motor direction correction to velocity components.
     * When motor is REVERSE, the vy and omega contributions must be inverted.
     * vx component is NOT affected by motor direction.
     * 
     * @param vxContribution The vx contribution (never inverted)
     * @param vyContribution The vy contribution (inverted for REVERSE)
     * @param omegaContribution The omega contribution (inverted for REVERSE)
     * @param direction The motor direction (FORWARD or REVERSE)
     * @return Total wheel velocity with direction correction applied
     */
    private static double applyMotorDirectionCorrection(double vxContribution, double vyContribution, 
                                                        double omegaContribution, DcMotorSimple.Direction direction) {
        if (direction == DcMotorSimple.Direction.REVERSE) {
            // REVERSE motors: flip signs of vy and omega, keep vx unchanged
            return vxContribution - vyContribution - omegaContribution;
        } else {
            // FORWARD motors: use standard signs
            return vxContribution + vyContribution + omegaContribution;
        }
    }
    
    /**
     * Represents the velocities/powers for all four mecanum wheels
     */
    public static class WheelVelocities {
        public double frontLeft;
        public double frontRight;
        public double backLeft;
        public double backRight;
        
        public WheelVelocities(double fl, double fr, double bl, double br) {
            this.frontLeft = fl;
            this.frontRight = fr;
            this.backLeft = bl;
            this.backRight = br;
        }
        
        @Override
        public String toString() {
            return String.format("Wheels[FL=%.2f, FR=%.2f, BL=%.2f, BR=%.2f]", 
                frontLeft, frontRight, backLeft, backRight);
        }
    }
    
    /**
     * Represents a 2D velocity vector with rotation
     */
    public static class Velocity2D {
        public double vx;  // Linear velocity in X direction (inches/sec)
        public double vy;  // Linear velocity in Y direction (inches/sec)
        public double omega;  // Angular velocity (degrees/sec)
        
        public Velocity2D(double vx, double vy, double omega) {
            this.vx = vx;
            this.vy = vy;
            this.omega = omega;
        }
        
        @Override
        public String toString() {
            return String.format("Vel[vx=%.2f, vy=%.2f, omega=%.3f]", vx, vy, omega);
        }
    }
    
    // ========== FORWARD KINEMATICS ==========
    
    /**
     * Converts desired robot velocity to individual wheel velocities.
     * 
     * Automatically adjusts equations based on motor directions configured in RobotConstants.
     * The FTC SDK inverts velocity commands for REVERSE motors, so we pre-invert the equations
     * to compensate. This makes the method work with ANY motor direction configuration.
     * 
     * Standard mecanum (all motors FORWARD):
     *   FL = vx + vy + omega*R,  FR = vx - vy - omega*R
     *   BL = vx - vy + omega*R,  BR = vx + vy - omega*R
     * 
     * @param vx Forward velocity (inches/sec, positive = forward)
     * @param vy Strafe velocity (inches/sec, positive = left)
     * @param omega Rotational velocity (degrees/sec, positive = counter-clockwise)
     * @return Wheel velocities (inches/sec or as power ratio if normalized)
     */
    public static WheelVelocities robotVelocitiesToWheelVelocities(double vx, double vy, double omega) {
        // Calculate the effective turning radius (distance from robot center to wheel)
        double turningRadius = Math.sqrt(
            Math.pow(MotionConfig.TRACK_WIDTH / 2.0, 2) + 
            Math.pow(MotionConfig.WHEELBASE / 2.0, 2)
        );
        // Convert omega from degrees/sec to radians/sec for calculation
        double omegaRad = Math.toRadians(omega);
        double omegaContribution = omegaRad * turningRadius;
        
        // Mecanum wheel equations with motor direction correction
        // Standard mecanum (all FORWARD): FL=vx+vy+ω, FR=vx-vy-ω, BL=vx-vy+ω, BR=vx+vy-ω
        // 
        // Current config (FL/BL=REVERSE, FR/BR=FORWARD) produces:
        // FL=vx-vy-ω, FR=vx+vy+ω, BL=vx+vy-ω, BR=vx-vy+ω
        //double fl = vx - vy - omegaContribution;
        //double fr = vx + vy + omegaContribution;
        //double bl = vx + vy - omegaContribution;
        //double br = vx - vy + omegaContribution;


        int flDir = (MotionConfig.FRONT_LEFT_DIRECTION == DcMotorSimple.Direction.FORWARD) ? 1 : -1;
        int frDir = (MotionConfig.FRONT_RIGHT_DIRECTION == DcMotorSimple.Direction.FORWARD) ? 1 : -1;
        int blDir = (MotionConfig.BACK_LEFT_DIRECTION == DcMotorSimple.Direction.FORWARD) ? 1 : -1;
        int brDir = (MotionConfig.BACK_RIGHT_DIRECTION == DcMotorSimple.Direction.FORWARD) ? 1 : -1;
        
        // Base patterns: Front wheels (vy + omega), Back wheels (-vy + omega)
        // Motor direction flips the vy and omega contributions
        double fl = vx + (vy + omegaContribution) * flDir;
        double fr = vx + (vy + omegaContribution) * frDir;
        double bl = vx + (-vy + omegaContribution) * blDir;
        double br = vx + (-vy + omegaContribution) * brDir;
        
        return new WheelVelocities(fl, fr, bl, br);
    }
    
    /**
     * Converts desired robot velocity to wheel velocities with 8-factor scaling applied.
     * 
     * This applies calibrated scaling factors to compensate for individual wheel performance differences.
     * Each wheel has separate scaling for positive and negative contributions.
     * 
     * @param vx Forward velocity (inches/sec)
     * @param vy Strafe velocity (inches/sec)
     * @param omega Rotational velocity (degrees/sec)
     * @return Scaled wheel velocities (inches/sec)
     */
    public static WheelVelocities robotVelocitiesToWheelVelocities8Factor(double vx, double vy, double omega) {
        // First calculate ideal wheel velocities
        WheelVelocities ideal = robotVelocitiesToWheelVelocities(vx, vy, omega);
        
        // Apply 8-factor scaling: scale each wheel based on its direction
        // Scaling logic: If wheel velocity is positive, use POSITIVE scale; if negative, use NEGATIVE scale
        double fl = ideal.frontLeft >= 0 ? 
            ideal.frontLeft * CalibrationCoefficients.WHEEL_VELOCITY_SCALE_FL_POSITIVE : 
            ideal.frontLeft * CalibrationCoefficients.WHEEL_VELOCITY_SCALE_FL_NEGATIVE;
            
        double fr = ideal.frontRight >= 0 ? 
            ideal.frontRight * CalibrationCoefficients.WHEEL_VELOCITY_SCALE_FR_POSITIVE : 
            ideal.frontRight * CalibrationCoefficients.WHEEL_VELOCITY_SCALE_FR_NEGATIVE;
            
        double bl = ideal.backLeft >= 0 ? 
            ideal.backLeft * CalibrationCoefficients.WHEEL_VELOCITY_SCALE_BL_POSITIVE : 
            ideal.backLeft * CalibrationCoefficients.WHEEL_VELOCITY_SCALE_BL_NEGATIVE;
            
        double br = ideal.backRight >= 0 ? 
            ideal.backRight * CalibrationCoefficients.WHEEL_VELOCITY_SCALE_BR_POSITIVE : 
            ideal.backRight * CalibrationCoefficients.WHEEL_VELOCITY_SCALE_BR_NEGATIVE;
        
        return new WheelVelocities(fl, fr, bl, br);
    }
    
    /**
     * Converts desired robot velocity to wheel velocities and normalizes to motor power range.
     * 
     * @param vx Forward velocity (inches/sec)
     * @param vy Strafe velocity (inches/sec)
     * @param omega Rotational velocity (degrees/sec)
     * @return Normalized wheel powers in range [-1.0, 1.0]
     */
    public static WheelVelocities robotVelocitiesToWheelPowers(double vx, double vy, double omega) {
        WheelVelocities wheelVels = robotVelocitiesToWheelVelocities(vx, vy, omega);
        return normalizeWheelVelocities(wheelVels);
    }
    
    // ========== INVERSE KINEMATICS ==========
    
    /**
     * Converts individual wheel velocities to robot velocity.
     * Useful for odometry calculations or verification.
     * 
     * @param wheelVels Wheel velocities (inches/sec)
     * @return Robot velocity (vx, vy, omega)
     */
    public static Velocity2D wheelVelocitiesToRobotVelocities(WheelVelocities wheelVels) {
        // Inverse mecanum kinematics
        // vx = (FL + FR + BL + BR) / 4
        // vy = (FL - FR - BL + BR) / 4
        // omega = (FL - FR + BL - BR) / (4 * turningRadius)
        
        double vx = (wheelVels.frontLeft + wheelVels.frontRight + 
                     wheelVels.backLeft + wheelVels.backRight) / 4.0;
        
        double vy = (wheelVels.frontLeft - wheelVels.frontRight - 
                     wheelVels.backLeft + wheelVels.backRight) / 4.0;
        
        double turningRadius = Math.sqrt(
            Math.pow(MotionConfig.TRACK_WIDTH / 2.0, 2) + 
            Math.pow(MotionConfig.WHEELBASE / 2.0, 2)
        );
        double omegaRad = (wheelVels.frontLeft - wheelVels.frontRight + 
                          wheelVels.backLeft - wheelVels.backRight) / (4.0 * turningRadius);
        // Convert from radians/sec to degrees/sec
        double omega = Math.toDegrees(omegaRad);
        
        return new Velocity2D(vx, vy, omega);
    }
    
    // ========== COORDINATE TRANSFORMATIONS ==========
    
    /**
     * Transforms velocity from robot frame to field frame.
     * 
     * Robot frame: X=forward, Y=left relative to robot
     * Field frame: X and Y fixed to field coordinates
     * 
     * @param vx_robot Velocity in robot X direction (inches/sec)
     * @param vy_robot Velocity in robot Y direction (inches/sec)
     * @param robotHeading Current robot heading - the angle the robot's forward direction makes with the field frame (degrees, 0 = facing +X field axis)
     * @return Velocity in field frame
     */
    public static Velocity2D robotToFieldFrame(double vx_robot, double vy_robot, double robotHeading) {
        // Rotation matrix transformation (convert heading to radians for trig functions)
        double headingRad = Math.toRadians(robotHeading);
        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);
        
        double vx_field = vx_robot * cos - vy_robot * sin;
        double vy_field = vx_robot * sin + vy_robot * cos;
        
        // Angular velocity is the same in both frames
        return new Velocity2D(vx_field, vy_field, 0);
    }
    
    /**
     * Transforms velocity from field frame to robot frame.
     * 
     * @param vx_field Velocity in field X direction (inches/sec)
     * @param vy_field Velocity in field Y direction (inches/sec)
     * @param robotHeading Current robot heading - the angle the robot's forward direction makes with the field frame (degrees)
     * @return Velocity in robot frame
     */
    public static Velocity2D fieldToRobotFrame(double vx_field, double vy_field, double robotHeading) {
        // Inverse rotation matrix transformation (convert heading to radians for trig functions)
        double headingRad = Math.toRadians(-robotHeading);
        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);
        
        double vx_robot = vx_field * cos - vy_field * sin;
        double vy_robot = vx_field * sin + vy_field * cos;
        
        return new Velocity2D(vx_robot, vy_robot, 0);
    }
    
    // ========== NORMALIZATION & SCALING ==========
    
    /**
     * Normalizes wheel velocities to fit within motor power range [-1.0, 1.0]
     * while maintaining the ratio between wheels.
     * 
     * @param wheelVels Wheel velocities (any units)
     * @return Normalized wheel powers in range [-1.0, 1.0]
     */
    public static WheelVelocities normalizeWheelVelocities(WheelVelocities wheelVels) {
        // Find maximum absolute value
        double max = Math.max(
            Math.max(Math.abs(wheelVels.frontLeft), Math.abs(wheelVels.frontRight)),
            Math.max(Math.abs(wheelVels.backLeft), Math.abs(wheelVels.backRight))
        );
        
        // If already within range, return as-is
        if (max <= 1.0) {
            return wheelVels;
        }
        
        // Scale all wheels by the same factor to maintain ratios
        return new WheelVelocities(
            wheelVels.frontLeft / max,
            wheelVels.frontRight / max,
            wheelVels.backLeft / max,
            wheelVels.backRight / max
        );
    }
    
    /**
     * Scales wheel velocities by a constant factor.
     * 
     * Use cases:
     * - Slowing down all wheels proportionally for precise movements (scale < 1.0)
     * - Implementing velocity ramps during acceleration/deceleration profiles
     * - Adjusting speed based on battery voltage or other dynamic factors
     * - Fine-tuning motion speed while maintaining direction and trajectory
     * 
     * Unlike normalizeWheelVelocities(), this maintains absolute velocity relationships
     * rather than just ratios, and does not clamp to [-1.0, 1.0] range.
     * 
     * @param wheelVels Wheel velocities (any units)
     * @param scale Scale factor (e.g., 0.5 = half speed, 2.0 = double speed)
     * @return Scaled wheel velocities
     */
    public static WheelVelocities scaleWheelVelocities(WheelVelocities wheelVels, double scale) {
        return new WheelVelocities(
            wheelVels.frontLeft * scale,
            wheelVels.frontRight * scale,
            wheelVels.backLeft * scale,
            wheelVels.backRight * scale
        );
    }
    
    /**
     * Clamps wheel velocities to maximum allowed value.
     * 
     * @param wheelVels Wheel velocities
     * @param maxVelocity Maximum allowed velocity (absolute value)
     * @return Clamped wheel velocities
     */
    public static WheelVelocities clampWheelVelocities(WheelVelocities wheelVels, double maxVelocity) {
        return new WheelVelocities(
            Math.max(-maxVelocity, Math.min(maxVelocity, wheelVels.frontLeft)),
            Math.max(-maxVelocity, Math.min(maxVelocity, wheelVels.frontRight)),
            Math.max(-maxVelocity, Math.min(maxVelocity, wheelVels.backLeft)),
            Math.max(-maxVelocity, Math.min(maxVelocity, wheelVels.backRight))
        );
    }
    
    // ========== CONVERSION UTILITIES ==========
    
    /**
     * Converts wheel velocity in inches/sec to motor encoder ticks/sec.
     * 
     * @param velocityInchesPerSec Wheel velocity (inches/sec)
     * @return Encoder velocity (ticks/sec)
     */
    public static double velocityToTicks(double velocityInchesPerSec) {
        return velocityInchesPerSec * MotionConfig.COUNTS_PER_INCH;
    }
    
    /**
     * Converts encoder ticks/sec to wheel velocity in inches/sec.
     * 
     * @param ticksPerSec Encoder velocity (ticks/sec)
     * @return Wheel velocity (inches/sec)
     */
    public static double ticksToVelocity(double ticksPerSec) {
        return ticksPerSec / MotionConfig.COUNTS_PER_INCH;
    }
    
    /**
     * Converts encoder ticks to calibrated distance in inches.
     * Applies real-world calibration factors for accurate distance measurement.
     * 
     * @param ticks Encoder ticks
     * @param isStrafe True for strafe movement, false for forward/backward
     * @return Calibrated distance (inches)
     */
    public static double ticksToDistance(double ticks, boolean isStrafe) {
        double theoretical = ticks / MotionConfig.COUNTS_PER_INCH;
        return theoretical * (isStrafe ? 
            CalibrationCoefficients.ODOMETRY_Y_SCALE : 
            CalibrationCoefficients.ODOMETRY_X_SCALE);
    }
    
    /**
     * Converts encoder ticks/sec to calibrated velocity in inches/sec.
     * Applies real-world calibration factors for accurate velocity measurement.
     * 
     * @param ticksPerSec Encoder velocity (ticks/sec)
     * @param isStrafe True for strafe movement, false for forward/backward
     * @return Calibrated velocity (inches/sec)
     */
    public static double ticksToVelocity(double ticksPerSec, boolean isStrafe) {
        double theoretical = ticksPerSec / MotionConfig.COUNTS_PER_INCH;
        return theoretical * (isStrafe ? 
            CalibrationCoefficients.ODOMETRY_Y_SCALE : 
            CalibrationCoefficients.ODOMETRY_X_SCALE);
    }
    
    /**
     * Converts distance in inches to encoder ticks with calibration.
     * 
     * @param distance Distance (inches)
     * @param isStrafe True for strafe movement, false for forward/backward
     * @return Encoder ticks
     */
    public static double distanceToTicks(double distance, boolean isStrafe) {
        double calibratedDistance = distance / (isStrafe ? 
            CalibrationCoefficients.ODOMETRY_Y_SCALE : 
            CalibrationCoefficients.ODOMETRY_X_SCALE);
        return calibratedDistance * MotionConfig.COUNTS_PER_INCH;
    }
    
    /**
     * Converts all wheel velocities to encoder ticks/sec.
     * 
     * @param wheelVels Wheel velocities (inches/sec)
     * @return Wheel velocities (ticks/sec)
     */
    public static WheelVelocities wheelVelocitiesToTicks(WheelVelocities wheelVels) {
        return new WheelVelocities(
            velocityToTicks(wheelVels.frontLeft),
            velocityToTicks(wheelVels.frontRight),
            velocityToTicks(wheelVels.backLeft),
            velocityToTicks(wheelVels.backRight)
        );
    }
    
    // ========== MOTION CALCULATIONS ==========
    
    /**
     * Calculates the required velocity for circular motion around a point.
     * 
     * @param centerX Center point X coordinate (inches)
     * @param centerY Center point Y coordinate (inches)
     * @param robotX Current robot X position (inches)
     * @param robotY Current robot Y position (inches)
     * @param angularVelocity Angular velocity around center (degrees/sec)
     * @return Velocity vector for circular motion
     */
    public static Velocity2D circularMotionVelocity(double centerX, double centerY, 
                                                    double robotX, double robotY, 
                                                    double angularVelocity) {
        // Vector from center to robot
        double dx = robotX - centerX;
        double dy = robotY - centerY;
        double radius = Math.hypot(dx, dy);
        
        // Special case: center at robot position = pure rotation
        if (radius < 0.01) {  // Less than 0.01 inch (effectively zero)
            return new Velocity2D(0, 0, angularVelocity);
        }
        
        // Tangent velocity magnitude (convert angular velocity to radians for calculation)
        double angularVelocityRad = Math.toRadians(angularVelocity);
        double tangentSpeed = angularVelocityRad * radius;
        
        // Tangent direction (perpendicular to radius)
        // For counter-clockwise motion: tangent = (-dy, dx) / radius
        double vx = -tangentSpeed * dy / radius;
        double vy = tangentSpeed * dx / radius;
        
        return new Velocity2D(vx, vy, angularVelocity);
    }
    
    /**
     * Calculates angle from one point to another.
     * 
     * @param fromX Start point X
     * @param fromY Start point Y
     * @param toX End point X
     * @param toY End point Y
     * @return Angle in degrees
     */
    public static double angleBetweenPoints(double fromX, double fromY, double toX, double toY) {
        return Math.toDegrees(Math.atan2(toY - fromY, toX - fromX));
    }
    
    /**
     * Calculates distance between two points.
     * 
     * @param x1 First point X
     * @param y1 First point Y
     * @param x2 Second point X
     * @param y2 Second point Y
     * @return Distance in inches
     */
    public static double distanceBetweenPoints(double x1, double y1, double x2, double y2) {
        return Math.hypot(x2 - x1, y2 - y1);
    }
    
    /**
     * Normalizes an angle to the range [-180, 180].
     * 
     * @param angle Angle in degrees
     * @return Normalized angle in range [-180, 180]
     */
    public static double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
    
    // Prevent instantiation
    /**
     * Checks if a position error is within tolerance
     * @param error Position error in inches
     * @return true if within tolerance
     */
    public static boolean isPositionWithinTolerance(double error) {
        return Math.abs(error) <= MotionConfig.POSITION_TOLERANCE;
    }
    
    /**
     * Checks if a heading error is within tolerance
     * @param error Heading error in degrees
     * @return true if within tolerance
     */
    public static boolean isHeadingWithinTolerance(double error) {
        return Math.abs(error) <= MotionConfig.HEADING_TOLERANCE;
    }
    
    // ========== CALIBRATED KINEMATICS ==========
    
    /**
     * Forward kinematics using calibrated 3×4 matrix
     * Converts wheel velocities to actual robot motion
     * 
     * @param wheelVels Wheel velocities (inches/sec)
     * @return Actual robot velocity accounting for cross-coupling effects
     */
    public static Velocity2D wheelVelocitiesToRobotVelocitiesCalibrated(WheelVelocities wheelVels) {
        if (!CalibrationCoefficients.KINEMATIC_MATRIX_CALIBRATED) {
            // Fall back to ideal kinematics if not calibrated
            return wheelVelocitiesToRobotVelocities(wheelVels);
        }
        
        double[][] K = CalibrationCoefficients.getEffectiveKinematicMatrix();
        double[] wheels = {wheelVels.frontLeft, wheelVels.frontRight, 
                          wheelVels.backLeft, wheelVels.backRight};
        
        // Matrix multiplication: [vx, vy, ω] = K × [wheel_velocities]
        double vx = K[0][0]*wheels[0] + K[0][1]*wheels[1] + K[0][2]*wheels[2] + K[0][3]*wheels[3];
        double vy = K[1][0]*wheels[0] + K[1][1]*wheels[1] + K[1][2]*wheels[2] + K[1][3]*wheels[3];
        double omega = K[2][0]*wheels[0] + K[2][1]*wheels[1] + K[2][2]*wheels[2] + K[2][3]*wheels[3];
        
        return new Velocity2D(vx, vy, omega);
    }
    
    /**
     * Inverse kinematics using calibrated matrix (iterative approach)
     * Converts desired robot motion to required wheel velocities
     * 
     * @param vx Desired forward velocity (inches/sec)
     * @param vy Desired strafe velocity (inches/sec)
     * @param omega Desired angular velocity (degrees/sec)
     * @return Required wheel velocities accounting for calibration
     */
    public static WheelVelocities robotVelocitiesToWheelVelocitiesCalibrated(double vx, double vy, double omega) {
        if (!CalibrationCoefficients.KINEMATIC_MATRIX_CALIBRATED) {
            // Fall back to ideal kinematics if not calibrated
            return robotVelocitiesToWheelVelocities(vx, vy, omega);
        }
        
        // Use iterative correction approach for stability
        return robotVelocitiesToWheelVelocitiesIterative(vx, vy, omega);
    }
    
    /**
     * Iterative approach for inverse calibrated kinematics
     * More stable than direct pseudo-inverse for real-time use
     * 
     * @param vx_desired Desired forward velocity (inches/sec)
     * @param vy_desired Desired strafe velocity (inches/sec)
     * @param omega_desired Desired angular velocity (degrees/sec)
     * @return Corrected wheel velocities
     */
    private static WheelVelocities robotVelocitiesToWheelVelocitiesIterative(double vx_desired, double vy_desired, double omega_desired) {
        // Start with ideal kinematics
        WheelVelocities wheelVels = robotVelocitiesToWheelVelocities(vx_desired, vy_desired, omega_desired);
        
        // Iterative correction (2-3 iterations usually sufficient)
        for (int iter = 0; iter < 3; iter++) {
            // Predict actual motion using calibrated forward kinematics (direct matrix multiplication)
            double[][] K = CalibrationCoefficients.getEffectiveKinematicMatrix();
            double[] wheels = {wheelVels.frontLeft, wheelVels.frontRight, 
                              wheelVels.backLeft, wheelVels.backRight};
            
            double vx_pred = K[0][0]*wheels[0] + K[0][1]*wheels[1] + K[0][2]*wheels[2] + K[0][3]*wheels[3];
            double vy_pred = K[1][0]*wheels[0] + K[1][1]*wheels[1] + K[1][2]*wheels[2] + K[1][3]*wheels[3];
            double omega_pred = K[2][0]*wheels[0] + K[2][1]*wheels[1] + K[2][2]*wheels[2] + K[2][3]*wheels[3];
            
            Velocity2D predictedMotion = new Velocity2D(vx_pred, vy_pred, omega_pred);
            
            // Calculate error
            double vx_error = vx_desired - predictedMotion.vx;
            double vy_error = vy_desired - predictedMotion.vy;
            double omega_error = omega_desired - predictedMotion.omega;
            
            // Apply correction with damping
            double damping = 0.7; // Prevents oscillation
            WheelVelocities correction = robotVelocitiesToWheelVelocities(
                vx_error * damping, vy_error * damping, omega_error * damping);
            
            // Update wheel velocities
            wheelVels = new WheelVelocities(
                wheelVels.frontLeft + correction.frontLeft,
                wheelVels.frontRight + correction.frontRight,
                wheelVels.backLeft + correction.backLeft,
                wheelVels.backRight + correction.backRight
            );
            
            // Check convergence
            if (Math.abs(vx_error) < 0.1 && Math.abs(vy_error) < 0.1 && Math.abs(omega_error) < 1.0) {
                break;
            }
        }
        
        return wheelVels;
    }

    private MecanumKinematics() {}
}

