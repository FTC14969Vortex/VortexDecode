package org.firstinspires.ftc.teamcode.motion;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.external.gobilida.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.calibration.RobotConstants.ComponentPosition;
import org.firstinspires.ftc.teamcode.calibration.CalibrationCoefficients;
import org.firstinspires.ftc.teamcode.motion.FieldPose;
import org.firstinspires.ftc.teamcode.motion.MotionState;
import org.firstinspires.ftc.teamcode.motion.OdometryManager;


/**
 * Control modes for position control
 */
enum PositionControlMode {
    PURE_FEEDBACK,    // PID only
    PURE_FEEDFORWARD, // Feedforward only
    HYBRID           // Feedforward + feedback blend
}


/**
 * Motion command executor with real-time control.
 *
 * Layer 2: Motion command execution
 *
 * Responsibilities:
 * - Execute motion primitives (linear, circular, combined)
 * - Velocity profile generation (trapezoidal)
 * - Closed-loop control using odometry feedback
 * - Coordinate frame transformations
 * - Safety monitoring (timeouts, stalls)
 *
 * This class manages hardware and implements the control loops.
 *
 * ========== MOTION PRIMITIVES SUMMARY ==========
 *
 * 1. linearMove(angle, distance, [targetHeading], velocity, [coordinateMode])
 *    - Moves robot in straight line at specified angle for specified distance
 *    - Uses position-based control to guarantee reaching target coordinates
 *    - Optional targetHeading: rotate to target heading while moving (default: maintain current heading)
 *    - Default coordinate mode: ROBOT_CENTRIC (angle relative to robot's current heading)
 *    - Can specify FIELD_CENTRIC for absolute field angles if needed
 *    - Internally delegates to moveToPose for accurate endpoint positioning
 *    - Use for: straight-line navigation, diagonal moves, approach sequences
 *
 * 2. moveToPose(targetX, targetY, targetHeading, velocity)
 *    - Moves robot to absolute field position with target heading
 *    - Always operates in field-centric coordinates (relative to odometry origin)
 *    - Simultaneously controls position (X, Y) and heading using independent P-controllers
 *    - Self-correcting: continuously adjusts path toward target
 *    - Robust to calibration errors and disturbances
 *    - Use for: point-to-point navigation, scoring positions, waypoint following
 *
 * 3. circularMove(centerX, centerY, angularVelocity, totalAngle, coordinateMode)
 *    - Moves robot in circular arc around a center point
 *    - Robot maintains tangent velocity to circle while traversing arc
 *    - Special case: radius=0 (at center) automatically becomes spin-in-place
 *    - Useful for smooth curved paths and circular trajectories
 *    - Use for: curved approaches, orbiting points, smooth path transitions
 *
 * 4. combinedMotion(vx, vy, omega, coordinateMode, maxDuration, maxDistance, stopAtTarget, customCondition)
 *    - Most flexible primitive: arbitrary velocity commands with multiple termination conditions
 *    - Supports time-based, distance-based, target-based, or custom termination
 *    - Direct velocity control without trajectory planning
 *    - Use for: custom motion profiles, sensor-driven navigation, complex behaviors
 *
 * 5. setVelocity(vx, vy, omega, coordinateMode)
 *    - Direct velocity control (open-loop at high level, closed-loop at motor level)
 *    - Continuous motion until explicitly stopped
 *    - Accepts both robot-centric and field-centric velocity commands
 *    - Use for: TeleOp driving, real-time control, manual navigation
 *
 * All motion primitives use power-based control for consistent motion across different
 * robot configurations and battery voltage levels.
 */
public class MotionExecutor {

    /**
     * Control mode enumeration for motion control system
     */
    public enum ControlMode {
        /** Pure feedforward control - open loop, predictable timing */
        PURE_FEEDFORWARD,

        /** Pure feedback (PID) control - closed loop, disturbance rejection */
        PURE_FEEDBACK,

        /** Hybrid control - intelligent switching between feedforward and feedback (default) */
        HYBRID
    }

    // ========== HARDWARE REFERENCES ==========

    private final DcMotorEx frontLeftMotor;
    private final DcMotorEx frontRightMotor;
    private final DcMotorEx backLeftMotor;
    private final DcMotorEx backRightMotor;
    private final GoBildaPinpointDriver odometry;

    // ========== STATE MANAGEMENT ==========

    private MotionState motionState;

    private final ElapsedTime timer;
    private final ElapsedTime stallTimer;

    // ========== CONTROLLERS ==========


    private OdometryManager odometryManager;

    // ========== COORDINATE SYSTEM ==========

    private CoordinateTransformer coordinateTransformer;

    // ========== HARDWARE ABSTRACTION ==========

    private DriveHardware driveHardware;

    // ========== CONTROL MODE PARAMETERS ==========

    private PositionControlMode positionControlMode = PositionControlMode.PURE_FEEDBACK;
    private double positionFeedforwardGain = MotionConfig.POSITION_FEEDFORWARD_GAIN;

    private double accelerationLimit = MotionConfig.ACCELERATION_LIMIT;


    // ========== CONTROL MODE MANAGEMENT ==========

    private ControlMode currentControlMode = ControlMode.HYBRID;
    private ControlMode defaultControlMode = ControlMode.HYBRID;

    // ========== VELOCITY RAMPING ==========

    private double previousVx = 0.0;
    private double previousVy = 0.0;
    private double previousVAngular = 0.0;
    private long lastControlTime = 0;

    // ========== EXECUTION STATUS ==========

    private boolean isExecuting = false;
    private double lastPositionError = 0.0;

    /**
     * Result of a motion command execution
     */
    public static class MotionResult {
        public boolean success;
        public double finalPositionError;
        public double finalHeadingError;
        public double executionTimeMs;
        public String failureReason;

        public MotionResult(boolean success, double posError, double hdgError, double timeMs, String reason) {
            this.success = success;
            this.finalPositionError = posError;
            this.finalHeadingError = hdgError;
            this.executionTimeMs = timeMs;
            this.failureReason = reason;
        }

        @Override
        public String toString() {
            if (success) {
                return String.format("SUCCESS (%.2fs, pos_err=%.2f\", hdg_err=%.1f degrees)",
                        executionTimeMs/1000.0, finalPositionError, finalHeadingError);
            } else {
                return String.format("FAILED: %s (%.2fs)", failureReason, executionTimeMs/1000.0);
            }
        }
    }

    // ========== CONSTRUCTOR ==========

    /**
     * Creates a new MotionExecutor
     *
     * @param frontLeft Front left motor (DcMotorEx)
     * @param frontRight Front right motor (DcMotorEx)
     * @param backLeft Back left motor (DcMotorEx)
     * @param backRight Back right motor (DcMotorEx)
     * @param odometry Odometry system (GoBilda Pinpoint)
     */
    public MotionExecutor(DcMotorEx frontLeft, DcMotorEx frontRight,
                          DcMotorEx backLeft, DcMotorEx backRight,
                          GoBildaPinpointDriver odometry) {
        this.frontLeftMotor = frontLeft;
        this.frontRightMotor = frontRight;
        this.backLeftMotor = backLeft;
        this.backRightMotor = backRight;
        this.odometry = odometry;

        // Create OdometryManager for pose tracking
        this.odometryManager = new OdometryManager(odometry);


        // Create MotionState for state tracking with odometry integration
        this.motionState = new MotionState(this.odometryManager);

        // Create CoordinateTransformer for coordinate system management
        this.coordinateTransformer = new CoordinateTransformer(this.odometryManager, this.motionState);

        // Create DriveHardware for hardware abstraction
        this.driveHardware = new DriveHardware(frontLeft, frontRight, backLeft, backRight, odometry);


        this.timer = new ElapsedTime();
        this.stallTimer = new ElapsedTime();

    }

    /**
     * Initialize PID controllers and velocity profiler
     * Uses factory methods to ensure proper velocity-based configuration
     */

    /**
     * Configure motor settings
     */

    // ========== INITIALIZATION & STATE MANAGEMENT ==========

    /**
     * Gets the current motion state
     * @return Current motion state
     */
    public MotionState getMotionState() {
        return motionState;
    }

    /**
     * Gets the coordinate transformer
     * @return Coordinate transformer for coordinate system conversions
     */
    public CoordinateTransformer getCoordinateTransformer() {
        return coordinateTransformer;
    }

    /**
     * Updates motion state from calibrated odometry
     */
    public void updateState() {
        odometryManager.update();
        motionState.updateCurrentPose(odometryManager.getCurrentPose());
    }

    /**
     * Checks if currently executing a motion command
     * @return true if executing
     */
    public boolean isExecuting() {
        return isExecuting;
    }

    // ========== LOW-LEVEL MOTOR CONTROL ==========

    /**
     * Sets motor velocities from wheel velocities object (velocity-based control)
     *
     * Uses direct velocity control for precise motion. Velocities are in ticks/sec
     * and are handled by the motor controller's internal PID loop.
     *
     * @param wheelVelocities Wheel velocities in ticks/sec
     */
    private void setMotorVelocitiesFromWheels(MecanumKinematics.WheelVelocities wheelVelocities) {
        driveHardware.setWheelVelocities(wheelVelocities);
    }

    /**
     * Stops all motors
     */
    public void stop() {
        driveHardware.stop();
        motionState.stopMotion();
        isExecuting = false;
    }

    // ========== DIRECT VELOCITY CONTROL ==========

    /**
     * Sets robot velocity directly (power-based control)
     *
     * Converts velocity commands to motor powers for consistent motion control.
     * Velocities are normalized based on maximum configured velocities.
     *
     * @param vx Velocity in X direction (inches/sec, forward/backward: + = forward, - = backward)
     * @param vy Velocity in Y direction (inches/sec, strafe left/right: + = left, - = right)
     * @param omega Angular velocity (degrees/sec, rotation: + = counter-clockwise, - = clockwise)
     * @param coordinateMode ROBOT_CENTRIC or FIELD_CENTRIC
     */
    public void setVelocity(double vx, double vy, double omega, MotionState.CoordinateMode coordinateMode) {
        updateState();

        // Transform to robot frame if needed
        MecanumKinematics.Velocity2D robotVel;
        if (coordinateMode == MotionState.CoordinateMode.FIELD_CENTRIC) {
            robotVel = MecanumKinematics.fieldToRobotFrame(
                    vx, vy, motionState.getHeading()
            );
        } else {
            robotVel = new MecanumKinematics.Velocity2D(vx, vy, omega);
        }
        double vx_robot = robotVel.vx;
        double vy_robot = robotVel.vy;

        // Calculate wheel velocities (inches/sec)
        // Choose calibration method based on configuration
        MecanumKinematics.WheelVelocities wheelVelocities;

        if (CalibrationCoefficients.USE_8_FACTOR_VELOCITY_SCALING) {
            // Use 8-factor scaling (per-wheel, per-direction compensation)
            wheelVelocities = MecanumKinematics.robotVelocitiesToWheelVelocities8Factor(vx_robot, vy_robot, omega);
        } else if (CalibrationCoefficients.KINEMATIC_MATRIX_CALIBRATED) {
            // Use full kinematic matrix calibration (cross-coupling correction)
            wheelVelocities = MecanumKinematics.robotVelocitiesToWheelVelocitiesCalibrated(vx_robot, vy_robot, omega);
        } else {
            // Use ideal kinematics (no calibration)
            wheelVelocities = MecanumKinematics.robotVelocitiesToWheelVelocities(vx_robot, vy_robot, omega);
        }

        // Convert velocities to ticks/sec for motor velocity control
        MecanumKinematics.WheelVelocities wheelVelocitiesTicks =
                MecanumKinematics.wheelVelocitiesToTicks(wheelVelocities);


        // Apply to motors using velocity control
        setMotorVelocitiesFromWheels(wheelVelocitiesTicks);

        // Update state
        motionState.setVelocity(vx, vy, omega);
        motionState.setCoordinateMode(coordinateMode);
    }

    // ========== CORE MOTION PRIMITIVES ==========
    // These are the primary motion commands you'll use most often.
    // Each has multiple overloaded variants for convenience.
    // Organized by motion type: linearMove, circularMove, moveToPose, combinedMotion, rotate

    // ========== LINEAR MOTION FAMILY ==========
    // Moves robot in straight lines at specified angles    

    public MotionResult linearMove(double angle, double distance) {
        return linearMove(angle, distance, MotionConfig.MAX_LINEAR_VELOCITY, MotionState.CoordinateMode.ROBOT_CENTRIC);
    }

    public MotionResult linearMove(double angle, double distance, double maxVelocity) {
        return linearMove(angle, distance, maxVelocity, MotionState.CoordinateMode.ROBOT_CENTRIC);
    }

    public MotionResult linearMove(double angle, double distance, double maxVelocity,
                                   MotionState.CoordinateMode coordinateMode) {
        // Get current heading to maintain it
        updateState();
        double currentHeading = motionState.getHeading();
        return linearMove(angle, distance, currentHeading, maxVelocity, 0.0, coordinateMode);
    }

    public MotionResult linearMove(double angle, double distance, double targetHeading,
                                   double maxVelocity) {
        return linearMove(angle, distance, targetHeading, maxVelocity, MotionState.CoordinateMode.ROBOT_CENTRIC);
    }

    public MotionResult linearMove(double angle, double distance, double targetHeading,
                                   double maxVelocity, MotionState.CoordinateMode coordinateMode) {
        return linearMove(angle, distance, targetHeading, maxVelocity, 0.0, coordinateMode);
    }

    /**
     * Moves robot in straight line at specified angle and distance with target heading
     *
     * Uses position-based control (like moveToPose) to guarantee reaching the target position.
     * Continuously moves toward calculated target point while simultaneously rotating to target heading.
     * This approach is robust to calibration errors and provides accurate endpoint positioning.
     *
     * @param angle Direction to move (degrees)
     *              ROBOT_CENTRIC: 0=forward, 90=left, 180=back, -90=right (relative to robot)
     *              FIELD_CENTRIC: absolute field angle (0=+X axis, 90=+Y axis)
     * @param distance Distance to travel (inches)
     * @param targetHeading Target heading to rotate to during motion (degrees, field-absolute)
     * @param maxVelocity Maximum velocity (inches/sec)
     * @param acceleration Maximum acceleration (inches/sec^2) - currently unused, reserved for trapezoidal profile
     * @param coordinateMode ROBOT_CENTRIC or FIELD_CENTRIC (determines angle interpretation)
     * @return MotionResult with execution status
     */
    public MotionResult linearMove(double angle, double distance, double targetHeading,
                                   double maxVelocity, double acceleration,
                                   MotionState.CoordinateMode coordinateMode) {
        // Validate inputs
        if (distance < 0) {
            return new MotionResult(false, 0, 0, 0, "Distance must be positive");
        }
        if (maxVelocity <= 0 || maxVelocity > MotionConfig.MAX_LINEAR_VELOCITY) {
            return new MotionResult(false, 0, 0, 0, "Invalid velocity");
        }

        // Get starting position and heading
        updateState();
        double startX = motionState.getX();
        double startY = motionState.getY();
        double startHeading = motionState.getHeading();

        // Calculate target position based on angle, distance, and coordinate mode
        double moveAngle;
        if (coordinateMode == MotionState.CoordinateMode.ROBOT_CENTRIC) {
            // Robot-centric: angle is relative to current heading
            // Add current heading to get absolute field angle
            moveAngle = startHeading + angle;
        } else {
            // Field-centric: angle is already absolute field direction
            moveAngle = angle;
        }

        // Calculate target position in field coordinates
        double angleRad = Math.toRadians(moveAngle);
        double targetX = startX + distance * Math.cos(angleRad);
        double targetY = startY + distance * Math.sin(angleRad);

        // Use moveToPose to execute - it provides position-based control
        // This guarantees we reach the target position (X, Y) accurately
        // while simultaneously rotating to the target heading
        // Note: moveToPose always operates in field-centric coordinates
        // Calculate estimated timeout: distance/velocity * safety factor (with margin for acceleration/deceleration)
        int estimatedTimeoutMs = (int) Math.max((distance / maxVelocity) * MotionConfig.TIMEOUT_SAFETY_FACTOR * 1000, MotionConfig.MOTION_TIMEOUT_MS);
        return moveToPose(targetX, targetY, targetHeading, maxVelocity, acceleration, estimatedTimeoutMs);
    }

    /**
     * Linear move from CURRENT reference point position to target pose (with default velocity)
     * Uses current odometry reading as start position
     * @param targetPose Target position for the reference point
     */
    public MotionResult linearMove(FieldPose targetPose) {
        return linearMove(targetPose, MotionConfig.MAX_LINEAR_VELOCITY);
    }
    /**
     * Linear move from CURRENT reference point position to target pose
     * Uses current odometry reading as start position
     * @param targetPose Target position for the reference point
     * @param velocity Movement velocity (inches/sec)
     */
    public MotionResult linearMove(FieldPose targetPose, double velocity) {
        // Get current reference point position from odometry
        updateState();
        // motionState.getX(), getY(), getHeading() return REFERENCE POINT coordinates from odometry
        FieldPose currentRefPoint = coordinateTransformer.getCurrentReferencePointPose();

        // Calculate angle and distance from current position to target
        double deltaX = targetPose.x - currentRefPoint.x;
        double deltaY = targetPose.y - currentRefPoint.y;
        double angle = Math.toDegrees(Math.atan2(deltaY, deltaX));
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        return linearMove(angle, distance, targetPose.heading, velocity, MotionState.CoordinateMode.FIELD_CENTRIC);
    }

    // ========== CIRCULAR MOTION FAMILY ==========
    // Moves robot in circular arcs around center points


    /**
     * Moves robot in circular arc around a center point (with default angular velocity)
     *
     * Robot follows circular path while maintaining tangent velocity.
     * Uses default angular velocity from MotionConfig.
     *
     * @param centerX Center point X coordinate (inches)
     * @param centerY Center point Y coordinate (inches)
     * @param totalAngle Total angle to traverse (degrees, positive = counter-clockwise)
     * @param coordinateMode ROBOT_CENTRIC or FIELD_CENTRIC
     * @return MotionResult with execution status
     */
    public MotionResult circularMove(double centerX, double centerY, double totalAngle,
                                     MotionState.CoordinateMode coordinateMode) {
        return circularMove(centerX, centerY, MotionConfig.MAX_ANGULAR_VELOCITY, totalAngle, coordinateMode);
    }
    /**
     * Moves robot in circular arc around a center point
     *
     * Robot follows circular path while maintaining tangent velocity.
     * Useful for smooth curved paths and circular trajectories.
     *
     * @param centerX Center point X coordinate (inches)
     * @param centerY Center point Y coordinate (inches)
     * @param angularVelocity Angular velocity around center (degrees/sec, positive = counter-clockwise)
     * @param totalAngle Total angle to traverse (degrees, positive = counter-clockwise)
     * @param acceleration Maximum acceleration (inches/sec^2) - currently unused, reserved for future
     * @param coordinateMode ROBOT_CENTRIC or FIELD_CENTRIC
     * @return MotionResult with execution status
     */
    public MotionResult circularMove(double centerX, double centerY, double angularVelocity,
                                     double totalAngle, MotionState.CoordinateMode coordinateMode) {
        return circularMove(centerX, centerY, angularVelocity, totalAngle, 0.0, coordinateMode);
    }

    public MotionResult circularMove(double centerX, double centerY, double angularVelocity,
                                     double totalAngle, double acceleration,
                                     MotionState.CoordinateMode coordinateMode) {
        // Validate inputs
        if (Math.abs(angularVelocity) > MotionConfig.MAX_ANGULAR_VELOCITY) {
            return new MotionResult(false, 0, 0, 0, "Angular velocity exceeds maximum");
        }
        if (totalAngle == 0) {
            return new MotionResult(false, 0, 0, 0, "Total angle is zero");
        }

        // Mark as executing
        isExecuting = true;
        timer.reset();
        stallTimer.reset();

        // Update state
        updateState();

        // ROBOT CENTER MOTION CONTROL:
        // Get robot center position for efficient circular motion around robot center
        Pose2D currentRobotCenter = coordinateTransformer.getCurrentRobotCenterPose();
        double startX = currentRobotCenter.getX(DistanceUnit.INCH);
        double startY = currentRobotCenter.getY(DistanceUnit.INCH);

        // Calculate starting angle from center
        // Convert center coordinates to robot center frame for efficient motion control
        double actualCenterX, actualCenterY;
        if (coordinateMode == MotionState.CoordinateMode.ROBOT_CENTRIC) {
            // Convert robot-relative center to robot center field coordinates
            double headingRad = Math.toRadians(motionState.getHeading());
            double cos = Math.cos(headingRad);
            double sin = Math.sin(headingRad);
            // Transform center from robot frame to robot center field frame
            actualCenterX = startX + (centerX * cos - centerY * sin);
            actualCenterY = startY + (centerX * sin + centerY * cos);
        } else {
            // FIELD_CENTRIC: centerX, centerY are in reference point coordinates
            // Convert to robot center coordinates for efficient motion control
            Pose2D centerRobotCenter = coordinateTransformer.convertReferencePointToRobotCenter(
                    centerX, centerY, motionState.getHeading());
            actualCenterX = centerRobotCenter.getX(DistanceUnit.INCH);
            actualCenterY = centerRobotCenter.getY(DistanceUnit.INCH);
        }

        double dx = startX - actualCenterX;
        double dy = startY - actualCenterY;
        double startAngle = Math.toDegrees(Math.atan2(dy, dx));
        double targetAngle = startAngle + totalAngle;

        // Calculate radius
        double radius = Math.hypot(dx, dy);

        // Special case: radius=0 or very small = pure rotation (spin in place)
        if (radius < 0.1) {
            // Pure rotation - convert to heading change and use heading controller
            return rotate(totalAngle, angularVelocity);
        }

        // Track angle traversed
        double lastAngle = startAngle;
        double angleTraversed = 0.0;


        // Create PID controller for angular control in circular motion
        PIDController angularPID = PIDController.createHeadingController();
        angularPID.setSetpoint(targetAngle);
        // Motion control loop
        while (isExecuting) {
            // Check timeout
            if (timer.milliseconds() > MotionConfig.MOTION_TIMEOUT_MS) {
                stop();
                return new MotionResult(false, motionState.getPositionError(),
                        motionState.getHeadingError(), timer.milliseconds(), "Timeout");
            }

            // Update state
            updateState();

            // ROBOT CENTER MOTION CONTROL: Get current robot center position
            Pose2D currentRobotCenterPose = coordinateTransformer.getCurrentRobotCenterPose();
            double currentX = currentRobotCenterPose.getX(DistanceUnit.INCH);
            double currentY = currentRobotCenterPose.getY(DistanceUnit.INCH);

            // Calculate current angle from center
            double currentDx = currentX - actualCenterX;
            double currentDy = currentY - actualCenterY;
            double currentAngle = Math.toDegrees(Math.atan2(currentDy, currentDx));

            // Calculate angle change (handle wraparound)
            double angleDelta = currentAngle - lastAngle;
            while (angleDelta > 180) angleDelta -= 360;
            while (angleDelta < -180) angleDelta += 360;
            angleTraversed += angleDelta;
            lastAngle = currentAngle;

            // Check if completed the arc
            double angleRemaining = totalAngle - angleTraversed;
            if (Math.abs(angleRemaining) < MotionConfig.HEADING_TOLERANCE) {
                stop();
                return new MotionResult(true, 0, Math.abs(angleRemaining),
                        timer.milliseconds(), "Success");
            }

            // Check for stall (verify we're still on the circular path)
            double currentRadius = Math.hypot(currentDx, currentDy);
            if (Math.abs(currentRadius - radius) > MotionConfig.POSITION_TOLERANCE * 5) {
                if (stallTimer.milliseconds() > MotionConfig.STALL_DETECTION_TIME_MS) {
                    stop();
                    return new MotionResult(false, Math.abs(currentRadius - radius),
                            Math.abs(angleRemaining), timer.milliseconds(), "Off circular path");
                }
            } else {
                stallTimer.reset();
            }

            // Calculate velocity for circular motion using hybrid angular control
            // Use hybrid control: feedforward for large errors, PID for precision
            double adjustedAngularVelocity = calculateAngularControl(
                    currentAngle, targetAngle, angularPID
            );

            // Ensure we do not exceed the requested angular velocity magnitude
            double maxVel = Math.abs(angularVelocity);
            adjustedAngularVelocity = Math.max(-maxVel, Math.min(maxVel, adjustedAngularVelocity));

            MecanumKinematics.Velocity2D velocity = MecanumKinematics.circularMotionVelocity(
                    actualCenterX, actualCenterY, currentX, currentY, adjustedAngularVelocity
            );

            // Apply velocity (always use field-centric for circular motion)
            setVelocity(velocity.vx, velocity.vy, velocity.omega,
                    MotionState.CoordinateMode.FIELD_CENTRIC);

            // Control loop timing
            try { Thread.sleep(MotionConfig.CONTROL_LOOP_PERIOD_MS); }
            catch (InterruptedException e) { Thread.currentThread().interrupt(); }
        }

        stop();
        return new MotionResult(false, 0, Math.abs(totalAngle - angleTraversed),
                timer.milliseconds(), "Interrupted");
    }

    /**
     * Circular move with reference point following arc around center (with default angular velocity)
     * @param centerPose Center point of circular motion (reference point coordinates)
     * @param totalAngle Total angle to traverse (degrees)
     */
    public MotionResult circularMove(FieldPose centerPose, double totalAngle) {
        return circularMove(centerPose.x, centerPose.y, MotionConfig.MAX_ANGULAR_VELOCITY, totalAngle, MotionState.CoordinateMode.FIELD_CENTRIC);
    }
    /**
     * Circular move with reference point following arc around center
     * @param centerPose Center point of circular motion (reference point coordinates)
     * @param angularVelocity Angular velocity (degrees/sec)
     * @param totalAngle Total angle to traverse (degrees)
     */
    public MotionResult circularMove(FieldPose centerPose, double angularVelocity, double totalAngle) {
        return circularMove(centerPose.x, centerPose.y, angularVelocity, totalAngle, MotionState.CoordinateMode.FIELD_CENTRIC);
    }


    /**
     * Moves robot to target pose (with default velocity, acceleration and timeout)
     *
     * Always operates in field-centric coordinates (relative to odometry reset point).
     * Uses default max velocity from MotionConfig.
     *
     * @param targetX Target X position (inches, field coordinates)
     * @param targetY Target Y position (inches, field coordinates)
     * @param targetHeading Target heading (degrees, field-absolute)
     * @return MotionResult with execution status
     */

    // ========== POSE-BASED MOTION FAMILY ==========
    // Moves robot to absolute field positions with target headings

    public MotionResult moveToPose(double targetX, double targetY, double targetHeading) {
        return moveToPose(targetX, targetY, targetHeading, MotionConfig.MAX_LINEAR_VELOCITY);
    }
    /**
     * Moves robot to target pose (with default acceleration and timeout)
     *
     * Always operates in field-centric coordinates (relative to odometry reset point).
     *
     * @param targetX Target X position (inches, field coordinates)
     * @param targetY Target Y position (inches, field coordinates)
     * @param targetHeading Target heading (degrees, field-absolute)
     * @param maxVelocity Maximum linear velocity (inches/sec)
     * @return MotionResult with execution status
     */
    public MotionResult moveToPose(double targetX, double targetY, double targetHeading,
                                   double maxVelocity) {
        // Calculate distance to target for timeout estimation
        updateState();
        double currentX = motionState.getX();
        double currentY = motionState.getY();
        double distance = Math.sqrt(Math.pow(targetX - currentX, 2) + Math.pow(targetY - currentY, 2));

        // Calculate estimated timeout: distance/velocity * safety factor (with margin for acceleration/deceleration)
        int estimatedTimeoutMs = (int) Math.max((distance / maxVelocity) * MotionConfig.TIMEOUT_SAFETY_FACTOR * 1000, MotionConfig.MOTION_TIMEOUT_MS);
        return moveToPose(targetX, targetY, targetHeading, maxVelocity, 0.0, estimatedTimeoutMs);
    }

    /**
     * Moves robot to target pose (with default acceleration)
     *
     * Always operates in field-centric coordinates (relative to odometry reset point).
     *
     * @param targetX Target X position (inches, field coordinates)
     * @param targetY Target Y position (inches, field coordinates)
     * @param targetHeading Target heading (degrees, field-absolute)
     * @param maxVelocity Maximum linear velocity (inches/sec)
     * @param timeoutMs Maximum duration in milliseconds (0 = use default timeout)
     * @return MotionResult with execution status
     */
    public MotionResult moveToPose(double targetX, double targetY, double targetHeading,
                                   double maxVelocity, int timeoutMs) {
        return moveToPose(targetX, targetY, targetHeading, maxVelocity, 0.0, timeoutMs);
    }

    /**
     * Moves robot to target pose (position and heading)
     *
     * Simultaneously controls position (X, Y) and heading using independent controllers.
     * Robot follows smooth path while rotating to target heading.
     * Motion completes when both position and heading are within tolerance.
     *
     * Always operates in field-centric coordinates. The coordinate system origin is set
     * when odometry is reset (typically at robot initialization with FIELD_ORIGIN values).
     * All positions and headings are relative to that origin point.
     *
     * @param targetX Target X position (inches, field coordinates)
     * @param targetY Target Y position (inches, field coordinates)
     * @param targetHeading Target heading (degrees, field-absolute: 0=+X axis, 90=+Y axis)
     * @param maxVelocity Maximum linear velocity (inches/sec)
     * @param acceleration Maximum acceleration (inches/sec^2) - currently unused, reserved for future
     * @param timeoutMs Maximum duration in milliseconds (0 = use default timeout)
     * @return MotionResult with execution status
     */
    public MotionResult moveToPose(double targetX, double targetY, double targetHeading,
                                   double maxVelocity, double acceleration, int timeoutMs) {
        // Validate inputs
        if (maxVelocity <= 0 || maxVelocity > MotionConfig.MAX_LINEAR_VELOCITY) {
            return new MotionResult(false, 0, 0, 0, "Invalid velocity");
        }

        // Mark as executing
        isExecuting = true;
        timer.reset();
        stallTimer.reset();

        // Update state and set target
        updateState();
        // ROBOT CENTER MOTION CONTROL:
        // Convert reference point target to robot center target for efficient motion control
        // smooth operation!

        Pose2D robotCenterTarget = coordinateTransformer.convertReferencePointToRobotCenter(
                targetX, targetY, targetHeading);

        // COORDINATE SYSTEM CONSISTENCY: Use REFERENCE POINT coordinates throughout
        motionState.setTarget(targetX, targetY, targetHeading);

        // Calculate initial distance to target
        double initialDistance = motionState.getPositionError();

        // Apply timeout (use provided or default)
        int effectiveTimeout = (timeoutMs > 0) ? timeoutMs : MotionConfig.MOTION_TIMEOUT_MS;

        // Note: Velocity profiling removed - using hybrid control with Motor PIDF

        // Reset controllers with DISTANCE and HEADING (2-PID architecture)
        driveHardware.getDistanceController().reset(0.0);  // Target distance is always 0 (we want to be AT the target)

        driveHardware.getHeadingController().reset(targetHeading);

        // Track last errors for stall detection
        lastPositionError = initialDistance;
        double lastHeadingError = motionState.getHeadingError();

        // Motion control loop
        while (isExecuting) {
            // Check timeout
            if (timer.milliseconds() > effectiveTimeout) {
                stop();
                return new MotionResult(false, motionState.getPositionError(),
                        motionState.getHeadingError(), timer.milliseconds(), "Timeout");
            }

            // Update state
            updateState();

            // Get current errors
            double positionError = motionState.getPositionError();
            double headingError = motionState.getHeadingError();

            // Check if reached target (both position and heading)
            if (motionState.atTarget()) {
                stop();
                return new MotionResult(true, positionError, headingError,
                        timer.milliseconds(), "Success");
            }

            // Check for stall (no progress on position AND heading)
            double loopTimeSec = MotionConfig.CONTROL_LOOP_PERIOD_MS / 1000.0;
            boolean positionStalled = Math.abs(lastPositionError - positionError) < MotionConfig.STALL_VELOCITY_THRESHOLD * loopTimeSec;
            boolean headingStalled = Math.abs(lastHeadingError - headingError) < MotionConfig.STALL_HEADING_THRESHOLD * loopTimeSec;

            if (positionStalled && headingStalled) {
                if (stallTimer.milliseconds() > MotionConfig.STALL_DETECTION_TIME_MS) {
                    stop();
                    return new MotionResult(false, positionError, headingError,
                            timer.milliseconds(), "Stalled - no progress on position and heading");
                }
            } else {
                stallTimer.reset();
            }

            lastPositionError = positionError;
            lastHeadingError = headingError;


            Pose2D currentRobotCenter = coordinateTransformer.getCurrentRobotCenterPose();

            double robotCenterTargetX = robotCenterTarget.getX(DistanceUnit.INCH);
            double robotCenterTargetY = robotCenterTarget.getY(DistanceUnit.INCH);
            double currentRobotCenterX = currentRobotCenter.getX(DistanceUnit.INCH);
            double currentRobotCenterY = currentRobotCenter.getY(DistanceUnit.INCH);

            // Calculate direction vector toward target (normalized)
            double deltaX = robotCenterTargetX - currentRobotCenterX;
            double deltaY = robotCenterTargetY - currentRobotCenterY;
            double distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
            // CRITICAL: Negate distance so PID error = setpoint(0) - (-distance) = +distance, producing positive speed
            double desiredLinearSpeed = driveHardware.getDistanceController().calculate(-distanceToTarget);

            // Convert speed and direction to velocity components
            double vx, vy;
            if (distanceToTarget > 0.001) {  // Avoid division by zero
                double directionX = deltaX / distanceToTarget;
                double directionY = deltaY / distanceToTarget;
                vx = desiredLinearSpeed * directionX;
                vy = desiredLinearSpeed * directionY;
            } else {
                vx = 0.0;
                vy = 0.0;
            }



            // Apply velocity limits (hybrid control already handles scaling)
            vx = Math.max(-maxVelocity, Math.min(maxVelocity, vx));
            vy = Math.max(-maxVelocity, Math.min(maxVelocity, vy));

            // Calculate rotational velocity using heading controller
            double omega = driveHardware.getHeadingController().calculate(motionState.getHeading());
            omega = Math.max(-MotionConfig.MAX_ANGULAR_VELOCITY,
                    Math.min(MotionConfig.MAX_ANGULAR_VELOCITY, omega));


            // Apply velocity ramping for smooth acceleration/deceleration
            double[] rampedVelocities = applyVelocityRamping(vx, vy, omega);
            vx = rampedVelocities[0];
            vy = rampedVelocities[1];
            omega = rampedVelocities[2];

            setVelocity(vx, vy, omega, MotionState.CoordinateMode.FIELD_CENTRIC);

            // Control loop timing
            try { Thread.sleep(MotionConfig.CONTROL_LOOP_PERIOD_MS); }
            catch (InterruptedException e) { Thread.currentThread().interrupt(); }
        }

        stop();
        return new MotionResult(false, motionState.getPositionError(),
                motionState.getHeadingError(), timer.milliseconds(), "Interrupted");
    }

    /**
     * Move reference point to target field pose (with default velocity)
     * @param targetPose Target position for the reference point
     */
    public MotionResult moveToPose(FieldPose targetPose) {
        return moveToPose(targetPose.x, targetPose.y, targetPose.heading, MotionConfig.MAX_LINEAR_VELOCITY);
    }
    /**
     * Move reference point to target field pose
     * @param targetPose Target position for the reference point
     * @param velocity Movement velocity (inches/sec)
     */
    public MotionResult moveToPose(FieldPose targetPose, double velocity) {
        return moveToPose(targetPose.x, targetPose.y, targetPose.heading, velocity);
    }

    /**
     * Combined motion with simultaneous translation and rotation
     *
     * Most flexible motion primitive - basis for all other motions.
     * Allows arbitrary velocity commands with multiple termination conditions.
     *
     * Termination conditions (first met wins):
     * - Time-based: maxDurationMs > 0
     * - Distance-based: maxDistance > 0
     * - Position tolerance: stopAtTarget = true (requires target set via motionState.setTarget)
     * - Custom condition: terminationCondition lambda returns true
     *
     * @param vx X velocity (inches/sec, forward/backward: + = forward, - = backward)
     * @param vy Y velocity (inches/sec, strafe left/right: + = left, - = right)
     * @param omega Angular velocity (degrees/sec, rotation: + = counter-clockwise, - = clockwise)
     * @param coordinateMode ROBOT_CENTRIC or FIELD_CENTRIC
     * @param maxDurationMs Maximum duration (milliseconds, 0 = no time limit)
     * @param maxDistance Maximum distance to travel (inches, 0 = no distance limit)
     * @param stopAtTarget If true, stops when reaching motionState target position
     * @param terminationCondition Custom termination lambda (can be null)
     * @return MotionResult with execution status
     */

    // ========== ADVANCED MOTION FAMILY ==========
    // Flexible motion commands with custom termination conditions

    public MotionResult combinedMotion(double vx, double vy, double omega,
                                       MotionState.CoordinateMode coordinateMode,
                                       int maxDurationMs, double maxDistance, boolean stopAtTarget,
                                       java.util.function.Supplier<Boolean> terminationCondition) {
        // Validate inputs
        if (Math.abs(vx) > MotionConfig.MAX_LINEAR_VELOCITY ||
                Math.abs(vy) > MotionConfig.MAX_LINEAR_VELOCITY) {
            return new MotionResult(false, 0, 0, 0, "Linear velocity exceeds maximum");
        }
        if (Math.abs(omega) > MotionConfig.MAX_ANGULAR_VELOCITY) {
            return new MotionResult(false, 0, 0, 0, "Angular velocity exceeds maximum");
        }

        // Mark as executing
        isExecuting = true;
        timer.reset();

        // Get starting position
        updateState();

        // ROBOT CENTER MOTION CONTROL: Use robot center for distance calculations
        Pose2D startRobotCenter = coordinateTransformer.getCurrentRobotCenterPose();
        double startX = startRobotCenter.getX(DistanceUnit.INCH);
        double startY = startRobotCenter.getY(DistanceUnit.INCH);
        double distanceTraveled = 0.0;

        // Apply default timeout if none specified
        int timeoutMs = (maxDurationMs > 0) ? maxDurationMs : MotionConfig.MOTION_TIMEOUT_MS;

        // Motion control loop
        while (isExecuting) {
            // Update state
            updateState();

            // Check timeout
            if (timer.milliseconds() > timeoutMs) {
                stop();
                return new MotionResult(false, motionState.getPositionError(),
                        motionState.getHeadingError(), timer.milliseconds(), "Timeout");
            }

            // Check time-based termination
            if (maxDurationMs > 0 && timer.milliseconds() >= maxDurationMs) {
                stop();
                return new MotionResult(true, motionState.getPositionError(),
                        motionState.getHeadingError(), timer.milliseconds(), "Time limit reached");
            }

            // Check distance-based termination
            if (maxDistance > 0) {
                // ROBOT CENTER MOTION CONTROL: Calculate distance using robot center
                Pose2D currentRobotCenter = coordinateTransformer.getCurrentRobotCenterPose();
                double dx = currentRobotCenter.getX(DistanceUnit.INCH) - startX;
                double dy = currentRobotCenter.getY(DistanceUnit.INCH) - startY;
                distanceTraveled = Math.hypot(dx, dy);

                if (distanceTraveled >= maxDistance) {
                    stop();
                    return new MotionResult(true, 0, motionState.getHeadingError(),
                            timer.milliseconds(), "Distance limit reached");
                }
            }

            // Check target-based termination
            if (stopAtTarget && motionState.atTarget()) {
                stop();
                return new MotionResult(true, motionState.getPositionError(),
                        motionState.getHeadingError(), timer.milliseconds(), "Target reached");
            }

            // Check custom termination condition
            if (terminationCondition != null && terminationCondition.get()) {
                stop();
                return new MotionResult(true, motionState.getPositionError(),
                        motionState.getHeadingError(), timer.milliseconds(), "Custom condition met");
            }

            // Apply velocity command
            setVelocity(vx, vy, omega, coordinateMode);

            // Control loop timing
            try { Thread.sleep(MotionConfig.CONTROL_LOOP_PERIOD_MS); }
            catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }

        stop();
        return new MotionResult(false, motionState.getPositionError(),
                motionState.getHeadingError(), timer.milliseconds(), "Interrupted");
    }

    /**
     * Simplified combined motion with time-based termination
     *
     * @param vx X velocity (inches/sec, forward/backward: + = forward, - = backward)
     * @param vy Y velocity (inches/sec, strafe left/right: + = left, - = right)
     * @param omega Angular velocity (degrees/sec, rotation: + = counter-clockwise, - = clockwise)
     * @param coordinateMode ROBOT_CENTRIC or FIELD_CENTRIC
     * @param durationMs Duration in milliseconds
     * @return MotionResult with execution status
     */
    public MotionResult timedMotion(double vx, double vy, double omega,
                                    MotionState.CoordinateMode coordinateMode, int durationMs) {
        return combinedMotion(vx, vy, omega, coordinateMode, durationMs, 0, false, null);
    }

    /**
     * Simplified combined motion with distance-based termination
     *
     * @param vx X velocity (inches/sec, forward/backward: + = forward, - = backward)
     * @param vy Y velocity (inches/sec, strafe left/right: + = left, - = right)
     * @param omega Angular velocity (degrees/sec, rotation: + = counter-clockwise, - = clockwise)
     * @param coordinateMode ROBOT_CENTRIC or FIELD_CENTRIC
     * @param distance Distance limit in inches
     * @return MotionResult with execution status
     */
    public MotionResult distanceMotion(double vx, double vy, double omega,
                                       MotionState.CoordinateMode coordinateMode, double distance) {
        return combinedMotion(vx, vy, omega, coordinateMode, 0, distance, false, null);
    }


    // ========== ROTATION FAMILY ==========
    public MotionResult rotate(double targetAngle) {
        return rotate(targetAngle, MotionConfig.MAX_ANGULAR_VELOCITY);
    }

    public MotionResult rotate(double targetAngle, double angularVelocity) {
        return rotate(targetAngle, angularVelocity, MotionConfig.MOTION_TIMEOUT_MS);
    }

      public MotionResult rotate(double targetAngle, double angularVelocity, int timeoutMs) {
        // Validate inputs
        if (Math.abs(angularVelocity) > MotionConfig.MAX_ANGULAR_VELOCITY) {
            angularVelocity = Math.signum(angularVelocity) * MotionConfig.MAX_ANGULAR_VELOCITY;
        }
        if (targetAngle == 0) {
            return new MotionResult(true, 0, 0, 0, "No rotation needed");
        }
        if (timeoutMs <= 0) {
            return new MotionResult(false, 0, 0, 0, "Timeout must be positive");
        }

        // Get current heading and calculate target
        updateState();
        double startHeading = motionState.getHeading();
        double targetHeading = startHeading + targetAngle;

        targetHeading = MecanumKinematics.normalizeAngle(targetHeading);

        // Calculate where reference point should be after rotating around robot center
        // Get current robot center position
        Pose2D currentRobotCenter = coordinateTransformer.getCurrentRobotCenterPose();
        double robotCenterX = currentRobotCenter.getX(DistanceUnit.INCH);
        double robotCenterY = currentRobotCenter.getY(DistanceUnit.INCH);

        // Create target robot center pose with new heading but same position
        Pose2D targetRobotCenterPose = new Pose2D(DistanceUnit.INCH, robotCenterX, robotCenterY, AngleUnit.DEGREES, targetHeading);

        // Use CoordinateTransformer to convert robot center pose to reference point
        FieldPose targetRefPoint = CoordinateTransformer.convertRobotCenterToReferencePoint(targetRobotCenterPose);

        // Use moveToPose to move reference point to new position with new heading
        // This achieves rotation around robot center
        return moveToPose(targetRefPoint.x, targetRefPoint.y, targetHeading, angularVelocity, 0, timeoutMs);
    }

    // ========== COORDINATE SYSTEM MANAGEMENT ==========
    /**
     * Resets odometry to field origin position
     */
    public void  resetToFieldOrigin() {

        // Set reference point to field origin
        resetToFieldOrigin(new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.DEGREES, 0 ));

    }
    public void resetToFieldOrigin(Pose2D fieldOrigin) {

        odometryManager.resetToFieldOrigin(fieldOrigin);
        // Update motion state with new position
        motionState.updateFromOdometry();
    }

    // ========== MULTI-MODE CONTROL SYSTEM ==========

    /**
     * Unified control calculation supporting all three control modes
     *
     * @param currentPosition Current position (inches)
     * @param targetPosition Target position (inches)
     * @param maxVelocity Maximum velocity (inches/sec)
     * @param pidController PID controller for this axis
     * @return Velocity command (inches/sec)
     */
    private double calculateControlOutput(double currentPosition, double targetPosition,
                                          double maxVelocity, PIDController pidController) {
        switch (currentControlMode) {
            case PURE_FEEDFORWARD:
                return calculatePureFeedforward(currentPosition, targetPosition, maxVelocity);

            case PURE_FEEDBACK:
                return calculatePureFeedback(currentPosition, pidController);

            case HYBRID:
                return calculatePositionControl(currentPosition, targetPosition, maxVelocity, pidController);

            default:
                return calculatePositionControl(currentPosition, targetPosition, maxVelocity, pidController);
        }
    }

    /**
     * Pure feedforward control - open loop, predictable timing
     *
     * Calculates target velocity based on position error and desired approach time.
     * Best for: Known trajectories, repeatable autonomous paths, time-critical moves
     *
     * @param currentPosition Current position (inches)
     * @param targetPosition Target position (inches)
     * @param maxVelocity Maximum velocity (inches/sec)
     * @return Velocity command (inches/sec)
     */
    private double calculatePureFeedforward(double currentPosition, double targetPosition, double maxVelocity) {
        double positionError = targetPosition - currentPosition;
        double absError = Math.abs(positionError);

        // Calculate time to target based on maximum velocity
        double timeToTarget = Math.max(absError / maxVelocity, 0.1); // Minimum time to prevent division issues

        // Calculate target velocity: position error / time to target
        double targetVelocity = positionError / timeToTarget;

        // Apply velocity limits
        return Math.max(-maxVelocity, Math.min(maxVelocity, targetVelocity));
    }

    /**
     * Pure feedback control - closed loop, disturbance rejection
     *
     * Uses only PID controller for all error ranges.
     * Best for: Precision positioning, holding position, dealing with disturbances
     *
     * @param currentPosition Current position (inches)
     * @param pidController PID controller for this axis
     * @return Velocity command (inches/sec)
     */
    private double calculatePureFeedback(double currentPosition, PIDController pidController) {
        // Simple: just use PID controller output
        return pidController.calculate(currentPosition);
    }

    /**
     * Apply velocity ramping to smooth acceleration and deceleration
     *
     * Limits the rate of change of velocity commands to prevent jerky motion
     * and reduce mechanical stress on the robot.
     *
     * @param targetVx Target X velocity (inches/sec)
     * @param targetVy Target Y velocity (inches/sec)
     * @param targetVAngular Target angular velocity (degrees/sec)
     * @return Array of ramped velocities [vx, vy, vAngular]
     */
    private double[] applyVelocityRamping(double targetVx, double targetVy, double targetVAngular) {
        long currentTime = System.currentTimeMillis();

        // Initialize timing on first call
        if (lastControlTime == 0) {
            lastControlTime = currentTime;
            previousVx = 0.0;
            previousVy = 0.0;
            previousVAngular = 0.0;
        }

        // Calculate time delta
        double deltaTime = (currentTime - lastControlTime) / 1000.0; // Convert to seconds
        deltaTime = Math.max(deltaTime, 0.001); // Minimum delta to prevent division by zero

        // Calculate maximum allowed velocity change based on acceleration limit
        double maxLinearVelocityChange = MotionConfig.MAX_LINEAR_ACCELERATION * deltaTime;
        double maxAngularVelocityChange = MotionConfig.MAX_ANGULAR_ACCELERATION * deltaTime;

        // Apply ramping to X velocity
        double vxChange = targetVx - previousVx;
        if (Math.abs(vxChange) > maxLinearVelocityChange) {
            vxChange = Math.signum(vxChange) * maxLinearVelocityChange;
        }
        double rampedVx = previousVx + vxChange;

        // Apply ramping to Y velocity
        double vyChange = targetVy - previousVy;
        if (Math.abs(vyChange) > maxLinearVelocityChange) {
            vyChange = Math.signum(vyChange) * maxLinearVelocityChange;
        }
        double rampedVy = previousVy + vyChange;

        // Apply ramping to angular velocity
        double vAngularChange = targetVAngular - previousVAngular;
        if (Math.abs(vAngularChange) > maxAngularVelocityChange) {
            vAngularChange = Math.signum(vAngularChange) * maxAngularVelocityChange;
        }
        double rampedVAngular = previousVAngular + vAngularChange;

        // Store for next iteration
        previousVx = rampedVx;
        previousVy = rampedVy;
        previousVAngular = rampedVAngular;
        lastControlTime = currentTime;

        return new double[]{rampedVx, rampedVy, rampedVAngular};
    }

    /**
     * Calculate position control output based on the current control mode.
     *
     * @param currentPosition Current position (inches)
     * @param targetPosition Target position (inches)
     * @param maxVelocity Maximum velocity (inches/sec)
     * @param pidController PID controller for this axis
     * @return Velocity command (inches/sec)
     */
    private double calculatePositionControl(double currentPosition, double targetPosition,
                                            double maxVelocity, PIDController pidController) {
        double positionError = targetPosition - currentPosition;
        double absError = Math.abs(positionError);

        switch (positionControlMode) {
            case PURE_FEEDBACK:
                // Pure PID control
                return pidController.calculate(currentPosition);

            case PURE_FEEDFORWARD:
                // Pure feedforward control
                double timeToTarget = Math.max(absError / maxVelocity, 0.1);
                return positionError / timeToTarget;

            case HYBRID:
                // Feedforward + feedback blend
                double timeToTarget2 = Math.max(absError / maxVelocity, 0.1);
                double feedforward = positionError / timeToTarget2;
                double feedback = pidController.calculate(currentPosition);

                // Use fixed blend ratio (80% feedforward, 20% feedback)
                return positionFeedforwardGain * feedforward + (1.0 - positionFeedforwardGain) * feedback;

            default:
                return pidController.calculate(currentPosition);
        }
    }


    /**
     * Calculate angular control output using pure feedback control.
     *
     * @param currentAngle Current angular position (degrees)
     * @param targetAngle Target angular position (degrees)
     * @param pidController PID controller for angular control
     * @return Angular velocity command (degrees/sec)
     */
    private double calculateAngularControl(double currentAngle, double targetAngle,
                                           PIDController pidController) {
        // Pure PID control for heading - no feedforward or hybrid modes
        return pidController.calculate(currentAngle);
    }


    // ========== LOW-LEVEL MOTOR CONTROL ==========

    /**
     * Sets individual motor powers for calibration testing
     * @param frontLeft Front left motor power (-1.0 to 1.0)
     * @param frontRight Front right motor power (-1.0 to 1.0)
     * @param backLeft Back left motor power (-1.0 to 1.0)
     * @param backRight Back right motor power (-1.0 to 1.0)
     */
    public void setMotorPowers(double frontLeft, double frontRight, double backLeft, double backRight) {
        driveHardware.setMotorPowers(frontLeft, frontRight, backLeft, backRight);
    }

    /**
     * Gets current encoder positions for calibration
     * @return Array of encoder positions [frontLeft, frontRight, backLeft, backRight]
     */
    public int[] getEncoderPositions() {
        return driveHardware.getEncoderPositions();
    }

    /**
     * Sets motor velocities directly with individual parameters
     * @param frontLeft Front left motor velocity in ticks/sec
     * @param frontRight Front right motor velocity in ticks/sec
     * @param backLeft Back left motor velocity in ticks/sec
     * @param backRight Back right motor velocity in ticks/sec
     */
    public void setMotorVelocities(double frontLeft, double frontRight, double backLeft, double backRight) {
        driveHardware.setMotorVelocities(frontLeft, frontRight, backLeft, backRight);
    }

    /**
     * Sets motor velocities using physical units (inches/sec) - automatically converts to ticks/sec
     * This is the preferred method for calibration and testing where physical units are more intuitive.
     *
     * @param frontLeft Front left motor velocity in inches/sec
     * @param frontRight Front right motor velocity in inches/sec
     * @param backLeft Back left motor velocity in inches/sec
     * @param backRight Back right motor velocity in inches/sec
     */
    public void setMotorVelocitiesPhysical(double frontLeft, double frontRight, double backLeft, double backRight) {
        driveHardware.setMotorVelocitiesPhysical(frontLeft, frontRight, backLeft, backRight);
    }


    // ========== PID CALIBRATION SUPPORT ==========

    /**
     * Updates X-axis position PID parameters for real-time calibration
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     */
    public void updatePositionXPID(double kP, double kI, double kD) {
        driveHardware.updatePositionXPID(kP, kI, kD);
    }

    /**
     * Updates Y-axis position PID parameters for real-time calibration
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     */
    public void updatePositionYPID(double kP, double kI, double kD) {
        driveHardware.updatePositionYPID(kP, kI, kD);
    }

    /**
     * Updates distance PID parameters for real-time calibration (2-PID architecture)
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     */
    public void updateDistancePID(double kP, double kI, double kD) {
        driveHardware.updateDistancePID(kP, kI, kD);
    }

    /**
     * Updates heading PID parameters for real-time calibration
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     */
    public void updateHeadingPID(double kP, double kI, double kD) {
        driveHardware.updateHeadingPID(kP, kI, kD);
    }

    /**
     * Gets current X-axis position PID parameters
     * @return Array of [kP, kI, kD] values
     */
    public double[] getPositionXPID() {
        return driveHardware.getPositionXPID();
    }

    /**
     * Gets current Y-axis position PID parameters
     * @return Array of [kP, kI, kD] values
     */
    public double[] getPositionYPID() {
        return driveHardware.getPositionYPID();
    }

    /**
     * Gets current heading PID parameters
     * @return Array of [kP, kI, kD] values
     */
    public double[] getHeadingPID() {
        return driveHardware.getHeadingPID();
    }

    // ========== CONTROL MODE MANAGEMENT METHODS ==========

    /**
     * Set the control mode for motion execution
     *
     * @param mode The control mode to use (PURE_FEEDFORWARD, PURE_FEEDBACK, or HYBRID)
     */
    public void setControlMode(ControlMode mode) {
        this.currentControlMode = mode;
    }

    /**
     * Get the current control mode
     *
     * @return The current control mode
     */
    public ControlMode getControlMode() {
        return currentControlMode;
    }

    /**
     * Set the default control mode (used when no mode is specified)
     *
     * @param mode The default control mode
     */
    public void setDefaultControlMode(ControlMode mode) {
        this.defaultControlMode = mode;
        this.currentControlMode = mode; // Also set as current
    }

    /**
     * Reset to default control mode
     */
    public void resetToDefaultControlMode() {
        this.currentControlMode = defaultControlMode;
    }

    /**
     * Set the hybrid gain for HYBRID control mode
     *
     * @param gain Hybrid gain (0.0 = 100% feedback, 1.0 = 100% feedforward)
     */
    public void setHybridGain(double gain) {
        // Clamp gain to valid range [0.0, 1.0]
        this.positionFeedforwardGain = Math.max(0.0, Math.min(1.0, gain));
    }

    /**
     * Get the current hybrid gain
     *
     * @return Current hybrid gain value (0.0 = 100% feedback, 1.0 = 100% feedforward)
     */
    public double getHybridGain() {
        return positionFeedforwardGain;
    }


    /**
     * Gets the DriveHardware instance for direct hardware access
     *
     * This is primarily used by calibration modules that need direct access
     * to the hardware layer for velocity control and calibration.
     *
     * @return DriveHardware instance
     */
    public DriveHardware getDriveHardware() {
        return driveHardware;
    }
}