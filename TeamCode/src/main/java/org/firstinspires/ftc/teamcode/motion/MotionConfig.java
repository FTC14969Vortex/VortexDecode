package org.firstinspires.ftc.teamcode.motion;

import org.firstinspires.ftc.teamcode.calibration.CalibrationCoefficients;
import org.firstinspires.ftc.teamcode.calibration.RobotConstants;
import org.firstinspires.ftc.teamcode.calibration.RobotConstants.ComponentPosition;
import org.firstinspires.ftc.teamcode.motion.FieldPositions;


/**
 * Configuration constants for the motion control system.
 *
 * ============================================================
 * REQUIRED SETUP - MUST BE MEASURED/TUNED BEFORE FIRST USE:
 * ============================================================
 *
 * 1. ROBOT DIMENSIONS (lines 30-35):
 *    - TRACK_WIDTH: Measure center-to-center distance between left/right wheels
 *    - WHEELBASE: Measure center-to-center distance between front/back wheels
 *    - WHEEL_DIAMETER: Measure actual wheel diameter (affects odometry accuracy)
 *
 * 2. ODOMETRY OFFSETS (lines 65-75):
 *    - ODOMETRY_X_OFFSET: Distance from robot center to odometry computer (forward/back)
 *    - ODOMETRY_Y_OFFSET: Distance from robot center to odometry computer (left/right)
 *    - CRITICAL: Incorrect offsets cause rotation errors!
 *
 * 3. FIELD INITIAL POSITION (lines 160-170):
 - FIELD_ORIGIN_X, FIELD_ORIGIN_Y: Robot's starting position on field (inches)
 - FIELD_ORIGIN_HEADING: Robot's starting orientation (degrees)
 - CRITICAL for field-centric navigation accuracy
 *
 * 5. PID GAINS (lines 120-130):
 *    - POSITION_KP, HEADING_KP: Start with defaults, tune for smooth motion
 *    - Increase Kp for faster response, decrease if oscillating
 *
 * 6. MOTION CONSTRAINTS (lines 85-95):
 *    - MAX_LINEAR_VELOCITY: Adjust based on desired speed vs control
 *    - MAX_LINEAR_ACCELERATION: Test and reduce if wheels slip
 *
 * 7. VELOCITY_FEEDFORWARD (line 130):
 *    - Tune for accurate velocity control
 *    - Adjust if robot consistently over/undershoots target speeds
 *
 * ============================================================
 *
 * Contains all tunable parameters for:
 * - Robot physical dimensions
 * - Motor specifications
 * - Motion constraints (velocity, acceleration)
 * - Control loop parameters
 * - PID gains
 * - Safety limits
 */
public class MotionConfig {

    // ========== ROBOT PHYSICAL PARAMETERS ==========

    /** Distance between left and right wheels (inches) */
    public static final double TRACK_WIDTH = RobotConstants.TRACK_WIDTH;

    /** Distance between front and back wheels (inches) */
    public static final double WHEELBASE = RobotConstants.WHEELBASE;

    /** Wheel diameter (inches) */
    public static final double WHEEL_DIAMETER = RobotConstants.WHEEL_DIAMETER;

    /** Wheel radius (inches) */
    public static final double WHEEL_RADIUS = WHEEL_DIAMETER / 2.0;


    // ========== MOTOR CONFIGURATION ==========

    /** Motor encoder counts per revolution */
    public static final double COUNTS_PER_MOTOR_REV = RobotConstants.ENCODER_COUNTS_PER_REV;

    /** Gear reduction ratio (1.0 = direct drive) */
    public static final double DRIVE_GEAR_REDUCTION = RobotConstants.GEAR_REDUCTION;

    /** Maximum motor RPM (goBILDA 5203 series) */
    public static final double MAX_MOTOR_RPM = RobotConstants.MOTOR_MAX_RPM_DATASHEET;

    // ========== DERIVED CONSTANTS ==========

    /** Encoder counts per inch of linear travel */
    public static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (Math.PI * WHEEL_DIAMETER);

    /** Maximum theoretical wheel velocity (inches/sec) */
    public static final double MAX_WHEEL_VELOCITY =
            (MAX_MOTOR_RPM / 60.0) * (Math.PI * WHEEL_DIAMETER);

    /** Encoder ticks per second at max RPM */
    public static final double MAX_TICKS_PER_SEC =
            (MAX_MOTOR_RPM / 60.0) * COUNTS_PER_MOTOR_REV;

    /** Turning radius - distance from robot center to wheel (inches) */
    public static final double TURNING_RADIUS = Math.sqrt(
            Math.pow(TRACK_WIDTH / 2.0, 2) + Math.pow(WHEELBASE / 2.0, 2)
    );

    // ========== MOTION CONSTRAINTS ==========

    /** Maximum linear velocity (inches/sec) - tunable, should be < MAX_WHEEL_VELOCITY */
    public static final double MAX_LINEAR_VELOCITY = CalibrationCoefficients.CALIBRATED_MAX_LINEAR_VELOCITY;

    /** Maximum linear acceleration (inches/sec^2) */
    public static final double MAX_LINEAR_ACCELERATION = CalibrationCoefficients.CALIBRATED_MAX_LINEAR_ACCELERATION;

    /** Maximum angular velocity (degrees/sec) - derived from MAX_LINEAR_VELOCITY / TURNING_RADIUS */
    public static final double MAX_ANGULAR_VELOCITY = 0.25*CalibrationCoefficients.CALIBRATED_MAX_ANGULAR_VELOCITY;

    /** Maximum angular acceleration (degrees/sec^2) - derived from MAX_LINEAR_ACCELERATION / TURNING_RADIUS */
    public static final double MAX_ANGULAR_ACCELERATION = CalibrationCoefficients.CALIBRATED_MAX_ANGULAR_ACCELERATION;

    // ========== CONTROL TOLERANCES ==========

    /** Position tolerance for move completion (inches) */
    public static final double POSITION_TOLERANCE = 0.5;

    /** Heading tolerance for rotation completion (degrees) */
    public static final double HEADING_TOLERANCE = 2.0;

    /** Velocity tolerance for zero-velocity detection (inches/sec) */
    public static final double VELOCITY_TOLERANCE = 2.0;

    /** Dead-band for position control - no movement within this range (inches) */
    public static final double POSITION_DEADBAND = 0.5;

    /** Dead-band for heading control - no rotation within this range (degrees) */
    public static final double HEADING_DEADBAND = 2;

    // ========== CONTROL LOOP PARAMETERS ==========

    /** Control loop period (milliseconds) - 50 Hz */
    public static final int CONTROL_LOOP_PERIOD_MS = 20;

    /** Control loop frequency (Hz) */
    public static final double CONTROL_LOOP_FREQUENCY = 1000.0 / CONTROL_LOOP_PERIOD_MS;

    // ========== PID CONTROLLER GAINS ==========

    // X-axis position controller (East-West motion)
    public static final double POSITION_X_KP = 5;   // Proportional gain for X-axis control, with 0.5" error, vx = 0.25 inch/s
    public static final double POSITION_X_KI = 0.1;  // Integral gain for X-axis control
    public static final double POSITION_X_KD = 0.0;    // Derivative gain for X-axis control

    // Y-axis position controller (North-South motion)
    public static final double POSITION_Y_KP = 5;   // Proportional gain for Y-axis control
    public static final double POSITION_Y_KI = 0.1;  // Integral gain for Y-axis control
    public static final double POSITION_Y_KD = 0.0;    // Derivative gain for Y-axis control


    // Distance controller (2-PID architecture: unified linear motion control)
    public static final double DISTANCE_KP = 3.5;   // Proportional gain for distance control (unified X/Y)
    public static final double DISTANCE_KI = 0.1;   // Integral gain for distance control
    public static final double DISTANCE_KD = 1.0;   // Derivative gain for distance control

    // Heading controller (rotation)
    public static final double HEADING_KP = 6.0;    // Increased for better heading control
    public static final double HEADING_KI = 2.0;   // Slightly increased integral gain
    public static final double HEADING_KD = 1;    // Derivative gain disabled for PID tuning



    // ========== MOTOR CONTROLLER PIDF COEFFICIENTS ==========

    /**
     * Motor velocity control PIDF coefficients for goBILDA 5203 series motors.
     *
     * These coefficients are used by the REV Hub's internal motor controllers
     * for closed-loop velocity control when using setVelocity().
     *
     * CRITICAL: These MUST be configured for setVelocity() to work properly!
     * Without these coefficients, motors will not respond to velocity commands.
     *
     * COEFFICIENT EXPLANATIONS:
     *
     * P (Proportional): 10.0
     *   - Primary gain for velocity error correction
     *   - Higher values = faster response, but may cause oscillation
     *   - Typical range for goBILDA motors: 8.0 - 15.0
     *   - Formula: motor_power = P * velocity_error
     *
     * I (Integral): 3.0
     *   - Eliminates steady-state velocity errors
     *   - Accumulates error over time to reach exact target velocity
     *   - Too high causes overshoot and instability
     *   - Typical range: 1.0 - 5.0
     *
     * D (Derivative): 0.0
     *   - Dampens oscillations and improves stability
     *   - Usually not needed for velocity control
     *   - Can cause noise amplification if set too high
     *   - Recommended: Start with 0.0, increase only if oscillating
     *
     * F (Feedforward): 16.8
     *   - MOST CRITICAL coefficient for velocity control!
     *   - Provides baseline power proportional to target velocity
     *   - Calculated as: F = 32767 / max_ticks_per_second
     *   - For goBILDA 5203 at 70% load: F = 32767 / 1954.5 ≈ 16.8
     *   - Accounts for realistic loaded motor performance
     *
     * CALCULATION DETAILS:
     * Max motor speed: 218.4 RPM (70% of 312 RPM unloaded, accounting for load)
     * Max ticks/sec: (218.4 RPM / 60) * 537.7 counts/rev = 1954.5 ticks/sec
     * REV Hub max output: 32767 (16-bit signed integer)
     * Feedforward: F = 32767 / 1954.5 = 16.77 ≈ 16.8
     *
     * TUNING NOTES:
     * - Start with these values and test basic movement
     * - If motors don't move: Check F coefficient first
     * - If oscillating: Reduce P, increase I slightly
     * - If slow response: Increase P, but watch for oscillation
     * - If steady-state error: Increase I slightly
     */
    public static final double MOTOR_VELOCITY_KP = 10.0;  // Proportional gain
    public static final double MOTOR_VELOCITY_KI = 3.0;   // Integral gain
    public static final double MOTOR_VELOCITY_KD = 0.0;   // Derivative gain
    public static final double MOTOR_VELOCITY_KF = 15; //was 16.8 - overshoot;  // Feedforward gain

    // ========== MOTOR DIRECTION CONFIGURATION ==========

    /** Motor direction constants for consistent configuration across all classes */
    public static final com.qualcomm.robotcore.hardware.DcMotor.Direction FRONT_LEFT_DIRECTION =
            RobotConstants.FRONT_LEFT_DIRECTION;
    public static final com.qualcomm.robotcore.hardware.DcMotor.Direction FRONT_RIGHT_DIRECTION =
            RobotConstants.FRONT_RIGHT_DIRECTION;
    public static final com.qualcomm.robotcore.hardware.DcMotor.Direction BACK_LEFT_DIRECTION =
            RobotConstants.BACK_LEFT_DIRECTION;
    public static final com.qualcomm.robotcore.hardware.DcMotor.Direction BACK_RIGHT_DIRECTION =
            RobotConstants.BACK_RIGHT_DIRECTION;

    // ========== ENCODER REVERSAL CONFIGURATION ==========

    /** Encoder reversal constants for consistent encoder reading across all classes */
    public static final boolean FRONT_LEFT_ENCODER_REVERSED = RobotConstants.FRONT_LEFT_ENCODER_REVERSED;
    public static final boolean FRONT_RIGHT_ENCODER_REVERSED = RobotConstants.FRONT_RIGHT_ENCODER_REVERSED;
    public static final boolean BACK_LEFT_ENCODER_REVERSED = RobotConstants.BACK_LEFT_ENCODER_REVERSED;
    public static final boolean BACK_RIGHT_ENCODER_REVERSED = RobotConstants.BACK_RIGHT_ENCODER_REVERSED;


    // ========== HYBRID CONTROL CALIBRATION CONSTANTS ==========

    /** Position feedforward gain for hybrid control calibration */
    public static final double POSITION_FEEDFORWARD_GAIN = 0.5;  // TODO: CALIBRATE THIS VALUE!



    /** Acceleration limit for motion control (inches/sec²) */
    public static final double ACCELERATION_LIMIT = CalibrationCoefficients.CALIBRATED_MAX_LINEAR_ACCELERATION;  // TODO: CALIBRATE THIS VALUE!
    // ========== SAFETY LIMITS ==========

    /** Default timeout for motion commands (milliseconds) */
    public static final int MOTION_TIMEOUT_MS = 5000;  // Increased for more reliable motion execution

    /** Safety factor for timeout calculations (multiplier for estimated time) */
    public static final double TIMEOUT_SAFETY_FACTOR = 20.0;  // 50% extra time for acceleration/deceleration

    /** Stall detection time - no progress for this duration = stalled (milliseconds) */
    public static final int STALL_DETECTION_TIME_MS = 1000;

    /** Minimum motor power to overcome static friction */


    /** Velocity threshold for stall detection (inches/sec) */
    public static final double STALL_VELOCITY_THRESHOLD = 1.0;


    // Prevent instantiation
    private MotionConfig() {}

}
