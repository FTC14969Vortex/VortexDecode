package org.firstinspires.ftc.teamcode.motion;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.external.gobilida.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.calibration.CalibrationCoefficients;

/**
 * Hardware abstraction layer for drive system.
 * 
 * Responsibilities:
 * - Motor configuration and control
 * - Hardware state management
 * - PID controller management for calibration
 * - Low-level hardware commands
 * 
 * This class isolates all hardware-specific operations from motion control logic,
 * enabling proper PID calibration and improved testability.
 */
public class DriveHardware {
    
    // ========== HARDWARE COMPONENTS ==========
    private final DcMotorEx frontLeftMotor;
    private final DcMotorEx frontRightMotor;
    private final DcMotorEx backLeftMotor;
    private final DcMotorEx backRightMotor;
    private final GoBildaPinpointDriver odometry;
    
    // ========== CONTROLLERS ==========
    private PIDController positionXController;
    private PIDController positionYController;
    private PIDController distanceController;  // 2-PID architecture: unified distance control
    private PIDController headingController;
    
    // ========== CONSTRUCTOR ==========
    
    /**
     * Creates a new DriveHardware instance
     * 
     * @param frontLeft Front left motor (DcMotorEx)
     * @param frontRight Front right motor (DcMotorEx)
     * @param backLeft Back left motor (DcMotorEx)
     * @param backRight Back right motor (DcMotorEx)
     * @param odometry Odometry system (GoBilda Pinpoint)
     */
    public DriveHardware(DcMotorEx frontLeft, DcMotorEx frontRight, 
                        DcMotorEx backLeft, DcMotorEx backRight,
                        GoBildaPinpointDriver odometry) {
        this.frontLeftMotor = frontLeft;
        this.frontRightMotor = frontRight;
        this.backLeftMotor = backLeft;
        this.backRightMotor = backRight;
        this.odometry = odometry;
        
        initializeControllers();
        configureMotors();
    }
    
    // ========== INITIALIZATION ==========
    
    /**
     * Initialize PID controllers and velocity profiler
     * Uses factory methods to ensure proper velocity-based configuration
     */
    private void initializeControllers() {
        // Position X controller (for East-West motion)
        // Use X-specific factory method with X-specific gains
        positionXController = PIDController.createPositionXController();
        
        // Position Y controller (for North-South motion)  
        // Use Y-specific factory method with Y-specific gains
        positionYController = PIDController.createPositionYController();
        
        // Distance controller (2-PID architecture: unified linear motion control)
        // Use distance-specific factory method with distance-specific gains
        distanceController = PIDController.createDistanceController();
        
        // Heading controller (for rotation)
        // Use factory method to ensure velocity-based limits
        headingController = PIDController.createHeadingController();
        
        // Note: Velocity profiling removed - using Motor PIDF for velocity control
    }
    
    /**
     * Configure motor settings
     */
    private void configureMotors() {
        // Set to RUN_USING_ENCODER for velocity control
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Use centralized motor direction configuration from MotionConfig
        frontLeftMotor.setDirection(MotionConfig.FRONT_LEFT_DIRECTION);
        frontRightMotor.setDirection(MotionConfig.FRONT_RIGHT_DIRECTION);
        backLeftMotor.setDirection(MotionConfig.BACK_LEFT_DIRECTION);
        backRightMotor.setDirection(MotionConfig.BACK_RIGHT_DIRECTION);
        
        // Set zero power behavior to BRAKE for precise stops
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Configure PIDF coefficients for velocity control
        // CRITICAL: This is required for setVelocity() to work properly!
        PIDFCoefficients velocityPIDF = new PIDFCoefficients(
            MotionConfig.MOTOR_VELOCITY_KP,
            MotionConfig.MOTOR_VELOCITY_KI,
            MotionConfig.MOTOR_VELOCITY_KD,
            MotionConfig.MOTOR_VELOCITY_KF
        );
        
        frontLeftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, velocityPIDF);
        frontRightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, velocityPIDF);
        backLeftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, velocityPIDF);
        backRightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, velocityPIDF);
    }
    
    // ========== MOTOR CONTROL ==========
    
    /**
     * Sets motor velocities from wheel velocities object (velocity-based control)
     * 
     * Uses direct velocity control for precise motion. Velocities are in ticks/sec
     * and are handled by the motor controller's internal PID loop.
     * 
     * @param wheelVelocities Wheel velocities in ticks/sec
     */
    public void setWheelVelocities(MecanumKinematics.WheelVelocities wheelVelocities) {
        frontLeftMotor.setVelocity(wheelVelocities.frontLeft);
        frontRightMotor.setVelocity(wheelVelocities.frontRight);
        backLeftMotor.setVelocity(wheelVelocities.backLeft);
        backRightMotor.setVelocity(wheelVelocities.backRight);
    }
    
    /**
     * Sets robot velocity with full calibration support (ROBOT_CENTRIC coordinates)
     * 
     * This method applies the currently configured calibration method:
     * - 8-factor scaling (if USE_8_FACTOR_VELOCITY_SCALING = true)
     * - Kinematic matrix calibration (if KINEMATIC_MATRIX_CALIBRATED = true)
     * - Ideal kinematics (if no calibration enabled)
     * 
     * @param vx Robot velocity in X direction (inches/sec, forward/backward: + = forward, - = backward)
     * @param vy Robot velocity in Y direction (inches/sec, strafe left/right: + = left, - = right)
     * @param omega Angular velocity (degrees/sec, rotation: + = counter-clockwise, - = clockwise)
     */
    public void setRobotVelocity(double vx, double vy, double omega) {
        setRobotVelocity(vx, vy, omega, MotionState.CoordinateMode.ROBOT_CENTRIC, 0.0);
    }
    
    /**
     * Sets robot velocity with full calibration support and coordinate transformation
     * 
     * This method applies the currently configured calibration method:
     * - 8-factor scaling (if USE_8_FACTOR_VELOCITY_SCALING = true)
     * - Kinematic matrix calibration (if KINEMATIC_MATRIX_CALIBRATED = true)
     * - Ideal kinematics (if no calibration enabled)
     * 
     * @param vx Velocity in X direction (inches/sec, forward/backward: + = forward, - = backward)
     * @param vy Velocity in Y direction (inches/sec, strafe left/right: + = left, - = right)
     * @param omega Angular velocity (degrees/sec, rotation: + = counter-clockwise, - = clockwise)
     * @param coordinateMode ROBOT_CENTRIC or FIELD_CENTRIC
     * @param robotHeading Current robot heading for field-centric transformation (degrees)
     */
    public void setRobotVelocity(double vx, double vy, double omega, MotionState.CoordinateMode coordinateMode, double robotHeading) {
        // Transform to robot frame if needed
        MecanumKinematics.Velocity2D robotVel;
        if (coordinateMode == MotionState.CoordinateMode.FIELD_CENTRIC) {
            robotVel = MecanumKinematics.fieldToRobotFrame(vx, vy, robotHeading);
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
        setWheelVelocities(wheelVelocitiesTicks);
    }
    
    /**
     * Sets robot velocity using IDEAL kinematics only (no calibration applied)
     * 
     * This method always uses ideal mecanum kinematics regardless of calibration settings.
     * Useful for testing, comparison, or when you want uncalibrated motion.
     * 
     * @param vx Robot velocity in X direction (inches/sec, forward/backward: + = forward, - = backward)
     * @param vy Robot velocity in Y direction (inches/sec, strafe left/right: + = left, - = right)
     * @param omega Angular velocity (degrees/sec, rotation: + = counter-clockwise, - = clockwise)
     */
    public void setRobotVelocityIdeal(double vx, double vy, double omega) {
        setRobotVelocityIdeal(vx, vy, omega, MotionState.CoordinateMode.ROBOT_CENTRIC, 0.0);
    }
    
    /**
     * Sets robot velocity using IDEAL kinematics only with coordinate transformation
     * 
     * This method always uses ideal mecanum kinematics regardless of calibration settings.
     * Useful for testing, comparison, or when you want uncalibrated motion.
     * 
     * @param vx Velocity in X direction (inches/sec, forward/backward: + = forward, - = backward)
     * @param vy Velocity in Y direction (inches/sec, strafe left/right: + = left, - = right)
     * @param omega Angular velocity (degrees/sec, rotation: + = counter-clockwise, - = clockwise)
     * @param coordinateMode ROBOT_CENTRIC or FIELD_CENTRIC
     * @param robotHeading Current robot heading for field-centric transformation (degrees)
     */
    public void setRobotVelocityIdeal(double vx, double vy, double omega, MotionState.CoordinateMode coordinateMode, double robotHeading) {
        // Transform to robot frame if needed
        MecanumKinematics.Velocity2D robotVel;
        if (coordinateMode == MotionState.CoordinateMode.FIELD_CENTRIC) {
            robotVel = MecanumKinematics.fieldToRobotFrame(vx, vy, robotHeading);
        } else {
            robotVel = new MecanumKinematics.Velocity2D(vx, vy, omega);
        }
        double vx_robot = robotVel.vx;
        double vy_robot = robotVel.vy;
        
        // Always use ideal kinematics (no calibration)
        MecanumKinematics.WheelVelocities wheelVelocities = 
            MecanumKinematics.robotVelocitiesToWheelVelocities(vx_robot, vy_robot, omega);
        
        // Convert velocities to ticks/sec for motor velocity control
        MecanumKinematics.WheelVelocities wheelVelocitiesTicks = 
            MecanumKinematics.wheelVelocitiesToTicks(wheelVelocities);
        
        // Apply to motors using velocity control
        setWheelVelocities(wheelVelocitiesTicks);
    }
    
    /**
     * Stops all motors
     */
    public void stop() {
        frontLeftMotor.setVelocity(0);
        frontRightMotor.setVelocity(0);
        backLeftMotor.setVelocity(0);
        backRightMotor.setVelocity(0);
    }
    
    /**
     * Sets motor powers directly (power-based control)
     * 
     * @param frontLeft Front left motor power (-1.0 to 1.0)
     * @param frontRight Front right motor power (-1.0 to 1.0)
     * @param backLeft Back left motor power (-1.0 to 1.0)
     * @param backRight Back right motor power (-1.0 to 1.0)
     */
    public void setMotorPowers(double frontLeft, double frontRight, double backLeft, double backRight) {
        frontLeftMotor.setPower(frontLeft);
        frontRightMotor.setPower(frontRight);
        backLeftMotor.setPower(backLeft);
        backRightMotor.setPower(backRight);
    }
    
    /**
     * Sets motor velocities directly with individual parameters
     * @param frontLeft Front left motor velocity in ticks/sec
     * @param frontRight Front right motor velocity in ticks/sec  
     * @param backLeft Back left motor velocity in ticks/sec
     * @param backRight Back right motor velocity in ticks/sec
     */
    public void setMotorVelocities(double frontLeft, double frontRight, double backLeft, double backRight) {
        frontLeftMotor.setVelocity(frontLeft);
        frontRightMotor.setVelocity(frontRight);
        backLeftMotor.setVelocity(backLeft);
        backRightMotor.setVelocity(backRight);
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
        // Convert physical velocities (inches/sec) to motor velocities (ticks/sec)
        double frontLeftTicks = MecanumKinematics.velocityToTicks(frontLeft);
        double frontRightTicks = MecanumKinematics.velocityToTicks(frontRight);
        double backLeftTicks = MecanumKinematics.velocityToTicks(backLeft);
        double backRightTicks = MecanumKinematics.velocityToTicks(backRight);
        
        // Apply to motors
        setMotorVelocities(frontLeftTicks, frontRightTicks, backLeftTicks, backRightTicks);
    }
    
    // ========== HARDWARE STATE ==========
    
    /**
     * Gets current encoder positions for calibration
     * @return Array of encoder positions [frontLeft, frontRight, backLeft, backRight]
     */
    public int[] getEncoderPositions() {
        return new int[] {
            frontLeftMotor.getCurrentPosition(),
            frontRightMotor.getCurrentPosition(),
            backLeftMotor.getCurrentPosition(),
            backRightMotor.getCurrentPosition()
        };
    }
    
    /**
     * Checks if any motor is currently moving
     * @return true if any motor has non-zero velocity
     */
    public boolean isMoving() {
        return frontLeftMotor.getVelocity() != 0 ||
               frontRightMotor.getVelocity() != 0 ||
               backLeftMotor.getVelocity() != 0 ||
               backRightMotor.getVelocity() != 0;
    }
    
    /**
     * Gets current motor velocities
     * @return Array of velocities [frontLeft, frontRight, backLeft, backRight] in ticks/sec
     */
    public double[] getMotorVelocities() {
        return new double[] {
            frontLeftMotor.getVelocity(),
            frontRightMotor.getVelocity(),
            backLeftMotor.getVelocity(),
            backRightMotor.getVelocity()
        };
    }
    
    // ========== PID CALIBRATION INTERFACE ==========
    
    /**
     * Updates X-axis position PID parameters for real-time calibration
     * @param kP Proportional gain
     * @param kI Integral gain  
     * @param kD Derivative gain
     */
    public void updatePositionXPID(double kP, double kI, double kD) {
        if (positionXController != null) {
            positionXController.setPIDGains(kP, kI, kD);
        }
    }
    
    /**
     * Updates Y-axis position PID parameters for real-time calibration
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     */
    public void updatePositionYPID(double kP, double kI, double kD) {
        if (positionYController != null) {
            positionYController.setPIDGains(kP, kI, kD);
        }
    }
    
    /**
     * Updates distance PID parameters for real-time calibration (2-PID architecture)
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     */
    public void updateDistancePID(double kP, double kI, double kD) {
        if (distanceController != null) {
            distanceController.setPIDGains(kP, kI, kD);
        }
    }
    
    /**
     * Updates heading PID parameters for real-time calibration
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     */
    public void updateHeadingPID(double kP, double kI, double kD) {
        if (headingController != null) {
            headingController.setPIDGains(kP, kI, kD);
        }
    }
    
    /**
     * Gets current X-axis position PID parameters
     * @return Array of [kP, kI, kD] values
     */
    public double[] getPositionXPID() {
        if (positionXController != null) {
            return new double[] {
                positionXController.getKp(),
                positionXController.getKi(),
                positionXController.getKd()
            };
        }
        return new double[] {0, 0, 0};
    }
    
    /**
     * Gets current Y-axis position PID parameters
     * @return Array of [kP, kI, kD] values
     */
    public double[] getPositionYPID() {
        if (positionYController != null) {
            return new double[] {
                positionYController.getKp(),
                positionYController.getKi(),
                positionYController.getKd()
            };
        }
        return new double[] {0, 0, 0};
    }
    
    /**
     * Gets current heading PID parameters
     * @return Array of [kP, kI, kD] values
     */
    public double[] getHeadingPID() {
        if (headingController != null) {
            return new double[] {
                headingController.getKp(),
                headingController.getKi(),
                headingController.getKd()
            };
        }
        return new double[] {0, 0, 0};
    }
    
    // ========== PID CONTROLLER ACCESS ==========
    
    /**
     * Gets the position X PID controller for advanced calibration
     * @return Position X PID controller
     */
    public PIDController getPositionXController() {
        return positionXController;
    }
    
    /**
     * Gets the position Y PID controller for advanced calibration
     * @return Position Y PID controller
     */
    public PIDController getPositionYController() {
        return positionYController;
    }
    
    /**
     * Gets the distance PID controller for 2-PID architecture calibration
     * @return Distance PID controller (unified linear motion control)
     */
    public PIDController getDistanceController() {
        return distanceController;
    }
    
    /**
     * Gets the heading PID controller for advanced calibration
     * @return Heading PID controller
     */
    public PIDController getHeadingController() {
        return headingController;
    }
    
    // ========== CONFIGURATION ==========
    
    /**
     * Resets all motor encoders to zero
     */
    public void resetEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Return to velocity control mode
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    /**
     * Reconfigures motors (useful for runtime configuration changes)
     */
    public void reconfigureMotors() {
        configureMotors();
    }
}
