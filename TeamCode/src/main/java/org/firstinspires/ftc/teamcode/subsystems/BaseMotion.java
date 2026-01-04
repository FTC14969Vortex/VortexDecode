package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.calibration.RobotConstants;
import org.firstinspires.ftc.teamcode.external.gobilida.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.motion.MotionConfig;
import org.firstinspires.ftc.teamcode.motion.MotionExecutor;
import org.firstinspires.ftc.teamcode.motion.MotionExecutor.ControlMode;
import org.firstinspires.ftc.teamcode.motion.FieldPose;
import org.firstinspires.ftc.teamcode.motion.FieldPositions;
import org.firstinspires.ftc.teamcode.motion.CoordinateTransformer;
import org.firstinspires.ftc.teamcode.motion.MotionState;
import org.firstinspires.ftc.teamcode.calibration.RobotConstants.ComponentPosition;

/**
 * BaseMotion subsystem for robot base movement control.
 *
 * This subsystem provides a high-level interface to the motion control system,
 * delegating to MotionExecutor for autonomous movements and providing teleop
 * control with field-centric and robot-centric modes.
 *
 * Usage pattern (like Chassis):
 * 1. Create instance: BaseMotion baseMotion = new BaseMotion();
 * 2. Initialize: baseMotion.init(this);
 * 3. Use methods for control
 */
public class BaseMotion {

    // ========== HARDWARE REFERENCES ==========

    private OpMode opMode;
    private MotionExecutor motionExecutor;

    // Hardware components
    private DcMotorEx frontLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx backRightMotor;
    private GoBildaPinpointDriver odometry;

    // ========== CONTROL MODE MANAGEMENT ==========

    public enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }

    public enum Direction {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT
    }

    private DriveMode driveMode = DriveMode.ROBOT_CENTRIC;

    // ========== TIMING ==========

    private ElapsedTime timer = new ElapsedTime();

    // ========== INITIALIZATION ==========

    /**
     * Initializes the BaseMotion subsystem
     *
     * @param opMode The OpMode instance for hardware access
     */
    public void init(OpMode opMode) {
        this.opMode = opMode;

        // Get hardware references
        frontLeftMotor = opMode.hardwareMap.get(DcMotorEx.class, "frontLeftDrive");
        frontRightMotor = opMode.hardwareMap.get(DcMotorEx.class, "frontRightDrive");
        backLeftMotor = opMode.hardwareMap.get(DcMotorEx.class, "backLeftDrive");
        backRightMotor = opMode.hardwareMap.get(DcMotorEx.class, "backRightDrive");
        odometry = opMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Create MotionExecutor for delegation
        motionExecutor = new MotionExecutor(
                frontLeftMotor, frontRightMotor,
                backLeftMotor, backRightMotor,
                odometry
        );

        opMode.telemetry.addData("BaseMotion", "Initialized successfully");
        opMode.telemetry.update();
    }

    // ========== AUTONOMOUS MOTION DELEGATION ==========

    /**
     * Moves robot to absolute field position with target heading
     *
     * @param targetX Target X coordinate (inches)
     * @param targetY Target Y coordinate (inches)
     * @param targetHeading Target heading (degrees)
     * @param velocity Maximum velocity (inches/sec)
     * @return MotionResult indicating success/failure
     */
    public MotionExecutor.MotionResult moveToPose(double targetX, double targetY, double targetHeading, double velocity) {
        return motionExecutor.moveToPose(targetX, targetY, targetHeading, velocity);
    }

    /**
     * Moves robot to a FieldPose position
     *
     * @param targetPose Target field pose
     * @param velocity Maximum velocity (inches/sec)
     * @return MotionResult indicating success/failure
     */
    public MotionExecutor.MotionResult moveToPose(FieldPose targetPose, double velocity) {
        return motionExecutor.moveToPose(targetPose.x, targetPose.y, targetPose.heading, velocity);
    }

    /**
     * Moves robot in straight line at specified angle for specified distance
     *
     * @param angle Movement angle (degrees, 0 = +Y field axis)
     * @param distance Distance to move (inches)
     * @param velocity Maximum velocity (inches/sec)
     * @return MotionResult indicating success/failure
     */
    public MotionExecutor.MotionResult linearMove(double angle, double distance, double velocity) {
        return motionExecutor.linearMove(angle, distance, velocity);
    }

    /**
     * Moves robot in straight line with optional target heading
     *
     * @param angle Movement angle (degrees)
     * @param distance Distance to move (inches)
     * @param targetHeading Target heading during move (degrees)
     * @param velocity Maximum velocity (inches/sec)
     * @return MotionResult indicating success/failure
     */
    public MotionExecutor.MotionResult linearMove(double angle, double distance, double targetHeading, double velocity) {
        return motionExecutor.linearMove(angle, distance, targetHeading, velocity);
    }

    /**
     * Rotates robot to target heading
     *
     * @param targetHeading Target heading (degrees)
     * @param angularVelocity Maximum angular velocity (degrees/sec)
     * @return MotionResult indicating success/failure
     */
    public MotionExecutor.MotionResult rotate(double targetHeading, double angularVelocity) {
        // Note: MotionExecutor.rotate() only takes targetHeading, angularVelocity is handled internally
        return motionExecutor.rotate(targetHeading);
    }

    // ========== TIME-BASED MOTION ==========

    /**
     * Moves robot for a specified time in a given direction (robot-centric by default)
     *
     * @param direction Direction to move (FORWARD, BACKWARD, LEFT, RIGHT)
     * @param velocity Velocity (inches/sec)
     * @param duration Duration to move (seconds)
     */
    public void timeMotion(Direction direction, double velocity, double duration) {
        double vx = 0, vy = 0;

        switch (direction) {
            case FORWARD:
                vx = velocity;
                break;
            case BACKWARD:
                vx = -velocity;
                break;
            case LEFT:
                vy = velocity;
                break;
            case RIGHT:
                vy = -velocity;
                break;
        }

        timeMotion(vx, vy, 0, duration);
    }

    /**
     * Moves robot for a specified time at a specific angle (robot-centric by default)
     *
     * @param angle Movement angle (degrees, 0 = forward)
     * @param velocity Velocity (inches/sec)
     * @param duration Duration to move (seconds)
     */
    public void timeMotion(double angle, double velocity, double duration) {
        double angleRad = Math.toRadians(angle);
        double vx = velocity * Math.sin(angleRad);
        double vy = velocity * Math.cos(angleRad);

        timeMotion(vx, vy, 0, duration);
    }

    /**
     * Moves robot for a specified time with specific velocity components (robot-centric by default)
     *
     * @param vx X velocity (inches/sec)
     * @param vy Y velocity (inches/sec)
     * @param omega Angular velocity (degrees/sec)
     * @param duration Duration to move (seconds)
     */
    public void timeMotion(double vx, double vy, double omega, double duration) {
        int durationMs = (int)(duration * 1000);
        motionExecutor.combinedMotion(vx, vy, omega, MotionState.CoordinateMode.ROBOT_CENTRIC, durationMs, 0, false, null);
    }

    // ========== TELEOP CONTROL ==========

    /**
     * Sets motor powers for teleop control with gamepad inputs
     * Handles field-centric and robot-centric mode switching
     *
     * @param leftX Left stick X (strafe)
     * @param leftY Left stick Y (forward/backward)
     * @param rightX Right stick X (rotation)
     */
    public void setMotorPowers(double leftX, double leftY, double rightX) {
        // Apply deadzone
        leftX = applyDeadzone(leftX, 0.1);
        leftY = applyDeadzone(leftY, 0.1);
        rightX = applyDeadzone(rightX, 0.1);

        // Convert to velocity (scale by max velocity)
        double maxLinearVelocity = RobotConstants.MAX_THEORETICAL_LINEAR_VELOCITY; // inches/sec
        double maxAngularVelocity = RobotConstants.MAX_THEORETICAL_ANGULAR_VELOCITY; // degrees/sec

        double vy = leftX * maxLinearVelocity;
        double vx = leftY * maxLinearVelocity; // Invert Y for intuitive control
        double omega = rightX * maxAngularVelocity;

        // Apply velocity based on drive mode
        MotionState.CoordinateMode coordinateMode =
                (driveMode == DriveMode.FIELD_CENTRIC) ?
                        MotionState.CoordinateMode.FIELD_CENTRIC :
                        MotionState.CoordinateMode.ROBOT_CENTRIC;

        motionExecutor.setVelocity(vx, vy, omega, coordinateMode);
    }

    /**
     * Applies deadzone to joystick input
     *
     * @param input Raw joystick input
     * @param deadzone Deadzone threshold
     * @return Processed input with deadzone applied
     */
    private double applyDeadzone(double input, double deadzone) {
        if (Math.abs(input) < deadzone) {
            return 0.0;
        }
        return input;
    }

    /**
     * Sets the drive mode (field-centric or robot-centric)
     *
     * @param mode Drive mode to set
     */
    public void setDriveMode(DriveMode mode) {
        this.driveMode = mode;
    }

    /**
     * Gets the current drive mode
     *
     * @return Current drive mode
     */
    public DriveMode getDriveMode() {
        return driveMode;
    }

    /**
     * Toggles between field-centric and robot-centric drive modes
     */
    public void toggleDriveMode() {
        driveMode = (driveMode == DriveMode.ROBOT_CENTRIC) ?
                DriveMode.FIELD_CENTRIC : DriveMode.ROBOT_CENTRIC;
    }

    /**
     * Stops all robot motion
     */
    public void stopRobot() {
        motionExecutor.setVelocity(0, 0, 0, MotionState.CoordinateMode.ROBOT_CENTRIC);
    }

    // ========== REFERENCE POINT MANAGEMENT ==========

    /**
     * Sets the active reference point
     *
     * @param referencePoint ComponentPosition to use as reference point
     */
    public void setReferencePoint(ComponentPosition referencePoint) {
        FieldPositions.setActiveReferencePoint(referencePoint);
    }

    /**
     * Sets the reference point initial position in field coordinates
     *
     * @param position Field position where reference point should be initially positioned
     */
    public void setReferencePointInitialPosition(FieldPose position) {
        FieldPositions.setReferencePointInitialPosition(position, motionExecutor);
    }

    /**
     * Sets the reference point to field origin (0,0,0)
     */
    public void setReferencePointToFieldOrigin() {
        FieldPositions.setReferencePointToFieldOrigin(motionExecutor);
    }

    /**
     * Sets the reference point to any predefined position
     *
     * @param position Any FieldPose (can be BLUE or RED position)
     */
    public void setReferencePointToPosition(FieldPose position) {
        FieldPositions.setReferencePointToPosition(position, motionExecutor);
    }

    // ========== CONTROL MODE MANAGEMENT ==========

    /**
     * Sets the control mode for motion execution
     *
     * @param mode Control mode (PURE_FEEDBACK, PURE_FEEDFORWARD, or HYBRID)
     */
    public void setControlMode(ControlMode mode) {
        motionExecutor.setControlMode(mode);
    }

    /**
     * Gets the current control mode
     *
     * @return Current control mode
     */
    public ControlMode getControlMode() {
        return motionExecutor.getControlMode();
    }

    /**
     * Sets the default control mode
     *
     * @param mode Default control mode to use
     */
    public void setDefaultControlMode(ControlMode mode) {
        motionExecutor.setDefaultControlMode(mode);
    }

    /**
     * Resets to the default control mode
     */
    public void resetToDefaultControlMode() {
        motionExecutor.resetToDefaultControlMode();
    }

    /**
     * Sets the hybrid gain for HYBRID control mode
     *
     * @param gain Hybrid gain (0.0 = 100% feedback, 1.0 = 100% feedforward)
     */
    public void setHybridGain(double gain) {
        motionExecutor.setHybridGain(gain);
    }

    /**
     * Gets the current hybrid gain
     *
     * @return Current hybrid gain value
     */
    public double getHybridGain() {
        return motionExecutor.getHybridGain();
    }

    // ========== STATUS AND TELEMETRY ==========

    public FieldPose getCurrentPose() {
        Pose2D currentPose = motionExecutor.getMotionState().getCurrentPose();

        return new FieldPose(currentPose.getX(DistanceUnit.INCH),
                currentPose.getY(DistanceUnit.INCH),
                currentPose.getHeading(AngleUnit.DEGREES));
    }

    /**
     * Updates telemetry with BaseMotion status
     */
    public void updateTelemetry() {
        FieldPose currentPose = getCurrentPose();
        ControlMode currentControlMode = getControlMode();

        opMode.telemetry.addData("Drive Mode", driveMode);
        opMode.telemetry.addData("Control Mode", currentControlMode);

        // Show hybrid gain details when in HYBRID mode
        if (currentControlMode == ControlMode.HYBRID) {
            double hybridGain = getHybridGain();
            int feedforwardPercent = (int)(hybridGain * 100);
            int feedbackPercent = 100 - feedforwardPercent;
            opMode.telemetry.addData("Hybrid Gain", String.format("FF:%d%% FB:%d%%",
                    feedforwardPercent, feedbackPercent));
        }

        opMode.telemetry.addData("Active Reference Point", FieldPositions.getActiveReferencePoint());
        opMode.telemetry.addData("Robot Position", String.format("(%.1f, %.1f, %.1f deg)",
                currentPose.x, currentPose.y, currentPose.heading));
        opMode.telemetry.addData("Reference Initial Pos", FieldPositions.getReferencePointInitialPosition().toCompactString());
    }

    /**
     * Gets the MotionExecutor instance for advanced control
     *
     * @return MotionExecutor instance
     */
    public MotionExecutor getMotionExecutor() {
        return motionExecutor;
    }
}