package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.calibration.RobotConstants;
import org.firstinspires.ftc.teamcode.motion.FieldPose;
import org.firstinspires.ftc.teamcode.motion.FieldPositions;
import org.firstinspires.ftc.teamcode.motion.MotionExecutor;
import org.firstinspires.ftc.teamcode.subsystems.BaseMotion;
import org.firstinspires.ftc.teamcode.subsystems.FlyWheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Flipper;
import org.firstinspires.ftc.teamcode.vision.CameraServo;
import org.firstinspires.ftc.teamcode.utils.RobotOperations;


/**
 * Blue Auto Far - Blue Alliance Far Start Autonomous
 *
 * Complete autonomous routine for Blue Alliance starting far from the goal:
 * 1. Start at START_FAR
 * 2. Move to SHOOTING_FAR and shoot (3 preloaded samples)
 * 3. Move to INTAKE_LOADING_START
 * 4. Drive forward 1 sec for intake
 * 5. Return to SHOOTING_FAR and shoot
 * 6. Move to INTAKE_LOADING_START
 * 7. Drive forward 1 sec for intake
 * 8. Return to SHOOTING_FAR and shoot
 * 9. Stop and dump odometry coordinates
 *
 * INTAKE BEHAVIOR:
 * - Full power (1.0) when actively intaking
 * - Half power (0.5) when traveling to shooting position
 *
 * SHOOTING PREPARATION:
 * - When traveling to shooting: close kicker gate + ramp up flywheel
 * - Flywheel velocity determined by AprilTag distance (Tag 20 for blue)
 * - At shooting position: execute full shooting sequence
 *
 * REFERENCE:
 * - Based on FullAutoOperateTest structure
 * - Uses RobotOperations for all smart movement (shooting_far, intake_loading_start)
 * - Uses FieldPositions for all coordinates
 */
@Autonomous(name = "Blue Auto Far 0.2", group = "Debug")
public class BlueAutoFar extends LinearOpMode {

    // ========== SUBSYSTEMS ==========
    private BaseMotion baseMotion;
    private FlyWheel flyWheel;
    private Intake intake;
    private Kicker kicker;
    private Flipper flipper;
    private CameraServo cameraServo;
    private RobotOperations robotOperations;
    private static final double INTAKE_VELOCITY = 35.0; // inches/sec during intake

    // ========== INTAKE PARAMETERS ==========
    private static final double INTAKE_FULL_POWER = 1.0;
    private static final double INTAKE_TIME = 1.0; // 1 second intake time


    @Override
    public void runOpMode() throws InterruptedException {

        cameraServo = new CameraServo();
        
        // Initialize subsystems
        baseMotion = new BaseMotion();
        baseMotion.init(this);

        flyWheel = new FlyWheel();
        flyWheel.init(this);

        intake = new Intake();
        intake.init(this);
        intake.stopIntake();

        kicker = new Kicker();
        kicker.init(hardwareMap);
        kicker.setGatePosition(Kicker.GATE_INTAKE); // Start in intake position

        flipper = new Flipper();
        flipper.init(hardwareMap);
        flipper.resetFlipper();

        // Initialize CameraServo with full motion integration for proper pose-based aiming
        cameraServo.init(
            hardwareMap, 
            baseMotion.getMotionExecutor().getMotionState().getOdometryManager(),
            baseMotion.getMotionExecutor().getCoordinateTransformer(),
            baseMotion.getMotionExecutor()
        );
        cameraServo.moveToCenter(); // Keep servo at center position for this auto
        cameraServo.setAutoOdometryCorrection(false); // Disable autocorrection for pure odometry-based calculation
        cameraServo.update();
        cameraServo.startThread();

        // Initialize RobotOperations utility
        robotOperations = new RobotOperations();
        robotOperations.init(baseMotion, flyWheel, intake, kicker, flipper, cameraServo,
                baseMotion.getMotionExecutor().getCoordinateTransformer(), this);

        // Set blue alliance
        robotOperations.setAlliance(true); // Blue alliance

        // Set reference point to START_FAR position
        baseMotion.setControlMode(MotionExecutor.ControlMode.PURE_FEEDBACK);
        baseMotion.setReferencePoint(RobotConstants.BACK_RIGHT_CORNER);
        baseMotion.setReferencePointToPosition(FieldPositions.START_FAR);

        telemetryCurrentPose("Initializing at START_FAR");

        waitForStart();

        if (isStopRequested()) return;

        // ========== AUTONOMOUS SEQUENCE ==========

        boolean autoCompletedSuccessfully = false;

        try {

            // Step 1: Initial shooting with preloaded balls from START_FAR
            robotOperations.moveToLocation("shooting_far", 2000);
            robotOperations.shoot(true);
            telemetryCurrentPose("After Initial Shooting");

            // Step 2-3: First intake and shoot cycle
            intakeAndShoot();
            telemetryCurrentPose("After First Intake and Shoot");

            // Step 4-5: Second intake and shoot cycle
            intakeAndShoot();
            telemetryCurrentPose("After Second Intake and Shoot");

            // If we reach here, autonomous completed successfully
            autoCompletedSuccessfully = true;

        } catch (InterruptedException e) {
            telemetry.addLine("❌ Autonomous interrupted!");
            telemetry.update();
            Thread.currentThread().interrupt();
        } finally {
            // Save odometry position for TeleOp to use
            // autoCompletedSuccessfully = true only if we completed all steps
            // If we timed out or were interrupted, it will be false
            RobotOperations.saveOdometryAtAutoEnd(hardwareMap, baseMotion, autoCompletedSuccessfully);

            String completionStatus = autoCompletedSuccessfully ? "Completed" : "Timed Out/Interrupted";
            telemetryCurrentPose("Auto End - " + completionStatus + " - Saved for TeleOp");

            cleanupSubsystems();
        }
    }

    /**
     * Intake and shoot sequence
     * 1. Move to INTAKE_LOADING_START
     * 2. Drive forward 1 sec for intake
     * 3. Return to SHOOTING_FAR and shoot
     */
    private void intakeAndShoot() throws InterruptedException {
        telemetryCurrentPose("Starting Intake and Shoot Cycle");

        // Move to intake loading start position using smart movement
        MotionExecutor.MotionResult result = robotOperations.moveToLocation("intake_loading_start");

        // Display motion results for debugging
        telemetry.addData("Motion to Intake", "Success: %s", result.success);
        telemetry.addData("Position Error", "%.2f inches", result.finalPositionError);
        telemetry.addData("Heading Error", "%.1f degrees", result.finalHeadingError);
        telemetry.addData("Motion Duration", "%.0f ms", result.executionTimeMs);
        telemetry.addData("Motion Status", result.failureReason);
        telemetry.update();

        // Start intake at full power and move forward for 1 second
        kicker.setGatePosition(Kicker.GATE_INTAKE);
        intake.setIntakePower(INTAKE_FULL_POWER);

        // Move forward while intaking for 1 second
        baseMotion.timeMotion(BaseMotion.Direction.FORWARD, INTAKE_VELOCITY, INTAKE_TIME);

        // Move to shooting position and shoot
        MotionExecutor.MotionResult result1 = robotOperations.moveToLocation("shooting_far", 2000);
        
        // Display motion results for debugging
        telemetry.addData("Motion to Shooting", "Success: %s", result1.success);
        telemetry.addData("Position Error", "%.2f inches", result1.finalPositionError);
        telemetry.addData("Heading Error", "%.1f degrees", result1.finalHeadingError);
        telemetry.addData("Motion Duration", "%.0f ms", result1.executionTimeMs);
        telemetry.addData("Motion Status", result1.failureReason);
        telemetry.update();

        cameraServo.update(); // at shooting position - update camera servo

        robotOperations.shoot(true); // use cameraservo for align + distance
    }

    private void cleanupSubsystems() {
        // Clean shutdown
        if (flyWheel != null) {
            flyWheel.stop();
        }
        if (intake != null) {
            intake.stopIntake();
        }
        if (cameraServo != null) {
            cameraServo.stopThread();
            cameraServo.cleanup();
        }
    }

    private void telemetryCurrentPose(String message) {
        telemetry.addData("", message);
        FieldPose currPose = baseMotion.getCurrentPose();
        telemetry.addData("Position", "x: %.2f, y: %.2f, h: %.2f°",
                currPose.x, currPose.y, currPose.heading);
        telemetry.update();
    }
}
