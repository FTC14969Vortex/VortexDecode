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
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * Full Auto Operate Test - Blue Alliance Near Start
 *
 * Complete autonomous routine for Blue Alliance starting near the goal:
 * 1. Start at START_NEAR
 * 2. Move to SHOOTING_NEAR and shoot (3 preloaded samples)
 * 3. Move to INTAKE_1_START → INTAKE_1_FINISH (intake coral)
 * 4. Return to SHOOTING_NEAR and shoot
 * 5. Move to INTAKE_2_START → INTAKE_2_FINISH (intake coral)
 * 6. Return to SHOOTING_NEAR and shoot
 * 7. Move to INTAKE_3_START → INTAKE_3_FINISH (intake coral)
 * 8. Return to SHOOTING_NEAR and shoot
 * 9. Park at PARKING_NEAR
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
 * - Uses FlywheelVelocityTest shooting sequence
 * - Uses BaseMotion for all movements
 * - Uses FieldPositions for all coordinates
 */
@Autonomous(name = "Full Auto Operate Test 0.42", group = "Debug")
public class FullAutoOperateTest extends LinearOpMode {

    // ========== SUBSYSTEMS ==========
    private BaseMotion baseMotion;
    private FlyWheel flyWheel;
    private Intake intake;
    private Kicker kicker;
    private Flipper flipper;
    private CameraServo cameraServo;
    private RobotOperations robotOperations;

    // ========== VISION SYSTEM ==========
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    // ========== MOTION PARAMETERS ==========
    private static final double TRAVEL_VELOCITY = 50.0; // inches/sec for movement
    private static final double INTAKE_VELOCITY = 35.0; // inches/sec during intake

    // ========== INTAKE PARAMETERS ==========
    private static final double INTAKE_FULL_POWER = 1.0;


    @Override
    public void runOpMode() throws InterruptedException {

        initVisionSystem();

        cameraServo = new CameraServo();
        cameraServo.init(hardwareMap, aprilTagProcessor);
        cameraServo.setTargetTag(20); // Blue AprilTag
        cameraServo.moveToCenter(); // Keep servo at center position for this auto
        cameraServo.setAutoOdometryCorrection(false); // Disable autocorrection for pure odometry-based calculation

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

        // Initialize RobotOperations utility
        robotOperations = new RobotOperations();
        robotOperations.init(baseMotion, flyWheel, intake, kicker, flipper, cameraServo,
                baseMotion.getMotionExecutor().getCoordinateTransformer(), this);
        robotOperations.setAlliance(true); // Blue alliance

        // Set reference point to START_NEAR position
        baseMotion.setReferencePoint(RobotConstants.BACK_RIGHT_CORNER);
        baseMotion.setReferencePointToPosition(FieldPositions.START_NEAR);

        telemetryCurrentPose("Initializing");

        waitForStart();

        if (isStopRequested()) return;

        // ========== AUTONOMOUS SEQUENCE ==========

        try {

            // Initial shooting with preloaded balls
            robotOperations.moveToLocation("shooting_near");
            robotOperations.shoot();
            telemetryCurrentPose("After Shooting");

            // Step 2-4: Intake and shoot for positions 1, 2, 3
            for (int i = 1; i <= 3; i++) {
                intakeAndShoot(i);
                telemetryCurrentPose("After Intake and Shoot");
            }

            // Park at end position
            robotOperations.moveToLocation("parking_near");

        } catch (InterruptedException e) {
            telemetry.addLine("❌ Autonomous interrupted!");
            telemetry.update();
            Thread.currentThread().interrupt();
        } finally {

            cleanupSubsystems();
        }
    }

    /**
     * Intake and shoot sequence for a specific intake position (1, 2, or 3)
     */
    private void intakeAndShoot(int intakeNumber) throws InterruptedException {
        telemetryCurrentPose("Intake and Shoot");

        double intakeTime = 1.0;

        // Get intake positions based on number
        FieldPose intakeStart, intakeFinish;

        switch (intakeNumber) {
            case 1:
                intakeTime = 0.8;
                intakeStart = FieldPositions.INTAKE_1_START;
                intakeFinish = FieldPositions.INTAKE_1_FINISH;
                break;
            case 2:
                intakeTime = 1.0;
                intakeStart = FieldPositions.INTAKE_2_START;
                intakeFinish = FieldPositions.INTAKE_2_FINISH;
                break;
            case 3:
                intakeTime = 1.0;
                intakeStart = FieldPositions.INTAKE_3_START;
                intakeFinish = FieldPositions.INTAKE_3_FINISH;
                break;
            default:
                telemetry.addLine("❌ Invalid intake number: " + intakeNumber);
                telemetry.update();
                return;
        }

        // Move to intake start position
        baseMotion.moveToPose(intakeStart, TRAVEL_VELOCITY);

        // Start intake at full power and move forward for specified time

        kicker.setGatePosition(Kicker.GATE_INTAKE);
        intake.setIntakePower(INTAKE_FULL_POWER);

        // Move forward while intaking for specified time
        baseMotion.timeMotion(BaseMotion.Direction.FORWARD, INTAKE_VELOCITY, intakeTime);

        // Move to shooting position and shoot
        robotOperations.moveToLocation("shooting_near");
        robotOperations.shoot(); // use distance based velocity + alignment

    }


    /**
     * Initialize vision system (AprilTag processor and VisionPortal)
     */
    private void initVisionSystem() {
        // Initialize AprilTag processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setOutputUnits(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH,
                        org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES)
                .build();

        // Initialize vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();
    }

    private void cleanupSubsystems() {
        // Clean shutdown
        if (flyWheel != null) {
            flyWheel.stop();
        }
        if (intake != null) {
            intake.stopIntake();
        }
        if (visionPortal != null) {
            visionPortal.close();
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