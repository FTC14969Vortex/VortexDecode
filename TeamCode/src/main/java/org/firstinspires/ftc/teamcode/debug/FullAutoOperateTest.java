package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.calibration.RobotConstants;
import org.firstinspires.ftc.teamcode.motion.FieldPose;
import org.firstinspires.ftc.teamcode.motion.FieldPositions;
import org.firstinspires.ftc.teamcode.subsystems.BaseMotion;
import org.firstinspires.ftc.teamcode.subsystems.FlyWheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Flipper;
import org.firstinspires.ftc.teamcode.vision.CameraServo;
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
@Autonomous(name = "Full Auto Operate Test 0.38", group = "Debug")
public class FullAutoOperateTest extends LinearOpMode {

    // ========== SUBSYSTEMS ==========
    private BaseMotion baseMotion;
    private FlyWheel flyWheel;
    private Intake intake;
    private Kicker kicker;
    private Flipper flipper;
    private CameraServo cameraServo;



    // ========== VISION SYSTEM ==========
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    // ========== MOTION PARAMETERS ==========
    private static final double TRAVEL_VELOCITY = 50.0; // inches/sec for movement
    private static final double INTAKE_VELOCITY = 35.0; // inches/sec during intake

    private static final double SHOOTING_VELOCITY = 1100.0; // RPM for shooting

    // ========== SHOOTING PARAMETERS ==========
    private static final int NUM_SHOTS = 3;
    private static final double INITIAL_FLIPPER_ANGLE = 120.0;
    private static final double ANGLE_INCREMENT = 30.0;
    private static final int KICKER_OPEN_DELAY_MS = 100; //was 300
    private static final int BASE_FLIPPER_DELAY_MS = 200;
    private static final int FLIPPER_DELAY_INCREMENT_MS = 50;
    private static final int FLIPPER_RESET_DELAY_MS = 200;

    // ========== INTAKE PARAMETERS ==========
    private static final double INTAKE_FULL_POWER = 1.0;
    private static final double INTAKE_SHOOTING_POWER = 1.0;

    // ========== FLYWHEEL PARAMETERS ==========
    private static final long FLYWHEEL_SPINUP_TIMEOUT = 2000; // ms

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

        // Set reference point to START_NEAR position
        baseMotion.setReferencePoint(RobotConstants.BACK_RIGHT_CORNER);
        baseMotion.setReferencePointToPosition(FieldPositions.START_NEAR);

        telemetryCurrentPose("Initializing");

        waitForStart();

        if (isStopRequested()) return;

        // ========== AUTONOMOUS SEQUENCE ==========

        try {

            moveToShootingPosition(); // the pre-loaded shooting
            telemetryCurrentPose("After Shooting");
            // Step 2-4: Intake and shoot for positions 1, 2, 3
            for (int i = 1; i <= 3; i++) {
                intakeAndShoot(i);
                telemetryCurrentPose("After Intake and Shoot");
            }
            park();

        } catch (InterruptedException e) {
            telemetry.addLine("❌ Autonomous interrupted!");
            telemetry.update();
            Thread.currentThread().interrupt();
        } finally {

            cleanupSubsystems();
        }
    }

    private void moveToShootingPosition() throws InterruptedException {
        telemetryCurrentPose("Moving to Shooting Position");

        // Set intake to travel power
        intake.setIntakePower(INTAKE_SHOOTING_POWER);

        // Create preparation thread for parallel execution
        Thread preparationThread = new Thread(() -> {
            try {
                // Close kicker gate
                kicker.setGatePosition(Kicker.GATE_CLOSE);
                Thread.sleep(KICKER_OPEN_DELAY_MS);
                flyWheel.setToShootingVelocity(SHOOTING_VELOCITY, FLYWHEEL_SPINUP_TIMEOUT);

            } catch (Exception e) {
                telemetry.addLine("❌ Preparation thread error: " + e.getMessage());
                telemetry.update();
            }
        });

        preparationThread.start();
        baseMotion.moveToPose(FieldPositions.SHOOTING_NEAR, TRAVEL_VELOCITY);
        preparationThread.join();

        // flywheel should be ready
      //  sleep(100);
        shoot();
    }

    /**
     * Execute full shooting sequence (based on FlywheelVelocityTest)
     */
    private void shoot() throws InterruptedException {
        telemetryCurrentPose("Shooting");

        // Create flywheel maintenance thread to continuously maintain target velocity
        final boolean[] maintainFlywheel = {true};
        Thread flywheelMaintenanceThread = new Thread(() -> {
            while (maintainFlywheel[0] && !Thread.currentThread().isInterrupted()) {
                try {
                    flyWheel.setToShootingVelocity(SHOOTING_VELOCITY, FLYWHEEL_SPINUP_TIMEOUT);
                    Thread.sleep(50); // Check every 100ms
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                } catch (Exception e) {
                    // Log error but continue
                    telemetry.addLine("⚠️ Flywheel maintenance error: " + e.getMessage());
                    telemetry.update();
                }
            }
        });

        // Start flywheel maintenance thread
        flywheelMaintenanceThread.start();

        try {
            // Open gate for shooting
            kicker.setGatePosition(Kicker.GATE_SHOOT);
            //sleep(KICKER_OPEN_DELAY_MS);

            // Execute 3 flipper shots
            for (int shotNumber = 0; shotNumber < NUM_SHOTS; shotNumber++) {
                // Calculate flipper angle for this shot (120°, 150°, 180°)
                double currentFlipperAngle = INITIAL_FLIPPER_ANGLE + (shotNumber * ANGLE_INCREMENT);
                int flipperWaitTime = BASE_FLIPPER_DELAY_MS + (shotNumber * FLIPPER_DELAY_INCREMENT_MS);
                
                // Turn flipper
                flipper.turnFlipper(currentFlipperAngle);
                sleep(flipperWaitTime);

                // Reset flipper to starting position
                flipper.resetFlipper();
                sleep(flipperWaitTime);
            }

        } finally {
            // Stop flywheel maintenance thread
            maintainFlywheel[0] = false;
            flywheelMaintenanceThread.interrupt();
            
            try {
                flywheelMaintenanceThread.join(100); // Wait for thread to finish
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }

            // Cleanup
            kicker.setGatePosition(Kicker.GATE_CLOSE);
            flyWheel.fastStop();
            kicker.setGatePosition(Kicker.GATE_INTAKE);
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
        moveToShootingPosition();
    }

    /**
     * Park at PARKING_NEAR
     */
    private void park() throws InterruptedException {
        intake.stopIntake();
        baseMotion.moveToPose(FieldPositions.PARKING_NEAR, TRAVEL_VELOCITY);
    }

    /**
     * Align robot to optimal shooting angle calculated by CameraServo
     * Uses the shooting angle from AprilTag detection or predicted angle
     */
    private void alignToShootingAngle() throws InterruptedException {

        // Update CameraServo to get latest shooting angle
        cameraServo.update();

        // Get the optimal shooting angle from CameraServo
        double targetAngle = cameraServo.getShootingAngle();

        // Get current robot pose
        FieldPose currentPose = baseMotion.getCurrentPose();

        // Calculate angle difference
        double angleDifference = targetAngle - currentPose.heading;

        telemetry.addData("Current heading", "%.1f°", currentPose.heading);
        telemetry.addData("Target shooting angle", "%.1f°", targetAngle);
        telemetry.addData("Angle adjustment", "%.1f°", angleDifference);
        telemetry.addLine("✅ Aligned to shooting angle");
        telemetry.update();
    }


    /**
     * Get shooting velocity based on AprilTag distance (backup method)
     * CameraServo handles both detected and predicted distances automatically
     */
    private double getShootingVelocity() {
        // Update camera servo to process detections and calculate flywheel velocity
        cameraServo.update();

        // CameraServo automatically calculates velocity from either detected or predicted distance
        double velocity = cameraServo.getFlywheelVelocity();

        // Check if AprilTag was recently detected for telemetry info
        long timeSinceDetection = cameraServo.getTimeSinceLastDetection();

        if (timeSinceDetection < 1000) {
            telemetry.addData("🎯 AprilTag detected", "Distance: %.1f in", cameraServo.getLastDetectedDistance());
            telemetry.addData("Calculated velocity", "%.0f RPM", velocity);
        } else {
            telemetry.addData("🎯 Predicted distance", "%.0f RPM (odometry-based)", velocity);
        }
        telemetry.update();

        return velocity;
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