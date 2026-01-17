package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.calibration.RobotConstants;
import org.firstinspires.ftc.teamcode.motion.FieldPose;
import org.firstinspires.ftc.teamcode.motion.FieldPositions;
import org.firstinspires.ftc.teamcode.motion.MotionExecutor;
import org.firstinspires.ftc.teamcode.subsystems.BaseMotion;
import org.firstinspires.ftc.teamcode.subsystems.FlyWheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Flipper;
import org.firstinspires.ftc.teamcode.utils.RobotOperations;
import org.firstinspires.ftc.teamcode.vision.CameraServo;


/**
 * TeleopBlue - Enhanced Blue Alliance Teleop with Smart Shooting Logic
 *
 * Features:
 * 1. Manual driving with gamepad1 (left stick = translation, right stick = rotation)
 * 2. Special movements with gamepad1: open_gate, loading, parking_end
 * 3. Smart shooting system with gamepad2:
 *    - Move to shooting positions: shooting_near, shooting_far
 *    - Intelligent shooting logic:
 *      * Successful movement → auto shoot
 *      * Interrupted but in zone → auto shoot
 *      * Interrupted not in zone → manual positioning required
 *    - Right bumper always available for manual shooting override
 *
 * Controls:
 * GAMEPAD 1 (Driver):
 * - Left Stick: Robot translation (strafe)
 * - Right Stick X: Robot rotation
 * - Right Bumper: Toggle flywheel preparation/shoot (1st press: prepare flywheel, 2nd press: shoot)
 * - A: Move to open_gate
 * - B: Move to loading
 * - Y: Move to parking_end
 *
 * GAMEPAD 2 (Shooter):
 * - A: Move to shooting_near (with smart shooting)
 * - B: Move to shooting_far (with smart shooting)
 * - Right Bumper: Manual shoot (always available)
 * - X: Toggle intake on/off
 */
@TeleOp(name = "A-TeleopBlue -0.50x", group = "Debug")
public class TeleopBlue extends LinearOpMode {

    // ========== SUBSYSTEMS ==========
    private BaseMotion baseMotion;
    private FlyWheel flyWheel;
    private Intake intake;
    private Kicker kicker;
    private Flipper flipper;
    private CameraServo cameraServo;
    private RobotOperations robotOperations;



    // ========== CONTROL PARAMETERS ==========
    private static final double DRIVE_SPEED_MULTIPLIER = 1.0;  // Full speed for translation
    private static final double ROTATION_SPEED_MULTIPLIER = 0.6;  // Reduced speed for rotation precision
    private static final double BUTTON_DEBOUNCE_TIME = 0.3;    // seconds

    // ========== STATE TRACKING ==========
    private ElapsedTime buttonTimer = new ElapsedTime();
    private boolean intakeOn = false;
    private String lastOperation = "None";
    private boolean isMovingToShoot = false;  // Track if we're in a shooting movement
    private boolean isFlywheelPrepared = false; // Track if flywheel is prepared for manual shooting

    // ========== BUTTON DEBOUNCING ==========
    private boolean lastGamepad1A = false;
    private boolean lastGamepad1B = false;
    private boolean lastGamepad1Y = false;
    private boolean lastGamepad1RightBumper = false;
    private boolean lastGamepad2A = false;
    private boolean lastGamepad2B = false;
    private boolean lastGamepad2X = false;
    private boolean lastGamepad2RightBumper = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // ========== INITIALIZATION ==========

        telemetry.addLine("🔧 Initializing TeleopBlue...");
        telemetry.update();

        initSubsystems();
        initRobotOperations();

        telemetry.addLine("✅ Initialization complete!");
        telemetry.addLine("🎮 Ready for teleop control");
        telemetry.addLine("");
        telemetry.addLine("GAMEPAD 1 (Driver):");
        telemetry.addLine("  Left Stick: Translation");
        telemetry.addLine("  Right Stick X: Rotation");
        telemetry.addLine("  A: Open Gate  B: Loading  Y: Parking");
        telemetry.addLine("");
        telemetry.addLine("GAMEPAD 2 (Shooter):");
        telemetry.addLine("  A: Shoot Near  B: Shoot Far");
        telemetry.addLine("  Right Bumper: Manual Shoot");
        telemetry.addLine("  X: Toggle Intake");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // ========== MAIN TELEOP LOOP ==========

        buttonTimer.reset();

        while (opModeIsActive()) {

            handleDriverControls();           //game pad1 - driver

            handleShooterControls();          //game pad2 - shooter

            updateTelemetry();

            updateButtonStates();
            sleep(20);
        }


        cleanupSubsystems();


    }


    private void initSubsystems() {
        // Initialize BaseMotion
        baseMotion = new BaseMotion();
        baseMotion.init(this);

        // Initialize FlyWheel
        flyWheel = new FlyWheel();
        flyWheel.init(this);

        // Initialize Intake
        intake = new Intake();
        intake.init(this);
        intake.startIntake();

        // Initialize Kicker
        kicker = new Kicker();
        kicker.init(hardwareMap);
        kicker.setGatePosition(Kicker.GATE_INTAKE); // Start in intake position

        // Initialize Flipper
        flipper = new Flipper();
        flipper.init(hardwareMap);
        flipper.resetFlipper();

        // Initialize CameraServo
        cameraServo = new CameraServo();
        // Initialize CameraServo with full motion integration for proper pose-based aiming
        cameraServo.init(
                hardwareMap,
                baseMotion.getMotionExecutor().getMotionState().getOdometryManager(),
                baseMotion.getMotionExecutor().getCoordinateTransformer(),
                baseMotion.getMotionExecutor()
        );
        cameraServo.moveToCenter();
        cameraServo.update();
        cameraServo.setAutoOdometryCorrection(false); // Disable autocorrection for teleop
        cameraServo.setServoMovementEnabled(false); // Disable servo movement - keep stationary for teleop
        cameraServo.startThread(); // start cameraservo background thread
    }

    /**
     * Initialize RobotOperations with all subsystems
     */
    private void initRobotOperations() {
        robotOperations = new RobotOperations();
        robotOperations.init(baseMotion, flyWheel, intake, kicker, flipper, cameraServo,
                baseMotion.getMotionExecutor().getCoordinateTransformer(), this);
        robotOperations.setAlliance(true); // Blue alliance

        // Set reference point to match autonomous setup
        baseMotion.setReferencePoint(RobotConstants.BACK_RIGHT_CORNER);
        baseMotion.setReferencePointToPosition(FieldPositions.PARKING_NEAR);

        // Try to restore odometry from autonomous
        RobotOperations.OdometryRestoreResult restoreResult =
                RobotOperations.loadOdometryFromAuto(hardwareMap, baseMotion);

        robotOperations.setVisionCorrection(true); //turn on vision correction

        if (restoreResult.success) {
            telemetry.addLine("✅ Odometry Restored from Auto!");
            telemetry.addData("Message", restoreResult.message);
            telemetry.addData("Position", "x: %.2f, y: %.2f, h: %.2f°",
                    restoreResult.restoredPose.x, restoreResult.restoredPose.y, restoreResult.restoredPose.heading);
        } else {
            telemetry.addLine("⚠️ Odometry Restore Failed");
            telemetry.addData("Reason", restoreResult.message);
            telemetry.addLine("Starting from default position");
        }
        telemetry.update();
    }

    /**
     * Handle driver controls (gamepad1)
     */
    private void handleDriverControls() {
        // ========== MANUAL DRIVING ==========

        // Get joystick inputs
        double axial = -gamepad1.left_stick_y * DRIVE_SPEED_MULTIPLIER;   // Forward/backward
        double lateral = gamepad1.left_stick_x * DRIVE_SPEED_MULTIPLIER;  // Left/right strafe
        double yaw = gamepad1.right_stick_x * ROTATION_SPEED_MULTIPLIER;  // Rotation

        // Apply manual driving (only if not in autonomous movement)
        if ( (Math.abs(axial) > 0.1 || Math.abs(lateral) > 0.1 || Math.abs(yaw) > 0.1)) {
            baseMotion.setMotorPowers(lateral, axial, yaw);  // leftX, leftY, rightX
            lastOperation = "Manual Drive";
        } else {
            // Stop motors when joystick is released (within deadzone)
            baseMotion.setMotorPowers(0, 0, 0);
        }

        // ========== SPECIAL MOVEMENTS ==========

        // A Button: Move to open_gate
        if (gamepad1.a && !lastGamepad1A && buttonTimer.seconds() > BUTTON_DEBOUNCE_TIME) {
            lastOperation = "Moving to open_gate";
            try {
                MotionExecutor.MotionResult result = robotOperations.moveToLocation("open_gate", 2000); //add timeout
                lastOperation = result.success ? "Reached open_gate" : "Failed to reach open_gate: " + result.failureReason;
            } catch (Exception e) {
                lastOperation = "Error moving to open_gate: " + e.getMessage();
            }
            buttonTimer.reset();
        }

        // B Button: Move to loading
        if (gamepad1.b && !lastGamepad1B && buttonTimer.seconds() > BUTTON_DEBOUNCE_TIME) {
            lastOperation = "Moving to loading";
            try {
                MotionExecutor.MotionResult result = robotOperations.moveToLocation("loading", 2000); // add timeout
                lastOperation = result.success ? "Reached loading" : "Failed to reach loading: " + result.failureReason;
            } catch (Exception e) {
                lastOperation = "Error moving to loading: " + e.getMessage();
            }
            buttonTimer.reset();
        }

        // Y Button: Move to parking_end
        if (gamepad1.y && !lastGamepad1Y && buttonTimer.seconds() > BUTTON_DEBOUNCE_TIME) {
            if (isFlywheelPrepared) {
                robotOperations.stopDynamicFlywheelControl();
                isFlywheelPrepared = false;
            }
            lastOperation = "Moving to parking_end";
            try {
                MotionExecutor.MotionResult result = robotOperations.moveToLocation("parking_end", 2000); //2 s, if needed press button again
                lastOperation = result.success ? "Reached parking_end" : "Failed to reach parking_end: " + result.failureReason;
            } catch (Exception e) {
                lastOperation = "Error moving to parking_end: " + e.getMessage();
            }
            buttonTimer.reset();
        }

        // Right Bumper: Toggle flywheel preparation/shoot
        // First press: Start dynamic flywheel control for manual positioning
        // Second press: Execute shoot
        if (gamepad1.right_bumper && !lastGamepad1RightBumper && buttonTimer.seconds() > BUTTON_DEBOUNCE_TIME) {
            if (!isFlywheelPrepared) {
                // First press: Start flywheel preparation
                try {
                    robotOperations.startManualFlywheelPreparation();
                    isFlywheelPrepared = true;
                    lastOperation = "Flywheel prepared - Manual positioning enabled";
                } catch (Exception e) {
                    lastOperation = "Error starting flywheel prep: " + e.getMessage();
                }
            } else {
                // Second press: Execute shoot
                try {
                    robotOperations.executeManualShoot();
                    isFlywheelPrepared = false; // Reset state after shooting
                    lastOperation = "Manual shoot executed";
                } catch (Exception e) {
                    lastOperation = "Error during manual shoot: " + e.getMessage();
                    isFlywheelPrepared = false; // Reset state on error
                }
            }
            buttonTimer.reset();
        }
    }

    /**
     * Handle shooter controls (gamepad2)
     */
    private void handleShooterControls() {
        // ========== SMART SHOOTING MOVEMENTS ==========

        // A Button: Move to shooting_near with smart shooting
        if (gamepad2.a && !lastGamepad2A && buttonTimer.seconds() > BUTTON_DEBOUNCE_TIME) {
            // Reset manual preparation state since we're switching to autonomous
            if (isFlywheelPrepared) {
                robotOperations.stopDynamicFlywheelControl();
                isFlywheelPrepared = false;
                lastOperation = "Manual prep cancelled - Switching to autonomous near";
            }
            lastOperation = robotOperations.executeSmartShooting("shooting_near", true, 4000);
            buttonTimer.reset();
        }

        // B Button: Move to shooting_far with smart shooting
        if (gamepad2.b && !lastGamepad2B && buttonTimer.seconds() > BUTTON_DEBOUNCE_TIME) {
            // Reset manual preparation state since we're switching to autonomous
            if (isFlywheelPrepared) {
                robotOperations.stopDynamicFlywheelControl();
                isFlywheelPrepared = false;
                lastOperation = "Manual prep cancelled - Switching to autonomous far";
            }
            lastOperation = robotOperations.executeSmartShooting("shooting_far", true, 2000);
            buttonTimer.reset();
        }

        // ========== MANUAL SHOOTING ==========

        // Right Bumper: Manual shoot (always available)
        if (gamepad2.right_bumper && !lastGamepad2RightBumper && buttonTimer.seconds() > BUTTON_DEBOUNCE_TIME) {
            lastOperation = "Manual shooting";
            try {
                robotOperations.shoot(true);  // true: use cameraservo for alignment and distance if apriltag detected
                lastOperation = "Manual shooting completed";
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                lastOperation = "Manual shooting interrupted";
            } catch (Exception e) {
                lastOperation = "Manual shooting error: " + e.getMessage();
            }
            buttonTimer.reset();
        }

        // ========== INTAKE CONTROL ==========

        // X Button: Toggle intake
        if (gamepad2.x && !lastGamepad2X && buttonTimer.seconds() > BUTTON_DEBOUNCE_TIME) {
            intakeOn = !intakeOn;
            if (intakeOn) {
                intake.startIntake();
                lastOperation = "Intake ON";
            } else {
                intake.stopIntake();
                lastOperation = "Intake OFF";
            }
            buttonTimer.reset();
        }
    
    }



    /**
     * Update telemetry display
     */
    private void updateTelemetry() {
        // Current robot position
        FieldPose currentPose = baseMotion.getCurrentPose();
        telemetry.addData("🤖 Robot Position", "X: %.1f, Y: %.1f, H: %.1f°",
                currentPose.x, currentPose.y, currentPose.heading);

        // Last operation status
        telemetry.addData("🔧 Last Operation", lastOperation);

        // Movement result status
        String moveStatus = robotOperations.getLastMoveResultStatus();
        telemetry.addData("📊 Move Status", moveStatus);

        // Shooting zone status
        boolean inShootingZone = robotOperations.isInShootingZone();
        telemetry.addData("🎯 In Shooting Zone", inShootingZone ? "YES" : "NO");

        // Intake status
        telemetry.addData("🔄 Intake", intakeOn ? "ON" : "OFF");

        // Manual shooting state
        if (isFlywheelPrepared) {
            telemetry.addData("🎯 Manual Shooting", "✅ READY TO SHOOT (GP1 RB to shoot)");
            telemetry.addLine("⚠️  GP2 A/B will cancel manual prep and start autonomous");
        } else {
            telemetry.addData("🎯 Manual Shooting", "❌ NOT PREPARED (GP1 RB to prepare)");
        }

        // Dynamic flywheel status
        telemetry.addData("🌪️ Flywheel", robotOperations.isDynamicFlywheelActive() ?
                String.format("Dynamic (%.0f RPM)", robotOperations.getCurrentTargetVelocity()) : "Idle");

        // Control hints
        telemetry.addLine("");
        if (isFlywheelPrepared) {
            telemetry.addLine("🎮 Manual Mode: GP1 RB=Shoot | GP2 A/B=Cancel&Auto X=Intake");
        } else {
            telemetry.addLine("🎮 GP1: RB=Prep/Shoot A=Gate B=Load Y=Park | GP2: A=Near B=Far RB=Shoot X=Intake");
        }

        telemetry.update();
    }

    /**
     * Update button states for debouncing
     */
    private void updateButtonStates() {
        lastGamepad1A = gamepad1.a;
        lastGamepad1B = gamepad1.b;
        lastGamepad1Y = gamepad1.y;
        lastGamepad1RightBumper = gamepad1.right_bumper;
        
        lastGamepad2A = gamepad2.a;
        lastGamepad2B = gamepad2.b;
        lastGamepad2X = gamepad2.x;
        lastGamepad2RightBumper = gamepad2.right_bumper;
    }

    /**
     * Clean up all subsystems
     */
    private void cleanupSubsystems() {

        RobotOperations.saveOdometryAtAutoEnd(hardwareMap, baseMotion, true);
        telemetry.addLine("🧹 Save Odometry...");
        telemetry.update();

        telemetry.addLine("🧹 Cleaning up subsystems...");
        telemetry.update();

        if (robotOperations != null) {
            robotOperations.emergencyStop();
        }

        if (cameraServo != null) {
            cameraServo.stopThread();
            cameraServo.cleanup();
        }

        telemetry.addLine("✅ Cleanup complete");
        telemetry.update();
    }
}
