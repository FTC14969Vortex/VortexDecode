package org.firstinspires.ftc.teamcode.autonomous.StateMachines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
// IMPORTANT: Use fully qualified name to avoid conflict with TeamCode AprilTagProcessor
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.external.gobilida.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.motion.FieldPose;
import org.firstinspires.ftc.teamcode.motion.FieldPositions;
import org.firstinspires.ftc.teamcode.subsystems.Flipper;
import org.firstinspires.ftc.teamcode.subsystems.FlyWheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.utils.StateMachineLogger;
import org.firstinspires.ftc.teamcode.vision.CameraServo;
@Disabled
@Autonomous(name = " State Machines Auto 0.01" +
        "", group = "Autonomous")
public class VortexAutoOpMode extends LinearOpMode {

    // Pose constants - will be set based on alliance selection (BLUE or RED)
    private FieldPose startPose;
    private FieldPose shootPose; // Single shoot position
    private FieldPose pickup1Pose;
    private FieldPose pickup2Pose;
    private FieldPose pickup3Pose;
    
    // Vision system
    private VisionPortal visionPortal;
    private org.firstinspires.ftc.vision.apriltag.AprilTagProcessor aprilTagProcessor;
    private CameraServo cameraServo;

    // Detailed logging flag (default true for debugging)
    // Set to false to disable detailed file logging
    private static final boolean ENABLE_DETAILED_LOGGING = true;

    // Shooting distance (in inches) - single shoot position
    // Note: This is a fallback distance if CameraServo is not available
    double SHOOT_DISTANCE = 45.0;  // Distance for shooting

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Initialize Vision System (before waitForStart) ---
        initVisionSystem();
        
        // --- create hardware-level objects ---
        Intake intake = new Intake();
        FlyWheel flyWheel = new FlyWheel();
        Flipper flipper = new Flipper();
        Kicker kicker = new Kicker();

        // --- Initialize subsystems (required even if dryRun=true, for consistency) ---
        intake.init(this);
        intake.stopIntake();  // Start with intake stopped
        
        flyWheel.init(this);
        
        kicker.init(hardwareMap);
        kicker.setGatePosition(Kicker.GATE_INTAKE);  // Start in intake position
        
        flipper.init(hardwareMap);
        flipper.resetFlipper();

        // --- build high-level managers ---
        // DriverManager must be created BEFORE CameraServo (needs MotionExecutor)
        DriverManager driveManager = new DriverManager(this, telemetry);
        
        // Initialize CameraServo with CoordinateTransformer for odometry fallback
        cameraServo = new CameraServo();
        cameraServo.init(hardwareMap, 
                         null, // odometryManager (optional - can be null)
                         driveManager.getCoordinateTransformer(),
                         driveManager.getMotionExecutor());
        
        // Alliance selection via gamepad during init
        // Press X for BLUE (Tag 20) or B for RED (Tag 24)
        int allianceTagId = 20; // Default to blue
        String allianceName = "BLUE";
        telemetry.addData("Alliance Selection", "Press X for BLUE (Tag 20) or B for RED (Tag 24)");
        telemetry.update();
        
        // Wait for gamepad input to select alliance (before waitForStart)
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) {
                allianceTagId = 20;
                allianceName = "BLUE";
                telemetry.addData("Alliance", "Selected: %s (Tag %d)", allianceName, allianceTagId);
                telemetry.update();
                // Wait for button release
                while (gamepad1.x && !isStopRequested()) {
                    idle();
                }
            } else if (gamepad1.b) {
                allianceTagId = 24;
                allianceName = "RED";
                telemetry.addData("Alliance", "Selected: %s (Tag %d)", allianceName, allianceTagId);
                telemetry.update();
                // Wait for button release
                while (gamepad1.b && !isStopRequested()) {
                    idle();
                }
            }
            idle();
        }
        
        cameraServo.setTargetTag(allianceTagId);
        telemetry.addData("Alliance", "Final: %s (Tag ID: %d)", allianceName, allianceTagId);
        cameraServo.moveToCenter();
        cameraServo.setAutoOdometryCorrection(false);
        
        // Mirror field positions for RED alliance
        if (allianceTagId == 24) { // RED alliance
            startPose = FieldPositions.getRedPosition(FieldPositions.START_NEAR);
            shootPose = FieldPositions.getRedPosition(FieldPositions.SHOOTING_NEAR);
            pickup1Pose = FieldPositions.getRedPosition(FieldPositions.INTAKE_1_START);
            pickup2Pose = FieldPositions.getRedPosition(FieldPositions.INTAKE_2_START);
            pickup3Pose = FieldPositions.getRedPosition(FieldPositions.INTAKE_3_START);
            telemetry.addData("Alliance", "Using RED (mirrored) field positions");
        } else { // BLUE alliance (default)
            startPose = FieldPositions.START_NEAR;
            shootPose = FieldPositions.SHOOTING_NEAR;
            pickup1Pose = FieldPositions.INTAKE_1_START;
            pickup2Pose = FieldPositions.INTAKE_2_START;
            pickup3Pose = FieldPositions.INTAKE_3_START;
            telemetry.addData("Alliance", "Using BLUE field positions");
        }
        
        // Create detailed loggers for each manager (default enabled for debugging)
        // Use app-scoped external files directory (recommended for OpModes - avoids scoped storage issues)
        android.content.Context appContext = hardwareMap.appContext;
        StateMachineLogger gameLogger = new StateMachineLogger("GameManager", ENABLE_DETAILED_LOGGING, telemetry, appContext);
        StateMachineLogger driveLogger = new StateMachineLogger("DriverManager", ENABLE_DETAILED_LOGGING, telemetry, appContext);
        StateMachineLogger intakeLogger = new StateMachineLogger("IntakeManager", ENABLE_DETAILED_LOGGING, telemetry, appContext);
        StateMachineLogger shootLogger = new StateMachineLogger("ShootManager", ENABLE_DETAILED_LOGGING, telemetry, appContext);
        
        // Enable telemetry output for on-screen log summaries (optional - can be disabled to reduce telemetry spam)
        if (ENABLE_DETAILED_LOGGING) {
            gameLogger.setTelemetryEnabled(true);
            driveLogger.setTelemetryEnabled(true);
            intakeLogger.setTelemetryEnabled(true);
            shootLogger.setTelemetryEnabled(true);
        }
        
        // Create managers (dryRun=false to match FullAutoOperateTest behavior)
        IntakeManager intakeManager = new IntakeManager(intake, telemetry, false, intakeLogger);
        ShootManager shootManager = new ShootManager(flyWheel, kicker, flipper, intake, telemetry, cameraServo, false, shootLogger);

        // Reset odometry position and IMU before setting starting pose
        // Use the same GoBildaPinpointDriver class that BaseMotion uses to ensure we reset the same instance
        // This ensures accurate localization at the start of autonomous
        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.resetPosAndIMU();

        // Initialize reference point and set starting position (using FieldPose)
        driveManager.initReferencePoint(startPose);

        // Setup poses for GameManager (single shoot position) - using FieldPose
        // Mirror finish positions and parking position for RED alliance
        FieldPose[] BALL_POS = {pickup1Pose, pickup2Pose, pickup3Pose};
        FieldPose[] BALL_FINISH_POS;
        FieldPose PARK_POS;
        if (allianceTagId == 24) { // RED alliance
            BALL_FINISH_POS = new FieldPose[]{
                FieldPositions.getRedPosition(FieldPositions.INTAKE_1_FINISH),
                FieldPositions.getRedPosition(FieldPositions.INTAKE_2_FINISH),
                FieldPositions.getRedPosition(FieldPositions.INTAKE_3_FINISH)
            };
            PARK_POS = FieldPositions.getRedPosition(FieldPositions.PARKING_NEAR);
        } else { // BLUE alliance
            BALL_FINISH_POS = new FieldPose[]{
                FieldPositions.INTAKE_1_FINISH,
                FieldPositions.INTAKE_2_FINISH,
                FieldPositions.INTAKE_3_FINISH
            };
            PARK_POS = FieldPositions.PARKING_NEAR;
        }

        // Initialize GameManager with CameraServo and FlyWheel for parallel preparation
        GameManager gameManager = new GameManager(
            driveManager,
            intakeManager,
            shootManager,
            telemetry,
            BALL_POS,
            BALL_FINISH_POS,  // Finish positions for intake (matches FullAutoOperateTest)
            PARK_POS,   // parkPose
            shootPose,  // defaultShootSpot (single shoot position)
            30,  // autoTotalTimeSec
            5,   // parkReserveSec
            3,   // ballsPerSpot
            SHOOT_DISTANCE,  // defaultShootDistanceInch (single shoot distance)
            4,   // driveTimeoutSec
            3,   // intakeTimeoutSec
            3,   // shootTimeoutSec
            cameraServo,  // CameraServo for vision-based alignment
            flyWheel,  // FlyWheel for parallel spin-up during drive to shoot
            gameLogger  // Logger for detailed debugging
        );
        
        // Pass logger to DriverManager
        driveManager.setLogger(driveLogger);

        // Wait for start (LinearOpMode requirement)
        waitForStart();

        // Begin autonomous sequence after start (CRITICAL: Must be called for time-based logic)
        gameManager.startAuto(getRuntime());

        // Main update loop
        // NOTE: Update driveManager first so gameManager can react immediately to drive completion
        while (opModeIsActive() && !gameManager.isDone()) {
            double nowSec = getRuntime();

            driveManager.update(nowSec);     // local FSM: drive (non-blocking) - UPDATE FIRST
            intakeManager.update(nowSec);    // local FSM: intake
            shootManager.update(nowSec);     // local FSM: shooter
            gameManager.update(nowSec);      // global phase FSM - reacts to drive completion

            // Periodic flush of log buffers (non-blocking, every loop iteration)
            if (ENABLE_DETAILED_LOGGING) {
                gameLogger.periodicFlush();
                driveLogger.periodicFlush();
                intakeLogger.periodicFlush();
                shootLogger.periodicFlush();
            }

            telemetry.update();
        }
        
        // Close loggers when done
        if (ENABLE_DETAILED_LOGGING) {
            gameLogger.close();
            driveLogger.close();
            intakeLogger.close();
            shootLogger.close();
            
            telemetry.addData("Logging", "Logs saved to:");
            String gameLogPath = gameLogger.getLogFilePath();
            String driveLogPath = driveLogger.getLogFilePath();
            String intakeLogPath = intakeLogger.getLogFilePath();
            String shootLogPath = shootLogger.getLogFilePath();
            telemetry.addData("  GameManager", gameLogPath != null ? gameLogPath : "N/A");
            telemetry.addData("  DriverManager", driveLogPath != null ? driveLogPath : "N/A");
            telemetry.addData("  IntakeManager", intakeLogPath != null ? intakeLogPath : "N/A");
            telemetry.addData("  ShootManager", shootLogPath != null ? shootLogPath : "N/A");
            telemetry.update();
            sleep(2000);  // Give time to read log paths
        }
        
        // Close VisionPortal to prevent resource leak
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
    
    /**
     * Initialize vision system (VisionPortal and AprilTagProcessor).
     * Must be called before waitForStart().
     */
    private void initVisionSystem() {
        // Use fully qualified SDK class name to avoid name collision
        aprilTagProcessor = new org.firstinspires.ftc.vision.apriltag.AprilTagProcessor.Builder()
            .setTagFamily(org.firstinspires.ftc.vision.apriltag.AprilTagProcessor.TagFamily.TAG_36h11)
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
            .build();

        visionPortal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .addProcessor(aprilTagProcessor)
            .build();
            
        telemetry.addData("Vision", "VisionPortal initialized");
    }
}
