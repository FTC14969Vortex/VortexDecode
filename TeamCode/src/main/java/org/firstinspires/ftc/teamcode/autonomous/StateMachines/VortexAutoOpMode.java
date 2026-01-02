package org.firstinspires.ftc.teamcode.autonomous.StateMachines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Helper.DecodeAprilTag;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Helper.Flipper;
import org.firstinspires.ftc.teamcode.Helper.FlyWheel;
import org.firstinspires.ftc.teamcode.Helper.Intake;
import org.firstinspires.ftc.teamcode.Helper.Kicker;

@Autonomous(name = " State Machines Auto 0.01" +
        "", group = "Autonomous")
public class VortexAutoOpMode extends LinearOpMode {

    // Pose constants - using Pose2D directly (inches, degrees)
    private final Pose2D startPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    private final Pose2D shootPose = new Pose2D(DistanceUnit.INCH, 24, 24, AngleUnit.DEGREES, 0); // Single shoot position
    private final Pose2D pickup1Pose = new Pose2D(DistanceUnit.INCH, 37, 121, AngleUnit.DEGREES, 0);
    private final Pose2D pickup2Pose = new Pose2D(DistanceUnit.INCH, 43, 130, AngleUnit.DEGREES, 0);
    private final Pose2D pickup3Pose = new Pose2D(DistanceUnit.INCH, 49, 135, AngleUnit.DEGREES, 0);

    // Shooting distance (in inches) - single shoot position
    double SHOOT_DISTANCE = 45.0;  // Distance for shooting
        
    // Intake forward distance (in inches) - fixed distance to drive forward while intaking at each ball spot
    // This represents the distance needed to collect 3 balls in a row before hitting the field wall
    // TODO: Measure actual field distance and update this value
    double INTAKE_FORWARD_DISTANCE = 36.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- create hardware-level objects ---
        Intake intake = new Intake();
        FlyWheel flyWheel = new FlyWheel();
        Flipper flipper = new Flipper();
        Kicker kicker = new Kicker();
        DecodeAprilTag aprilTag = new DecodeAprilTag(this);

        // --- build high-level managers ---
        // Use dryRun=true to skip hardware calls when hardware is not initialized
        IntakeManager intakeManager = new IntakeManager(intake, telemetry, true);
        ShootManager shootManager = new ShootManager(flyWheel, kicker, flipper, intake, telemetry, true);
        
        // DriverManager uses MotionExecutor (initialized internally)
        DriverManager driveManager = new DriverManager(hardwareMap, telemetry);

        // Reset odometry position and IMU before setting starting pose
        // This ensures accurate localization at the start of autonomous
        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.resetPosAndIMU();

        // Set starting pose BEFORE waitForStart()
        driveManager.setStartingPose(startPose);

        // Setup poses for GameManager (single shoot position)
        Pose2D[] BALL_POS = {pickup1Pose, pickup2Pose, pickup3Pose};  
        Pose2D SHOOT_POS = shootPose;  // Single shoot position
        Pose2D PARK_POS = shootPose;  // Park at shoot pose

        // Initialize GameManager
        GameManager gameManager = new GameManager(
            driveManager,
            intakeManager,
            shootManager,
            telemetry,
            BALL_POS,
            SHOOT_POS,
            PARK_POS,
            SHOOT_POS,  // Initial shoot position (same as shoot position)
            30,  // autoTotalTimeSec
            5,   // parkReserveSec
            3,   // ballsPerSpot
            SHOOT_DISTANCE,
            new double[]{SHOOT_DISTANCE},  // Single shoot distance
            SHOOT_DISTANCE,
            INTAKE_FORWARD_DISTANCE,  // intakeForwardDistanceInch
            4,   // driveTimeoutSec
            3,   // intakeTimeoutSec
            3    // shootTimeoutSec
        );

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

            telemetry.update();
        }
    }
}
