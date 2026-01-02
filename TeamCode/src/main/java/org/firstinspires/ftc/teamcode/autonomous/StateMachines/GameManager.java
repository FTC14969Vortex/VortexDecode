package org.firstinspires.ftc.teamcode.autonomous.StateMachines;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * GameManager (Option B)
 *
 * - Top-level AUTO state machine (GameState)
 * - Orchestrates Drive / Intake / Shooter via their cycle-style APIs
 * - Non-blocking: call update(nowSec) once per loop
 */
public class GameManager {

    // ------------------------------------------------------------
    // GAME STATE
    // ------------------------------------------------------------

    public enum GameState {
        INIT,

        AUTO_DRIVE_TO_BALL,   // drive to ball spot[ballIndex]
        AUTO_INTAKE,          // run intake at that spot

        AUTO_DRIVE_TO_SHOOT,  // drive to shoot spot[ballIndex]
        AUTO_SHOOT,           // shoot balls

        PARK_DRIVE,           // drive to park pose

        DONE
    }

    // ------------------------------------------------------------
    // FIELDS
    // ------------------------------------------------------------

    private final DriverManager drive;
    private final IntakeManager intake;
    private final ShootManager shooter;
    private final Telemetry telemetry;

    private GameState state = GameState.INIT;

    // Field layout (global static config)
    private final Pose2D[] ballSpots;

    private final Pose2D defaultShootSpot;
    private final Pose2D parkPose;

    private int ballIndex = -1;  // which BALL_SPOT we are at (-1 = initial shoot, then 0, 1, 2...)

    // Auto timing
    private final double autoTotalTimeSec;   // e.g. 30.0
    private final double parkReserveSec;     // time reserved for parking at the end
    private double autoStartTimeSec = 0.0;

    // Subsystem timeouts / params
    private final int    ballsPerSpot;
    private final double defaultShootDistanceInch; // Ultimate fallback distance if array access fails
    private final double intakeForwardDistanceInch; // Fixed distance to drive forward while intaking at each ball spot

    //TODO: need to review
    private double driveTimeoutSec = 4;
    private double intakeTimeoutSec = 3;
    private double shootTimeoutSec = 3;

    private final ElapsedTime stateTimer = new ElapsedTime();

    // ------------------------------------------------------------
    // CONSTRUCTOR
    // ------------------------------------------------------------

    public GameManager(
            DriverManager drive,
            IntakeManager intake,
            ShootManager shooter,
            Telemetry telemetry,
            Pose2D[] ballSpots,
            Pose2D   parkPose,
            Pose2D   defaultShootSpot,
            double autoTotalTimeSec,
            double parkReserveSec,
            int    ballsPerSpot,
            double defaultShootDistanceInch,
            double intakeForwardDistanceInch,
            double driveTimeoutSec,
            double intakeTimeoutSec,
            double shootTimeoutSec
    ) {
        this.drive   = drive;
        this.intake  = intake;
        this.shooter = shooter;
        this.telemetry = telemetry;

        this.ballSpots  = ballSpots;
        this.defaultShootSpot = defaultShootSpot;
        this.parkPose   = parkPose;

        this.autoTotalTimeSec = autoTotalTimeSec;
        this.parkReserveSec   = parkReserveSec;
        this.ballsPerSpot     = ballsPerSpot;
        this.defaultShootDistanceInch = defaultShootDistanceInch;
        this.intakeForwardDistanceInch = intakeForwardDistanceInch;
        this.driveTimeoutSec  = driveTimeoutSec;
        this.intakeTimeoutSec = intakeTimeoutSec;
        this.shootTimeoutSec  = shootTimeoutSec;

        // Validation checks
        if (ballSpots == null || ballSpots.length == 0) {
            telemetry.addData("GM/WARNING", "ballSpots is null or empty");
        }
        if (defaultShootSpot == null) {
            telemetry.addData("GM/WARNING", "defaultShootSpot is null");
        }
        if (parkPose == null) {
            telemetry.addData("GM/WARNING", "parkPose is null");
        }
        if (defaultShootDistanceInch <= 0) {
            telemetry.addData("GM/WARNING", "defaultShootDistanceInch is invalid: %.1f", defaultShootDistanceInch);
        }
    }

    // ------------------------------------------------------------
    // PUBLIC API
    // ------------------------------------------------------------

    public GameState getState() {
        return state;
    }

    public boolean isDone() {
        return state == GameState.DONE;
    }

    /** Call once when AUTO actually starts (after waitForStart). */
    public void startAuto(double nowSec) {
        autoStartTimeSec = nowSec;
        ballIndex = -1;  // Start with initial shoot (-1), then go to 0, 1, 2...

        drive.resetCycle();
        intake.resetCycle();
        shooter.resetCycle();

        state = GameState.INIT;
        stateTimer.reset();

        telemetry.addData("GM", "Auto start: state=INIT (will do initial shoot first)");
    }

    /**
     * Main non-blocking update.
     * In Option B, OpMode should call:
     *
     *   drive.update(nowSec);
     *   intake.update(nowSec);
     *   shooter.update(nowSec);
     *   gameManager.update(nowSec);
     */
    public void update(double nowSec) {
        if (state == GameState.DONE) return;

        // hard stop when auto time is over
        if (timeElapsed(nowSec) >= autoTotalTimeSec) {
            forceDone("Auto time expired");
            return;
        }

        // if close to the end, force transition to parking, unless already parking/done
        if (timeRemaining(nowSec) <= parkReserveSec &&
                state != GameState.PARK_DRIVE &&
                state != GameState.DONE) {
            goToParkState();
        }

        switch (state) {
            case INIT:
                handleInit();
                break;

            case AUTO_DRIVE_TO_BALL:
                handleAutoDriveToBall();
                break;

            case AUTO_INTAKE:
                handleAutoIntake();
                break;

            case AUTO_DRIVE_TO_SHOOT:
                handleAutoDriveToShoot();
                break;

            case AUTO_SHOOT:
                handleAutoShoot();
                break;

            case PARK_DRIVE:
                handleParkDrive();
                break;

            case DONE:
            default:
                break;
        }

        telemetry.addData("GM/State", state);
        telemetry.addData("GM/ballIndex", ballIndex);
        telemetry.addData("GM/timeRemain", "%.1f", timeRemaining(nowSec));
    }

    // ------------------------------------------------------------
    // STATE HANDLERS
    // ------------------------------------------------------------

    /** INIT → go to initial shoot position first, then proceed to ball pickup cycles. */
    private void handleInit() {
        // Validate defaultShootSpot is not null
        if (defaultShootSpot == null) {
            telemetry.addData("GM/ERROR", "defaultShootSpot is null - going to park");
            goToParkState();
            return;
        }
        
        // Drive to initial shoot position for preload
        if (drive.isIdle()) {
            drive.resetCycle();
            drive.startCycle(
                    DriverManager.DriveGoalKind.GOTO_SHOOT_SPOT,
                    defaultShootSpot,
                    driveTimeoutSec
            );
            state = GameState.AUTO_DRIVE_TO_SHOOT;
            stateTimer.reset();
            telemetry.addData("GM", "Goto INITIAL_SHOOT (defaultShootSpot)");
            return;
        }

        if (drive.getState() == DriverManager.DriveState.DONE) {
            DriverManager.DriveResult res = drive.getResult();

            if (res == DriverManager.DriveResult.ARRIVED_OK) {
                // Start initial shoot (use default shoot distance)
                shooter.resetCycle();
                shooter.startCycle(defaultShootDistanceInch, shootTimeoutSec);
                state = GameState.AUTO_SHOOT;
                stateTimer.reset();
                telemetry.addData("GM", "Drive to initial shoot OK → AUTO_SHOOT (dist=%.1f)", defaultShootDistanceInch);
            } else {
                // Cannot reach initial shoot position → go to park
                telemetry.addData("GM/ERROR", "Cannot reach initial shoot position (%s) - going to park", res);
                goToParkState();
            }
        }
    }   

    /** Drive to BALL_SPOT[ballIndex]. */
    private void handleAutoDriveToBall() {
        // If drive is idle (not started, or reset), start it.
        if (drive.isIdle()) {
            startDriveToBallSpot();
            return;
        }

        if (drive.getState() == DriverManager.DriveState.DONE) {
            DriverManager.DriveResult res = drive.getResult();

            if (res == DriverManager.DriveResult.ARRIVED_OK) {
                // Transition to AUTO_INTAKE state (will start forward drive in handleAutoIntake)
                state = GameState.AUTO_INTAKE;
                stateTimer.reset();
                telemetry.addData("GM", "Drive to ball OK → AUTO_INTAKE (will drive forward)");
            } else {
                // drive failed / align failed / aborted → go next spot or park
                advanceBallIndexOrPark();
            }
        }
    }

    /** Drive forward while intaking at current ball spot. */
    private void handleAutoIntake() {
        // Critical: Ensure previous ball-spot drive is fully complete before starting forward drive
        // Check if drive state is DONE (previous drive finished)
        if (drive.getState() == DriverManager.DriveState.DONE && 
            drive.getGoalKind() != DriverManager.DriveGoalKind.DRIVE_FORWARD_FOR_INTAKE) {
            // Previous drive to ball spot is complete, start intake cycle and forward drive in parallel
            // Note: In testing mode, intake immediately completes, but we tie intake completion to drive completion
            intake.resetCycle();
            intake.startCycle(ballsPerSpot, (long) intakeTimeoutSec);
            
            drive.resetCycle(); // Reset to IDLE state to prepare for new drive
            drive.startForwardDrive(intakeForwardDistanceInch, driveTimeoutSec);
            stateTimer.reset();
            telemetry.addData("GM", "Starting forward drive + intake: %.1f inches", intakeForwardDistanceInch);
            return;
        }

        // Monitor forward drive progress (intake completion is tied to drive completion)
        if (drive.getGoalKind() == DriverManager.DriveGoalKind.DRIVE_FORWARD_FOR_INTAKE) {
            if (drive.getState() == DriverManager.DriveState.DONE) {
                DriverManager.DriveResult res = drive.getResult();

                if (res == DriverManager.DriveResult.ARRIVED_OK) {
                    // Forward drive completed successfully → intake is complete (GOT_BALLS)
                    // Note: We tie intake completion to drive completion since we're driving forward while intaking
                    telemetry.addData("GM", "Forward drive OK → intake complete (reached distance/wall), drive to shoot");
                    startDriveToShootSpot();
                } else {
                    // Forward drive failed → treat as intake failure → next ball spot or park
                    intake.abortCycle(); // Abort intake if drive failed
                    telemetry.addData("GM", "Forward drive failed (%s) → next spot or park", res);
                    advanceBallIndexOrPark();
                }
            }
        }
    }

    /** Drive to SHOOT_SPOT[ballIndex]. */
    private void handleAutoDriveToShoot() {
        if (drive.isIdle()) {
            startDriveToShootSpot();
            return;
        }

        if (drive.getState() == DriverManager.DriveState.DONE) {
            DriverManager.DriveResult res = drive.getResult();

            if (res == DriverManager.DriveResult.ARRIVED_OK) {
                // start shooting (always use defaultShootDistanceInch - single shoot position)
                shooter.resetCycle();
                shooter.startCycle(defaultShootDistanceInch, shootTimeoutSec);

                state = GameState.AUTO_SHOOT;
                stateTimer.reset();
                telemetry.addData("GM", "Drive to shoot OK → AUTO_SHOOT (ballIndex=%d, dist=%.1f)", ballIndex, defaultShootDistanceInch);
            } else {
                // cannot reach good shoot spot → next ball or park
                advanceBallIndexOrPark();
            }
        }
    }

    /** Run shooter at current shoot spot. */
    private void handleAutoShoot() {
        if (shooter.getState() == ShootManager.ShooterState.IDLE) {
            // If idle unexpectedly, start
            // Always use defaultShootDistanceInch (single shoot position)
            shooter.startCycle(defaultShootDistanceInch, shootTimeoutSec);
            return;
        }

        if (shooter.getState() == ShootManager.ShooterState.DONE) {
            ShootManager.ShooterResult res = shooter.getResult();

            // After initial shoot, transition to first ball pickup
            if (ballIndex == -1) {
                ballIndex = 0;  // Start with first ball spot (was -1 for initial shoot)
                startDriveToBallSpot();
                return;
            }

            // For cycle shoots, treat all results the same: move on
            switch (res) {
                case SUCCESS:
                case JAM_FAILED:
                case ABORTED:
                default:
                    advanceBallIndexOrPark();
                    break;
            }
        }
    }

    /** Drive to PARK position; after that we are DONE. */
    private void handleParkDrive() {
        // If drive was aborted (from goToParkState), reset it first
        if (drive.getState() == DriverManager.DriveState.DONE && 
            drive.getResult() == DriverManager.DriveResult.ABORTED) {
            drive.resetCycle();
        }
        
        if (drive.isIdle()) {
            drive.resetCycle();
            drive.startCycle(
                    DriverManager.DriveGoalKind.GOTO_PARK,
                    parkPose,
                    driveTimeoutSec
            );
            telemetry.addData("GM", "Start PARK_DRIVE");
            return;
        }

        if (drive.getState() == DriverManager.DriveState.DONE) {
            // Park move completed (or failed) → end auto
            forceDone("Park move finished");
        }
    }

    // ------------------------------------------------------------
    // TRANSITION HELPERS
    // ------------------------------------------------------------

    /** Start drive cycle toward BALL_SPOT[ballIndex]. */
    private void startDriveToBallSpot() {
        if (ballSpots == null || ballIndex >= ballSpots.length) {
            goToParkState();
            return;
        }

        Pose2D target = ballSpots[ballIndex];

        drive.resetCycle();
        drive.startCycle(
                DriverManager.DriveGoalKind.GOTO_BALL_SPOT,
                target,
                driveTimeoutSec
        );

        state = GameState.AUTO_DRIVE_TO_BALL;
        stateTimer.reset();
        telemetry.addData("GM", "Goto BALL_SPOT[%d]", ballIndex);
    }

    /** Start drive cycle toward defaultShootSpot. */
    private void startDriveToShootSpot() {
        // Validate defaultShootSpot is not null
        if (defaultShootSpot == null) {
            telemetry.addData("GM/ERROR", "defaultShootSpot is null - going to park");
            goToParkState();
            return;
        }

        // Always use the single defaultShootSpot
        drive.resetCycle();
        drive.startCycle(
                DriverManager.DriveGoalKind.GOTO_SHOOT_SPOT,
                defaultShootSpot,
                driveTimeoutSec
        );

        state = GameState.AUTO_DRIVE_TO_SHOOT;
        stateTimer.reset();
        telemetry.addData("GM", "Goto SHOOT_SPOT (ballIndex=%d)", ballIndex);
    }

    /**
     * After finishing intake/shoot, decide to:
     * - go next ball spot
     * - or go park if no more spots (or you later add time-based logic).
     */
    private void advanceBallIndexOrPark() {
         ballIndex++;
        if (ballSpots == null || ballIndex >= ballSpots.length) {
            goToParkState();
        } else {
            startDriveToBallSpot();
        }
    }

    private void goToParkState() {
        // Abort any running cycles before parking (including drive)
        drive.abortCycle();
        intake.abortCycle();
        shooter.abortCycle();
        
        state = GameState.PARK_DRIVE;
        stateTimer.reset();
        telemetry.addData("GM", "Transition → PARK_DRIVE");
    }

    private void forceDone(String reason) {
        drive.abortCycle();
        intake.abortCycle();
        shooter.abortCycle();

        state = GameState.DONE;
        telemetry.addData("GM", "DONE: %s", reason);
    }

    // ------------------------------------------------------------
    // TIME HELPERS
    // ------------------------------------------------------------

    private double timeElapsed(double nowSec) {
        return nowSec - autoStartTimeSec;
    }

    private double timeRemaining(double nowSec) {
        return autoTotalTimeSec - timeElapsed(nowSec);
    }

    // ------------------------------------------------------------
    // DISTANCE HELPERS
    // ------------------------------------------------------------

    // Note: Removed getShootDistance() - we now always use defaultShootDistanceInch
    // since there's only one shoot position
}