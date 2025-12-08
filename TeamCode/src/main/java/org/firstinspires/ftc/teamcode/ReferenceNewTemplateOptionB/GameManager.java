package org.firstinspires.ftc.teamcode.newStructureOptionB;

package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.subsystems.DriveController.Pose2d;
import org.firstinspires.ftc.teamcode.subsystems.DriveController.DriveState;
import org.firstinspires.ftc.teamcode.subsystems.DriveController.DriveResult;
import org.firstinspires.ftc.teamcode.subsystems.DriveController.DriveGoalKind;
import org.firstinspires.ftc.teamcode.subsystems.IntakeController.IntakeState;
import org.firstinspires.ftc.teamcode.subsystems.IntakeController.IntakeResult;
import org.firstinspires.ftc.teamcode.subsystems.ShooterController.ShooterState;
import org.firstinspires.ftc.teamcode.subsystems.ShooterController.ShooterResult;

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

        AUTO_DRIVE_TO_SHOOT,  // drive to shoot spot[shootIndex]
        AUTO_SHOOT,           // shoot balls

        PARK_DRIVE,           // drive to park pose

        DONE
    }

    // ------------------------------------------------------------
    // FIELDS
    // ------------------------------------------------------------

    private final DriveController   drive;
    private final IntakeController  intake;
    private final ShooterController shooter;
    private final Telemetry telemetry;

    private GameState state = GameState.INIT;

    // Field layout (global static config)
    private final Pose2d[] ballSpots;
    private final Pose2d[] shootSpots;
    private final Pose2d   parkPose;

    private int ballIndex  = 0;  // which BALL_SPOT we are at
    private int shootIndex = 0;  // which SHOOT_SPOT we are at (can be = ballIndex or separate)

    // Auto timing
    private final double autoTotalTimeSec;   // e.g. 30.0
    private final double parkReserveSec;     // time reserved for parking at the end
    private double autoStartTimeSec = 0.0;

    // Subsystem timeouts / params
    private final int    ballsPerSpot;
    private final double driveTimeoutSec;
    private final double intakeTimeoutSec;
    private final double shootTimeoutSec;

    private final ElapsedTime stateTimer = new ElapsedTime();

    // ------------------------------------------------------------
    // CONSTRUCTOR
    // ------------------------------------------------------------

    public GameManager(
            DriveController drive,
            IntakeController intake,
            ShooterController shooter,
            Telemetry telemetry,
            Pose2d[] ballSpots,
            Pose2d[] shootSpots,
            Pose2d   parkPose,
            double autoTotalTimeSec,
            double parkReserveSec,
            int    ballsPerSpot,
            double driveTimeoutSec,
            double intakeTimeoutSec,
            double shootTimeoutSec
    ) {
        this.drive   = drive;
        this.intake  = intake;
        this.shooter = shooter;
        this.telemetry = telemetry;

        this.ballSpots  = ballSpots;
        this.shootSpots = shootSpots;
        this.parkPose   = parkPose;

        this.autoTotalTimeSec = autoTotalTimeSec;
        this.parkReserveSec   = parkReserveSec;
        this.ballsPerSpot     = ballsPerSpot;
        this.driveTimeoutSec  = driveTimeoutSec;
        this.intakeTimeoutSec = intakeTimeoutSec;
        this.shootTimeoutSec  = shootTimeoutSec;
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
        ballIndex  = 0;
        shootIndex = 0;

        drive.resetCycle();
        intake.resetCycle();
        shooter.resetCycle();

        state = GameState.INIT;
        stateTimer.reset();

        telemetry.addData("GM", "Auto start: state=INIT");
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
        telemetry.addData("GM/shootIndex", shootIndex);
        telemetry.addData("GM/timeRemain", "%.1f", timeRemaining(nowSec));
    }

    // ------------------------------------------------------------
    // STATE HANDLERS
    // ------------------------------------------------------------

    /** INIT → go to first ball spot or directly park if no spots. */
    private void handleInit() {
        if (ballSpots == null || ballSpots.length == 0) {
            // No ball spots defined → go directly to park
            goToParkState();
            return;
        }

        startDriveToBallSpot();
    }

    /** Drive to BALL_SPOT[ballIndex]. */
    private void handleAutoDriveToBall() {
        // If drive is idle (not started, or reset), start it.
        if (drive.isIdle()) {
            startDriveToBallSpot();
            return;
        }

        if (drive.getState() == DriveState.DONE) {
            DriveResult res = drive.getResult();

            if (res == DriveResult.ARRIVED_OK) {
                // Start intake at this ball spot
                intake.resetCycle();
                intake.startCycle(ballsPerSpot, intakeTimeoutSec);

                state = GameState.AUTO_INTAKE;
                stateTimer.reset();
                telemetry.addData("GM", "Drive to ball OK → AUTO_INTAKE");
            } else {
                // drive failed / align failed / aborted → go next spot or park
                advanceBallIndexOrPark();
            }
        }
    }

    /** Run intake at current ball spot. */
    private void handleAutoIntake() {
        if (intake.getState() == IntakeState.IDLE) {
            // If somehow idle here, (re)start cycle
            intake.startCycle(ballsPerSpot, intakeTimeoutSec);
            return;
        }

        if (intake.getState() == IntakeState.DONE) {
            IntakeResult res = intake.getResult();

            switch (res) {
                case GOT_BALLS:
                    // Have ≥1 ball → drive to shoot spot
                    startDriveToShootSpot();
                    break;

                case NO_BALL:
                case ABORTED:
                default:
                    // No balls or aborted → next ball spot or park
                    advanceBallIndexOrPark();
                    break;
            }
        }
    }

    /** Drive to SHOOT_SPOT[shootIndex]. */
    private void handleAutoDriveToShoot() {
        if (drive.isIdle()) {
            startDriveToShootSpot();
            return;
        }

        if (drive.getState() == DriveState.DONE) {
            DriveResult res = drive.getResult();

            if (res == DriveResult.ARRIVED_OK) {
                // start shooting
                shooter.resetCycle();
                shooter.startCycle(ballsPerSpot, shootTimeoutSec);

                state = GameState.AUTO_SHOOT;
                stateTimer.reset();
                telemetry.addData("GM", "Drive to shoot OK → AUTO_SHOOT");
            } else {
                // cannot reach good shoot spot → next ball or park
                advanceBallIndexOrPark();
            }
        }
    }

    /** Run shooter at current shoot spot. */
    private void handleAutoShoot() {
        if (shooter.getState() == ShooterState.IDLE) {
            // If idle unexpectedly, start
            shooter.startCycle(ballsPerSpot, shootTimeoutSec);
            return;
        }

        if (shooter.getState() == ShooterState.DONE) {
            ShooterResult res = shooter.getResult();

            // For now, treat all results the same: move on
            // (you can branch differently if you want)
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
        if (drive.isIdle()) {
            drive.resetCycle();
            drive.startCycle(
                    DriveGoalKind.GOTO_PARK,
                    parkPose,
                    driveTimeoutSec
            );
            telemetry.addData("GM", "Start PARK_DRIVE");
            return;
        }

        if (drive.getState() == DriveState.DONE) {
            // whatever result, we end auto
            forceDone("Park move finished");
        }
    }

    // ------------------------------------------------------------
    // TRANSITION HELPERS
    // ------------------------------------------------------------

    /** Start drive cycle toward BALL_SPOT[ballIndex]. */
    private void startDriveToBallSpot() {
        if (ballIndex >= ballSpots.length) {
            goToParkState();
            return;
        }

        Pose2d target = ballSpots[ballIndex];

        drive.resetCycle();
        drive.startCycle(
                DriveGoalKind.GOTO_BALL_SPOT,
                target,
                driveTimeoutSec
        );

        state = GameState.AUTO_DRIVE_TO_BALL;
        stateTimer.reset();
        telemetry.addData("GM", "Goto BALL_SPOT[%d]", ballIndex);
    }

    /** Start drive cycle toward SHOOT_SPOT[shootIndex]. */
    private void startDriveToShootSpot() {
        if (shootSpots == null || shootSpots.length == 0) {
            // no shoot spots → go park instead
            goToParkState();
            return;
        }

        // simple mapping, you can make smarter later
        shootIndex = Math.min(ballIndex, shootSpots.length - 1);
        Pose2d target = shootSpots[shootIndex];

        drive.resetCycle();
        drive.startCycle(
                DriveGoalKind.GOTO_SHOOT_SPOT,
                target,
                driveTimeoutSec
        );

        state = GameState.AUTO_DRIVE_TO_SHOOT;
        stateTimer.reset();
        telemetry.addData("GM", "Goto SHOOT_SPOT[%d]", shootIndex);
    }

    /**
     * After finishing intake/shoot, decide to:
     * - go next ball spot
     * - or go park if no more spots (or you later add time-based logic).
     */
    private void advanceBallIndexOrPark() {
        ballIndex++;
        if (ballIndex >= ballSpots.length) {
            goToParkState();
        } else {
            startDriveToBallSpot();
        }
    }

    private void goToParkState() {
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
}
