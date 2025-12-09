package org.firstinspires.ftc.teamcode.ReferenceNewTemplateOptionA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class GameManager {

    public enum Mode {
        INIT,
        INTAKE_MODE,
        SHOOT_MODE,
        PARK_MODE,
        DONE
    }

    // ---------- Global state ----------
    private Mode mode = Mode.INIT;

    private final Pose2D[] BALL_POS;
    private final Pose2D[] SHOOT_POS;
    private final Pose2D PARK_POS;

    private int ballIndex  = 0;
    private int shootIndex = 0;

    private boolean readyForIntake = true;
    private boolean readyForShoot  = false;
    private boolean readyForPark   = false;

    private final long matchStartMs;

    // Subsystems
    private final IntakeManager intakeManager;
    private final ShootManager  shootManager;
    private final Telemetry telemetry;

    public GameManager(Pose2D[] ballPos,
                       Pose2D[] shootPos,
                       Pose2D parkPos,
                       IntakeManager intakeManager,
                       ShootManager shootManager,
                       Telemetry telemetry) {

        this.BALL_POS = ballPos;
        this.SHOOT_POS = shootPos;
        this.PARK_POS = parkPos;
        this.intakeManager = intakeManager;
        this.shootManager = shootManager;
        this.telemetry = telemetry;

        this.matchStartMs = System.currentTimeMillis();
    }

    // ---------- Helper: time & availability ----------
    private long timeElapsed() {
        return System.currentTimeMillis() - matchStartMs;
    }

    private long timeRemaining() {
        // 30s auto
        long matchDurationMs;
        matchDurationMs = 30000;
        return matchDurationMs - timeElapsed();
    }

    private boolean timeLow() {
        return timeRemaining() < 5000; // 5s left
    }

    private boolean timeEnoughForIntakeAndShoot() {
        return timeRemaining() > 8000; // tune
    }

    private boolean moreBallSpots() {
        return ballIndex < BALL_POS.length;
    }

    private boolean moreShootSpots() {
        return shootIndex < SHOOT_POS.length;
    }

    // ---------- Main update: called every loop ----------
    public void update() {
        // First, update local subsystems
        intakeManager.update();
        shootManager.update();

        switch (mode) {
            case INIT:
                // INIT can be more elaborate if needed.
                mode = Mode.INTAKE_MODE;
                readyForIntake = true;
                readyForShoot  = false;
                readyForPark   = false;
                break;

            case INTAKE_MODE:
                handleIntakeMode();
                break;

            case SHOOT_MODE:
                handleShootMode();
                break;

            case PARK_MODE:
                handleParkMode();
                break;

            case DONE:
                // nothing more to do
                break;
        }

        telemetry.addData("GM Mode", mode);
        telemetry.addData("GM ballIndex", ballIndex);
        telemetry.addData("GM shootIndex", shootIndex);
        telemetry.addData("GM readyIntake", readyForIntake);
        telemetry.addData("GM readyShoot", readyForShoot);
        telemetry.addData("GM readyPark", readyForPark);
    }

    // ---------- Per-mode handlers ----------

    private void handleIntakeMode() {
        // If we haven't started a cycle yet and we are allowed to intake:
        if (readyForIntake
                && intakeManager.getState() == IntakeManager.IntakeState.IDLE
                && moreBallSpots()
                && !timeLow()) {

            Pose2D ballPose = BALL_POS[ballIndex];
            // Start a new intake cycle
            intakeManager.startCycle(ballPose, ballIndex, /*timeout*/ 2500);
            readyForIntake = false; // we are now in progress
        }

        // Check if intake cycle finished
        if (intakeManager.getState() == IntakeManager.IntakeState.DONE) {
            IntakeManager.IntakeResult res = intakeManager.getResult();
            int count = intakeManager.getBallCount();

            if (timeLow()) {
                // Whatever happened, go park
                readyForPark = true;
                readyForShoot = false;
                readyForIntake = false;
                intakeManager.acknowledgeDone();
                mode = Mode.PARK_MODE;
                return;
            }

            switch (res) {
                case GOT_BALLS:
                    // We have some balls; if enough time, go to SHOOT_MODE
                    if (timeEnoughForIntakeAndShoot()) {
                        readyForShoot = true;
                        readyForIntake = false;
                        // stay on same shootIndex, since shoot spot corresponds to this ball spot
                        mode = Mode.SHOOT_MODE;
                    } else {
                        // Not enough time to shoot, park
                        readyForPark = true;
                        mode = Mode.PARK_MODE;
                    }
                    break;

                case NO_BALL:
                    // Move to next ball spot if time, otherwise park
                    ballIndex++;
                    if (moreBallSpots() && !timeLow()) {
                        readyForIntake = true;
                        mode = Mode.INTAKE_MODE;
                    } else {
                        readyForPark = true;
                        mode = Mode.PARK_MODE;
                    }
                    break;

                case ABORTED:
                default:
                    // Safest fallback: go park
                    readyForPark = true;
                    mode = Mode.PARK_MODE;
                    break;
            }

            // Mark that we've consumed this DONE
            intakeManager.acknowledgeDone();
        }
    }

    private void handleShootMode() {
        if (readyForShoot
                && shootManager.getState() == ShootManager.ShootState.IDLE
                && moreShootSpots()
                && !timeLow()) {

            Pose2D shootPose = SHOOT_POS[shootIndex];
            // For now, shoot all balls we have (GameManager might track this)
            int ballsToShoot = 3; // or from IntakeManager.getBallCount()
            shootManager.startCycle(shootPose, shootIndex, ballsToShoot, 4000);
            readyForShoot = false;
        }

        if (shootManager.getState() == ShootManager.ShootState.DONE) {
            ShootManager.ShootResult res = shootManager.getResult();

            if (timeLow()) {
                readyForPark = true;
                shootManager.acknowledgeDone();
                mode = Mode.PARK_MODE;
                return;
            }

            switch (res) {
                case SUCCESS:
                case PARTIAL:
                    // Consider this spot processed, go to next
                    shootIndex++;
                    ballIndex++; // if 1:1 mapping of ball/shoot spots
                    if (moreBallSpots() && timeEnoughForIntakeAndShoot()) {
                        readyForIntake = true;
                        mode = Mode.INTAKE_MODE;
                    } else {
                        readyForPark = true;
                        mode = Mode.PARK_MODE;
                    }
                    break;

                case FAIL_ALIGN:
                case FAIL_JAM:
                default:
                    // Up to you: either try another shoot spot or just park.
                    readyForPark = true;
                    mode = Mode.PARK_MODE;
                    break;
            }

            shootManager.acknowledgeDone();
        }
    }

    private void handleParkMode() {
        // Here you just delegate to some "park" helper.
        // For now, fake it as instant; in real code you'd call chassis.drive(PARK_POS)
        // and then set mode= DONE when that finishes.

        readyForIntake = false;
        readyForShoot = false;
        readyForPark = false;
        mode = Mode.DONE;
    }
    public boolean isDone() {
        return mode == Mode.DONE;
    }
}

