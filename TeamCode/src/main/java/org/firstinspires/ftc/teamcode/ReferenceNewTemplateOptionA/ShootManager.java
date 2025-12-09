package org.firstinspires.ftc.teamcode.ReferenceNewTemplateOptionA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.Helper.*;

public class ShootManager {

    public enum ShootState {
        IDLE,
        DRIVING_TO_SHOT,
        ALIGNING_TAG,
        SPINUP,
        SHOOTING,
        DONE
    }

    public enum ShootResult {
        NONE,
        SUCCESS,
        PARTIAL,
        FAIL_ALIGN,
        FAIL_JAM
    }

    private final Chassis chassis;
    private final FlyWheel flyWheel;
    private final Flipper flipper;
    private final Kicker kicker;
    private final Intake intake;
    private final DecodeAprilTag aprilTag;
    private final Telemetry telemetry;

    private ShootState state  = ShootState.IDLE;
    private ShootResult result = ShootResult.NONE;

    private int targetSpotIndex = -1;
    private int ballsToShoot = 0;
    private long timeoutMs = 4000;
    private long startTimeMs;

    private final String targetTagName = "SOME_TAG_NAME"; // configure

    public ShootManager(Chassis chassis,
                        FlyWheel flyWheel,
                        Flipper flipper,
                        Kicker kicker,
                        Intake intake,
                        DecodeAprilTag aprilTag,
                        Telemetry telemetry) {
        this.chassis = chassis;
        this.flyWheel = flyWheel;
        this.flipper = flipper;
        this.kicker = kicker;
        this.intake = intake;
        this.aprilTag = aprilTag;
        this.telemetry = telemetry;
    }

    public ShootState getState()   { return state; }
    public ShootResult getResult() { return result; }

    /**
     * Called once when GameManager decides "we are in SHOOT_MODE for this spot".
     */
    public void startCycle(Pose2D shootPose, int spotIndex, int ballsToShoot, long timeoutMs) {
        if (state != ShootState.IDLE && state != ShootState.DONE) {
            return;
        }
        this.targetSpotIndex = spotIndex;
        this.ballsToShoot = ballsToShoot;
        this.timeoutMs = timeoutMs;
        this.result = ShootResult.NONE;

        // Start driving to the shooting pose
        //TODO we need to call DriveManager here and then drive to the shootPose;
        // chassis.drive(shootPose.getX(DistanceUnit.CM), shootPose.getY(DistanceUnit.CM), shootPose.getHeading(AngleUnit.RADIANS));
        state = ShootState.DRIVING_TO_SHOT;
        startTimeMs = System.currentTimeMillis();
    }

    public void update() {
        long elapsed = System.currentTimeMillis() - startTimeMs;

        switch (state) {
            case IDLE:
            case DONE:
                break;

            case DRIVING_TO_SHOT:
                //TODO, if statement below shall also check if the robotics is at target position already
                //ideally: if(chassis.isAtTargetPose() || elapsed > timeoutMs)
                if (elapsed > timeoutMs) {
                    // If we timed out before reaching exact pose, we still TRY to align.
                    state = ShootState.ALIGNING_TAG;
                }
                break;

            case ALIGNING_TAG:
                // Use your Util.autoAlignWithAprilTag helper (blocking), or
                // break it into smaller steps. For now, call it once when entering:

                Util.AlignmentResult alignResult =
                        Util.autoAlignWithAprilTag(chassis.getOpMode(),
                                aprilTag,
                                targetTagName,
                                chassis,
                                telemetry);

                if (!alignResult.success) {
                    result = ShootResult.FAIL_ALIGN;
                    state = ShootState.DONE;
                } else {
                    // distance used for velocity calculations
                    double distanceInch = alignResult.distance;
                    // Spinup is done in one blocking helper call for now:
                    Util.prepareForShooting(flyWheel, kicker, flipper, intake, distanceInch, telemetry);
                    state = ShootState.SHOOTING;
                }
                break;

            case SHOOTING:
                // Use your existing Util.shoot() helper.
                // It already handles jam clearing etc. and returns to intake mode.
                // You could extend it to return a result; for now we assume SUCCESS.

                // Example (blocking call):
                Util.shoot(flyWheel, kicker, flipper, intake,
                        /* robotDistanceFromTag (fallback) */ 50.0,
                        aprilTag, targetTagName, telemetry);

                result = ShootResult.SUCCESS; // or PARTIAL/JAM if you extend Util.shoot
                state = ShootState.DONE;
                break;
        }

        telemetry.addData("ShootState", state);
        telemetry.addData("ShootResult", result);
    }

    public void acknowledgeDone() {
        if (state == ShootState.DONE) {
            state = ShootState.IDLE;
        }
    }

    public void abort() {
        // clean up: stop flywheel, reset mechanisms, etc.
        flyWheel.stop();
        kicker.setGatePosition(Kicker.GATE_CLOSE);
        flipper.resetFlipper();
        result = ShootResult.FAIL_JAM; // or ABORT
        state = ShootState.DONE;
    }
}