package org.firstinspires.ftc.teamcode.ReferenceNewTemplateOptionB;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Helper.Chassis;
import org.firstinspires.ftc.teamcode.Helper.GoBildaPinpointDriver;

public class DriveManager {


    public enum DriveState {
        IDLE,       // nothing currently running
        MOVING,     // driving toward target position
        ALIGNING,   // final alignment step (optional)
        DONE        // cycle finished
    }

    public enum DriveResult {
        NONE,           // no cycle yet
        ARRIVED_OK,     // reached target pose successfully
        ALIGNED,        // reached target heading successfully
        ALIGN_FAILED,   // couldn’t align at end
        PATH_FAILED,    // couldn’t reach target (timeout, large error)
        ABORTED         // cancelled by GameManager
    }

    // What kind of goal this cycle is about
    public enum DriveGoalKind {
        NONE,
        GOTO_BALL_SPOT,
        GOTO_SHOOT_SPOT,
        GOTO_PARK,
        PURE_ALIGN_TAG   // e.g. only alignment around current spot
    }


    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;
    private final GoBildaPinpointDriver odo;
    private final IMU imu;
    private final Telemetry telemetry;

    private final ElapsedTime timer = new ElapsedTime();

    private DriveState  state  = DriveState.IDLE;
    private DriveResult result = DriveResult.NONE;
    private DriveGoalKind goalKind = DriveGoalKind.NONE;

    // Target parameters for current cycle
    private double targetXInch;
    private double targetYInch;
    private double targetHeadingRad;
    private double timeoutSec = 5.0;


    private final Chassis drive;
    private double targetHeadingDeg;

    private Pose2D targetPose;

    public DriveManager (Chassis chassis, Telemetry telemetry) {
        this.frontLeft  = chassis.frontLeftDrive;
        this.frontRight = chassis.frontRightDrive;
        this.backLeft   = chassis.backLeftDrive;
        this.backRight  = chassis.backRightDrive;
        this.odo        = chassis.odo;
        this.imu        = chassis.imu;
        this.telemetry  = telemetry;
        this.drive     = chassis;
    }

    // ---------- Public API for GameManager ----------

    public DriveState getState()        { return state; }
    public DriveResult getResult()      { return result; }
    public DriveGoalKind getGoalKind()  { return goalKind; }

    public boolean isIdle() { return state == DriveState.IDLE; }
    public boolean isDone() { return state == DriveState.DONE; }

    /** Hard stop + clear result. */
    public void resetCycle() {

        //TODO, make sure we can stopMotors();
        state     = DriveState.IDLE;
        result    = DriveResult.NONE;
        goalKind  = DriveGoalKind.NONE;
    }

    public void abortCycle() {
        if (state == DriveState.IDLE) return;
        //TODO, make sure we can stopMotors();
        state  = DriveState.DONE;
        result = DriveResult.ABORTED;
    }
    // ------------------------------------------------------------
    // START NEW CYCLES
    // ------------------------------------------------------------

    /** Generalized startCycle() for MOVE goals (ball spot, shoot spot, park). */
    public void startCycle(DriveGoalKind kind, Pose2D target, double timeoutSec) {
        if (state != DriveState.IDLE) return;

        this.goalKind = kind;
        this.targetPose = target;
        this.timeoutSec = timeoutSec;

        state = DriveState.MOVING;
        result = DriveResult.NONE;

        timer.reset();

        telemetry.addData("Drive", "Start MOVE cycle: %s to (%.1f, %.1f, %.1f°)",
                kind, target.getX(DistanceUnit.CM), target.getY(DistanceUnit.CM), target.getHeading(AngleUnit.RADIANS));
    }

    /** startCycle() for ALIGN-ONLY mode (tag or heading). */
    public void startAlignCycle(double headingDeg, double timeoutSec) {
        if (state != DriveState.IDLE) return;

        this.goalKind = DriveGoalKind.PURE_ALIGN_TAG;
        this.targetHeadingDeg = headingDeg;
        this.timeoutSec = timeoutSec;

        state = DriveState.ALIGNING;
        result = DriveResult.NONE;
        timer.reset();

        telemetry.addData("Drive", "Start ALIGN cycle to %.1f°", headingDeg);
    }

    // ------------------------------------------------------------
    // UPDATE LOOP (non-blocking)
    // ------------------------------------------------------------

    public void update(double nowSec) {

        if (state == DriveState.IDLE || state == DriveState.DONE)
            return;

        if (timer.seconds() > timeoutSec) {
            finish(
                    (state == DriveState.ALIGNING)
                            ? DriveResult.ALIGN_FAILED
                            : DriveResult.PATH_FAILED
            );
            return;
        }

        switch (state) {

            case MOVING:
                boolean arrived = true; //TODO: need to implement a method that Chassis did arrive at the required position
                if (arrived) {
                    result = DriveResult.ARRIVED_OK;
                    // If this cycle requires alignment too:
                    if (goalKind == DriveGoalKind.GOTO_SHOOT_SPOT ||
                            goalKind == DriveGoalKind.GOTO_BALL_SPOT) {
                        // For precise goal alignment (e.g., tag or heading)
                        state = DriveState.ALIGNING;
                        timer.reset();
                    } else {
                        finish(result);
                    }
                }
                break;

            case ALIGNING:
                boolean aligned = true; // TODO implement alignHeadingNonBlocking(targetHeadingDeg);
                if (aligned) {
                    finish(DriveResult.ALIGNED);
                }
                break;

            default:
                break;
        }
    }

    // ------------------------------------------------------------
    // FINISH HANDLER
    // ------------------------------------------------------------

    private void finish(DriveResult finalResult) {
        // TODO stop all motors too
        state = DriveState.DONE;
        result = finalResult;

        telemetry.addData("Drive", "Cycle DONE: %s", finalResult);
    }

}