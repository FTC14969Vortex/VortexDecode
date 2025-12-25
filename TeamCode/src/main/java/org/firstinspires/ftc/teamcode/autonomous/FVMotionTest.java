package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.external.gobilida.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.motion.FieldPose;
import org.firstinspires.ftc.teamcode.motion.MotionExecutor;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.vision.CameraServo;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Simple Camera Servo Test with Robot Rotation
 *
 * Tests camera servo system by rotating robot in 90-degree steps
 * and checking if camera can stably detect blue goal tag.
 *
 * Controls:
 * - Dpad Left: Rotate robot LEFT 90 degrees
 * - Dpad Right: Rotate robot RIGHT 90 degrees
 * - A button: Start camera search pattern
 * - B button: Center camera servo
 *
 * Camera automatically aims at blue goal tag (Tag 20) when detected.
 */
@Autonomous(name = " FVMotionTest 0.16", group = "Debug")
public class FVMotionTest extends OpMode {

    Intake intake;


    // Motion system
    private MotionExecutor motionExecutor;

    // Robot state
    private FieldPose currentRobotPose = new FieldPose(0.0, 0.0, 0.0);

    // Control state
    private boolean leftPressed = false;
    private boolean rightPressed = false;
    private boolean aPressed = false;
    private boolean bPressed = false;

    @Override
    public void init() {

        // ========== INITIALIZATION ==========

        telemetry.addLine("Initializing Camera Servo Test...");
        telemetry.update();

        // Initialize motors
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeftDrive");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRightDrive");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeftDrive");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRightDrive");

        // Initialize odometry (MotionExecutor handles this internally)
        GoBildaPinpointDriver odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Initialize motion executor (handles odometry configuration internally via OdometryManager)
        motionExecutor = new MotionExecutor(frontLeft, frontRight, backLeft, backRight, odometry);


        intake = new Intake();
        intake.init(this);
    }

    @Override
    public void start() {

        // Update motion state and get current robot pose from MotionExecutor
        motionExecutor.updateState();
        Pose2D currentPose = motionExecutor.getMotionState().getCurrentPose();
        currentRobotPose = new FieldPose(
                currentPose.getX(DistanceUnit.INCH),
                currentPose.getY(DistanceUnit.INCH),
                currentPose.getHeading(AngleUnit.DEGREES)
        );

        // No threading needed - OpMode loop handles updates
    }

    int stepper = 0;
    @Override
    public void loop() {



        if (stepper == 0) {
            //intake.startIntake();
            //motionExecutor.linearMove(45, 67,90, 40);

            motionExecutor.moveToPose(0, 0, 90, 10);
            //motionExecutor.moveToPose(0, 0, 180);
            stepper++;
        } /*else if (stepper == 1) {
            motionExecutor.moveToPose(72, 28, 90);
            stepper++;
        } else if (stepper == 2) {
            motionExecutor.moveToPose(0, 24, -90);
            stepper++;
        }else if (stepper == 3) {
            motionExecutor.moveToPose(0, 0, 0);
            stepper++;
        }*/


        // Handle gamepad controls
        handleControls();

        motionExecutor.updateState();
        Pose2D currentPose = motionExecutor.getMotionState().getCurrentPose();
        currentRobotPose = new FieldPose(
                currentPose.getX(DistanceUnit.INCH),
                currentPose.getY(DistanceUnit.INCH),
                currentPose.getHeading(AngleUnit.DEGREES)
        );

        // Display telemetry
        displayTelemetry();
    }

    @Override
    public void stop() {
        // Cleanup vision portal
    }


    /**
     * Handle gamepad controls for robot rotation and camera
     */
    private void handleControls() {
        Gamepad gamepad = gamepad1;

        // Robot rotation controls (90-degree steps)
        if (gamepad.dpad_left && !leftPressed) {
            leftPressed = true;
        } else if (!gamepad.dpad_left) {
            leftPressed = false;
        }

        if (gamepad.dpad_right && !rightPressed) {
            rightPressed = true;
            telemetry.addLine("🔄 Rotating RIGHT 90°...");
            telemetry.update();
            motionExecutor.rotate(-90.0); // Rotate right 90 degrees
        } else if (!gamepad.dpad_right) {
            rightPressed = false;
        }

        // Camera controls
        if (gamepad.a && !aPressed) {
            aPressed = true;
            telemetry.addLine("🔍 Starting camera search...");
        } else if (!gamepad.a) {
            aPressed = false;
        }

        if (gamepad.b && !bPressed) {
            bPressed = true;
            telemetry.addLine("📍 Centering camera...");
        } else if (!gamepad.b) {
            bPressed = false;
        }

        // X button removed - camera auto-aims at blue goal tag when detected
    }

    /**
     * Display simple telemetry for camera servo test
     */
    private void displayTelemetry() {

        // ========== ROBOT POSITION ==========
        telemetry.addLine("=== ROBOT POSITION ===");
        telemetry.addData("X", "%.1f inches", currentRobotPose.x);
        telemetry.addData("Y", "%.1f inches", currentRobotPose.y);
        telemetry.addData("Heading", "%.1f°", currentRobotPose.heading);
        telemetry.addLine("");

        telemetry.update();
    }
        }
