package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.external.gobilida.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.motion.FieldPose;
import org.firstinspires.ftc.teamcode.motion.MotionExecutor;
import org.firstinspires.ftc.teamcode.vision.CameraServo;
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
@TeleOp(name = "Camera Servo Test", group = "Debug")
public class CameraServoTest extends OpMode {

    // Vision system
    private CameraServo cameraServo;

    // Motion system
    private MotionExecutor motionExecutor;

    // Robot state
    private FieldPose robotPose = new FieldPose(0.0, 0.0, 0.0);

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

        // Initialize camera servo system (handles vision initialization internally)
        cameraServo = new CameraServo();
        cameraServo.init(
                hardwareMap,
                motionExecutor.getMotionState().getOdometryManager(),
                motionExecutor.getCoordinateTransformer(),
                motionExecutor
        );
        cameraServo.setTargetTag(20); // Set target to blue GOAL (tag 20)
        cameraServo.setAutoOdometryCorrection(false);

        telemetry.addLine("✅ Systems Ready!");
        telemetry.addLine("");
        telemetry.addLine("Controls:");
        telemetry.addLine("Dpad Left: Rotate robot LEFT 90°");
        telemetry.addLine("Dpad Right: Rotate robot RIGHT 90°");
        telemetry.addLine("A: Start camera search");
        telemetry.addLine("B: Center camera servo");
        telemetry.addLine("Camera auto-aims at blue goal tag");
        telemetry.update();
    }

    @Override
    public void start() {
        // No threading needed - OpMode loop handles updates
    }

    @Override
    public void loop() {

        // Update motion state and get current robot pose from MotionExecutor
        motionExecutor.updateState();
        Pose2D currentPose = motionExecutor.getMotionState().getCurrentPose();
        robotPose = new FieldPose(
                currentPose.getX(DistanceUnit.INCH),
                currentPose.getY(DistanceUnit.INCH),
                currentPose.getHeading(AngleUnit.DEGREES)
        );

        // Update camera servo system
        cameraServo.update();

        // Auto-aim at target tag if detected (no manual X button needed)
        if (cameraServo.getTimeSinceLastDetection() < 1000) {
            cameraServo.aimAtTag(robotPose, 20); // Auto-aim at blue goal tag
        }

        // Handle gamepad controls
        handleControls();

        // Display telemetry
        displayTelemetry();
    }

    @Override
    public void stop() {
        // Cleanup vision portal
        cameraServo.cleanup();
    }



    /**
     * Handle gamepad controls for robot rotation and camera
     */
    private void handleControls() {
        Gamepad gamepad = gamepad1;

        // Robot rotation controls (90-degree steps)
        if (gamepad.dpad_left && !leftPressed) {
            leftPressed = true;
            telemetry.addLine("🔄 Rotating LEFT 90°...");
            telemetry.update();
            motionExecutor.rotate(90.0); // Rotate left 90 degrees
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
            cameraServo.startSearch(0.0); // Start search pattern around center
            telemetry.addLine("🔍 Starting camera search...");
        } else if (!gamepad.a) {
            aPressed = false;
        }

        if (gamepad.b && !bPressed) {
            bPressed = true;
            cameraServo.moveToCenter(); // Center camera servo
            telemetry.addLine("📍 Centering camera...");
        } else if (!gamepad.b) {
            bPressed = false;
        }

        // X button removed - camera auto-aims at blue goal tag when detected
    }

    /**
     * Display comprehensive telemetry for camera servo debugging
     */
    private void displayTelemetry() {

        // Get current odometry reading directly from motion executor
        Pose2D currentOdoPose = motionExecutor.getMotionState().getCurrentPose();

        // Get diagnostic data from camera servo
        double bearing = cameraServo.getLastDetectedBearing();
        double yaw = cameraServo.getLastDetectedYaw();
        double elevation = cameraServo.getLastDetectedElevation();
        double ftcPoseX = cameraServo.getLastFtcPoseX();
        double ftcPoseY = cameraServo.getLastFtcPoseY();
        double camFieldHeading = cameraServo.getLastCameraPosition().heading;
        long timeSinceDetection = cameraServo.getTimeSinceLastDetection();

        // ========== ODOMETRY READING ==========
        telemetry.addLine("=== CURRENT ODOMETRY ===");
        telemetry.addData("ODO X", "%.2f inches", currentOdoPose.getX(DistanceUnit.INCH));
        telemetry.addData("ODO Y", "%.2f inches", currentOdoPose.getY(DistanceUnit.INCH));
        telemetry.addData("ODO Heading", "%.2f°", currentOdoPose.getHeading(AngleUnit.DEGREES));
        telemetry.addLine("");

        // ========== ROBOT POSITION ==========
        telemetry.addLine("=== ROBOT POSITION ===");
        telemetry.addData("X", "%.1f inches", robotPose.x);
        telemetry.addData("Y", "%.1f inches", robotPose.y);
        telemetry.addData("Heading", "%.1f°", robotPose.heading);
        telemetry.addLine("");

        // ========== CAMERA SERVO ==========
        telemetry.addLine("=== CAMERA SERVO ===");
        telemetry.addData("Current Angle", "%.1f°", cameraServo.getCurrentAngle());
        telemetry.addData("Target Angle", "%.1f°", cameraServo.getTargetAngle());
        telemetry.addData("Searching", cameraServo.isSearching() ? "YES" : "NO");
        telemetry.addData("Auto ODO Correction", cameraServo.isAutoOdometryCorrectionEnabled() ? "ON" : "OFF");
        telemetry.addLine("");

        // ========== VISION-BASED POSITION ==========
        FieldPose visionPosition = cameraServo.getVisionBasedPosition();
        if (visionPosition != null) {
            telemetry.addLine("=== VISION-BASED POSITION ===");
            telemetry.addData("Vision X", "%.2f inches", visionPosition.x);
            telemetry.addData("Vision Y", "%.2f inches", visionPosition.y);
            telemetry.addData("Vision Heading", "%.2f°", visionPosition.heading);
            telemetry.addLine("");

            // ========== POSITION COMPARISON ==========
            telemetry.addLine("=== POSITION COMPARISON ===");
            double deltaX = visionPosition.x - currentOdoPose.getX(DistanceUnit.INCH);
            double deltaY = visionPosition.y - currentOdoPose.getY(DistanceUnit.INCH);
            double deltaHeading = visionPosition.heading - currentOdoPose.getHeading(AngleUnit.DEGREES);
            telemetry.addData("Delta X", "%.2f inches", deltaX);
            telemetry.addData("Delta Y", "%.2f inches", deltaY);
            telemetry.addData("Delta Heading", "%.2f°", deltaHeading);
            telemetry.addLine("");
        } else {
            telemetry.addLine("=== VISION-BASED POSITION ===");
            telemetry.addData("Status", "No recent detection");
            telemetry.addLine("");
        }

        // ========== TAG DETECTION ==========
        telemetry.addLine("=== TAG DETECTION ===");
        telemetry.addData("Target Tag", "Blue Goal (Tag 20)");
        telemetry.addData("Last Detected Tag", cameraServo.getLastDetectedTagId());
        telemetry.addData("Distance", "%.1f inches", cameraServo.getLastDetectedDistance());
        telemetry.addData("Time Since Detection", "%.1f sec", timeSinceDetection / 1000.0);
        telemetry.addData("ftcPose.x", "%.2f", ftcPoseX);
        telemetry.addData("ftcPose.y", "%.2f", ftcPoseY);
        telemetry.addData("Cam Field Heading", "%.1f°", camFieldHeading);

        // Detection status indicator with detailed diagnostics
        if (timeSinceDetection < 1000) {
            telemetry.addData("Detection Status", "🟢 ACTIVE");
            telemetry.addData("Camera Bearing", "%.1f°", bearing);
            telemetry.addData("Tag Yaw", "%.1f°", yaw);
            telemetry.addData("Camera Elevation", "%.1f°", elevation);
        } else if (timeSinceDetection < 3000) {
            telemetry.addData("Detection Status", "🟡 RECENT");
            telemetry.addData("Last Camera Bearing", "%.1f°", bearing);
            telemetry.addData("Last Tag Yaw", "%.1f°", yaw);
            telemetry.addData("Last Camera Elevation", "%.1f°", elevation);
        } else {
            telemetry.addData("Detection Status", "🔴 LOST");
            telemetry.addLine("Position robot to see AprilTag!");
        }
        telemetry.addLine("");

        // ========== ENHANCED DIAGNOSTICS ==========
        if (timeSinceDetection < 3000) { // Show if we have recent detection data
            telemetry.addLine("=== ENHANCED DIAGNOSTICS ===");

            // Camera position from last detection
            FieldPose lastCameraPos = cameraServo.getLastCameraPosition();
            telemetry.addData("Camera Pos X", "%.2f inches", lastCameraPos.x);
            telemetry.addData("Camera Pos Y", "%.2f inches", lastCameraPos.y);
            telemetry.addData("Camera Heading", "%.1f°", lastCameraPos.heading);

            // Servo position from last detection
            FieldPose lastServoPos = cameraServo.getLastServoPosition();
            telemetry.addData("Servo Pos X", "%.2f inches", lastServoPos.x);
            telemetry.addData("Servo Pos Y", "%.2f inches", lastServoPos.y);
            telemetry.addData("Servo Heading", "%.1f°", lastServoPos.heading);

            // Robot alignment angle for shooting
            double alignmentAngle = cameraServo.getShootingAngle();
            telemetry.addData("Shoot Align Angle", "%.1f°", alignmentAngle);
            telemetry.addData("Align Formula", "angleToTag - robotHeading - 180°");

            // Show whether angle is from vision or odometry
            if (timeSinceDetection < 1000) {
                telemetry.addData("Angle Source", "🎯 VISION (AprilTag detected)");
            } else {
                telemetry.addData("Angle Source", "🧭 ODOMETRY (fallback)");
            }

            telemetry.addLine("");
        }

        // ========== FLYWHEEL VELOCITY DEBUG ==========
        if (timeSinceDetection < 3000) { // Show if we have recent detection data
            telemetry.addLine("=== FLYWHEEL VELOCITY DEBUG ===");
            double distance = cameraServo.getLastDetectedDistance();
            double flywheelVelocity = cameraServo.getFlywheelVelocity();
            telemetry.addData("Calculated Distance", "%.1f inches", distance);
            telemetry.addData("Target Velocity", "%.0f RPM", flywheelVelocity);
            telemetry.addData("Velocity Formula", "8 RPM/inch + 800 RPM base");

            // Show distance calculation source
            if (timeSinceDetection < 1000) {
                telemetry.addData("Distance Source", "🎯 VISION (robot center to tag)");
            } else {
                telemetry.addData("Distance Source", "🧭 ODOMETRY (robot center to tag)");
            }
            telemetry.addLine("");
        }

        // ========== CONTROLS ==========
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("Dpad Left/Right: Rotate robot 90°");
        telemetry.addLine("A: Start camera search");
        telemetry.addLine("B: Center camera");
        telemetry.addLine("X: Aim at blue goal");

        telemetry.update();
    }


}