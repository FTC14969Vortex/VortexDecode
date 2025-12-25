package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.external.gobilida.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.motion.FieldPose;
import org.firstinspires.ftc.teamcode.motion.MotionExecutor;
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
@TeleOp(name = "Camera Servo Test", group = "Debug")
public class CameraServoTest extends OpMode {
    
    // Vision system
    private CameraServo cameraServo;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    
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
        
        // Initialize vision system
        initVisionSystem();
        
        // Initialize camera servo system
        cameraServo = new CameraServo();
        cameraServo.init(
            hardwareMap, 
            aprilTagProcessor, 
            motionExecutor.getMotionState().getOdometryManager(),
            motionExecutor.getCoordinateTransformer(),
            motionExecutor
        );
        cameraServo.setTargetTag(20); // Set target to blue GOAL (tag 20)
        
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
        if (visionPortal != null) {
            visionPortal.close();
        }
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
            cameraServo.center(); // Center camera servo
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
        telemetry.addData("X", "%.1f inches", robotPose.x);
        telemetry.addData("Y", "%.1f inches", robotPose.y);
        telemetry.addData("Heading", "%.1f°", robotPose.heading);
        telemetry.addLine("");
        
        // ========== CAMERA SERVO ==========
        telemetry.addLine("=== CAMERA SERVO ===");
        telemetry.addData("Current Angle", "%.1f°", cameraServo.getCurrentAngle());
        telemetry.addData("Target Angle", "%.1f°", cameraServo.getTargetAngle());
        telemetry.addData("Moving", cameraServo.isMoving() ? "YES" : "NO");
        telemetry.addData("Searching", cameraServo.isSearching() ? "YES" : "NO");
        telemetry.addLine("");
        
        // ========== TAG DETECTION ==========
        telemetry.addLine("=== TAG DETECTION ===");
        telemetry.addData("Target Tag", "Blue Goal (Tag 20)");
        telemetry.addData("Last Detected Tag", cameraServo.getLastDetectedTagId());
        telemetry.addData("Distance", "%.1f inches", cameraServo.getLastDetectedDistance());
        telemetry.addData("Time Since Detection", "%.1f sec", cameraServo.getTimeSinceLastDetection() / 1000.0);
        
        // Detection status indicator
        if (cameraServo.getTimeSinceLastDetection() < 1000) {
            telemetry.addData("Detection Status", "🟢 ACTIVE");
        } else if (cameraServo.getTimeSinceLastDetection() < 3000) {
            telemetry.addData("Detection Status", "🟡 RECENT");
        } else {
            telemetry.addData("Detection Status", "🔴 LOST");
        }
        telemetry.addLine("");
        
        // ========== CONTROLS ==========
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("Dpad Left/Right: Rotate robot 90°");
        telemetry.addLine("A: Start camera search");
        telemetry.addLine("B: Center camera");
        telemetry.addLine("X: Aim at blue goal");
        
        telemetry.update();
    }
    
    /**
     * Initialize vision system (AprilTag processor and VisionPortal)
     */
    private void initVisionSystem() {
        // Initialize AprilTag processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setOutputUnits(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH,
                               org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES)
                .build();
        
        // Initialize vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();
    }
}
