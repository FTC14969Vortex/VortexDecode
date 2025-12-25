package org.firstinspires.ftc.teamcode.debug;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.external.gobilida.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.motion.FieldPose;
import org.firstinspires.ftc.teamcode.motion.FieldPositions;
import org.firstinspires.ftc.teamcode.motion.MotionExecutor;

import org.firstinspires.ftc.teamcode.vision.CameraServo;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Enhanced Camera Servo Test with FTCDashboard Field Visualization
 * 
 * Demonstrates vision-based localization with real-time field visualization:
 * - Robot position tracking with odometry
 * - AprilTag detection and camera servo control
 * - Vision-based pose corrections
 * - Real-time field canvas showing robot and tag positions
 * - Teleop drive controls for field movement
 * 
 * Controls:
 * - Left Stick: Drive (forward/backward, strafe left/right)
 * - Right Stick X: Turn left/right
 * - Dpad Left/Right: Rotate robot 90 degrees (for testing)
 * - A: Start camera search pattern
 * - B: Center camera servo
 * - X: Toggle target tag (Blue/Red goal)
 * - Y: Reset odometry to (0,0,0)
 * 
 * FTCDashboard Features:
 * - Field canvas (144" x 144" FTC field)
 * - Robot position (real-time updates)
 * - AprilTag positions (Red/Blue goals, Obelisk)
 * - Vision correction indicators
 * - Camera servo angle visualization
 */
@Config
@TeleOp(name = "Camera Servo Test Enhanced", group = "Debug")
public class CameraServoTestEnhanced extends OpMode {
    
    // ========== DASHBOARD CONFIGURATION ==========
    
    /** Field canvas size in inches (FTC field is 144" x 144") */
    public static double FIELD_SIZE_INCHES = 144.0;
    
    /** Dashboard update rate (Hz) */
    public static double DASHBOARD_UPDATE_HZ = 20.0;
    
    /** Robot size for visualization (inches) */
    public static double ROBOT_SIZE_INCHES = 18.0;
    
    /** AprilTag marker size (inches) */
    public static double TAG_SIZE_INCHES = 6.0;
    
    /** Show vision corrections on dashboard */
    public static boolean SHOW_VISION_CORRECTIONS = true;
    
    /** Show camera servo angle visualization */
    public static boolean SHOW_CAMERA_SERVO = true;
    
    /** Drive power multiplier (0.0 to 1.0) */
    public static double DRIVE_POWER = 0.7;
    
    /** Turn power multiplier (0.0 to 1.0) */
    public static double TURN_POWER = 0.5;
    
    // ========== HARDWARE ==========
    
    // Vision system
    private CameraServo cameraServo;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    
    // Motion system
    private MotionExecutor motionExecutor;
    
    // Motors
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    
    // ========== STATE TRACKING ==========
    
    // Robot pose
    private FieldPose robotPose = new FieldPose(0.0, 0.0, 0.0);
    private FieldPose lastVisionPose = null;
    private long lastVisionCorrectionTime = 0;
    
    // Target tag selection
    private int targetTagId = 20; // Default to blue goal
    private boolean targetTagToggled = false;
    
    // Control state
    private boolean leftPressed = false;
    private boolean rightPressed = false;
    private boolean aPressed = false;
    private boolean bPressed = false;
    private boolean xPressed = false;
    private boolean yPressed = false;
    
    // Dashboard timing
    private ElapsedTime dashboardTimer = new ElapsedTime();
    private FtcDashboard dashboard;
    
    @Override
    public void init() {
        
        // ========== INITIALIZATION ==========
        
        telemetry.addLine("Initializing Enhanced Camera Servo Test...");
        telemetry.update();
        
        // Initialize FTC Dashboard
        dashboard = FtcDashboard.getInstance();
        dashboardTimer.reset();
        
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeftDrive");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRightDrive");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeftDrive");
        backRight = hardwareMap.get(DcMotorEx.class, "backRightDrive");
        
        // Initialize odometry
        GoBildaPinpointDriver odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        
        // Initialize motion system components
        motionExecutor = new MotionExecutor(frontLeft, frontRight, backLeft, backRight, odometry);
        
        // Initialize vision system
        initVisionSystem();
        
        // Initialize camera servo system with FULL integration for automatic odometry correction
        // CRITICAL: Must pass odometryManager and coordinateTransformer for vision-based pose corrections
        cameraServo = new CameraServo();
        cameraServo.init(
            hardwareMap, 
            aprilTagProcessor, 
            motionExecutor.getMotionState().getOdometryManager(),
            motionExecutor.getCoordinateTransformer(),
            motionExecutor
        );
        cameraServo.setTargetTag(targetTagId); // Set target to blue GOAL (tag 20)
        
        telemetry.addLine("✅ Systems Ready!");
        telemetry.addLine("");
        telemetry.addLine("Controls:");
        telemetry.addLine("Left Stick: Drive (forward/strafe)");
        telemetry.addLine("Right Stick X: Turn");
        telemetry.addLine("Dpad Left/Right: Rotate 90°");
        telemetry.addLine("A: Camera search | B: Center camera");
        telemetry.addLine("X: Toggle target tag | Y: Reset odometry");
        telemetry.addLine("");
        telemetry.addLine("🎯 Check FTC Dashboard for field visualization!");
        telemetry.update();
    }
    
    @Override
    public void start() {
        // Reset timers
        dashboardTimer.reset();
    }
    
    @Override
    public void loop() {
        
        // Update motion state and get current robot pose
        motionExecutor.updateState();
        updateRobotPose();
        
        // Update camera servo system
        cameraServo.update();
        
        // Auto-aim at target tag if detected
        if (cameraServo.getTimeSinceLastDetection() < 1000) {
            cameraServo.aimAtTag(robotPose, targetTagId);
            
            // Check for vision corrections
            if (SHOW_VISION_CORRECTIONS) {
                checkForVisionCorrections();
            }
        }
        
        // Handle gamepad controls
        handleTeleopControls();
        handleCameraControls();
        
        // Update FTC Dashboard
        updateDashboard();
        
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
     * Update robot pose from motion system
     */
    private void updateRobotPose() {
        Pose2D currentPose = motionExecutor.getMotionState().getCurrentPose();
        robotPose = new FieldPose(
            currentPose.getX(DistanceUnit.INCH),
            currentPose.getY(DistanceUnit.INCH), 
            currentPose.getHeading(AngleUnit.DEGREES)
        );
    }
    
    /**
     * Check for vision-based pose corrections
     */
    private void checkForVisionCorrections() {
        // This is a simplified check - in practice, you'd compare odometry vs vision poses
        // For now, we'll just track when vision detections occur
        if (cameraServo.getTimeSinceLastDetection() < 100) { // Very recent detection
            lastVisionCorrectionTime = System.currentTimeMillis();
        }
    }
    
    /**
     * Handle teleop drive controls
     */
    private void handleTeleopControls() {
        Gamepad gamepad = gamepad1;
        
        // Get drive inputs
        double drive = -gamepad.left_stick_y * DRIVE_POWER;    // Forward/backward
        double strafe = gamepad.left_stick_x * DRIVE_POWER;    // Left/right
        double turn = gamepad.right_stick_x * TURN_POWER;      // Rotation
        
        // Apply mecanum drive
        double frontLeftPower = drive + strafe + turn;
        double frontRightPower = drive - strafe - turn;
        double backLeftPower = drive - strafe + turn;
        double backRightPower = drive + strafe - turn;
        
        // Normalize powers
        double maxPower = Math.max(Math.abs(frontLeftPower), 
                         Math.max(Math.abs(frontRightPower),
                         Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
        
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }
        
        // Set motor powers
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
        
        // Handle 90-degree rotation controls
        if (gamepad.dpad_left && !leftPressed) {
            leftPressed = true;
            motionExecutor.rotate(90.0); // Rotate left 90 degrees
        } else if (!gamepad.dpad_left) {
            leftPressed = false;
        }
        
        if (gamepad.dpad_right && !rightPressed) {
            rightPressed = true;
            motionExecutor.rotate(-90.0); // Rotate right 90 degrees
        } else if (!gamepad.dpad_right) {
            rightPressed = false;
        }
    }
    
    /**
     * Handle camera and system controls
     */
    private void handleCameraControls() {
        Gamepad gamepad = gamepad1;
        
        // Camera controls
        if (gamepad.a && !aPressed) {
            aPressed = true;
            cameraServo.startSearch(0.0); // Start search pattern around center
        } else if (!gamepad.a) {
            aPressed = false;
        }
        
        if (gamepad.b && !bPressed) {
            bPressed = true;
            cameraServo.center(); // Center camera servo
        } else if (!gamepad.b) {
            bPressed = false;
        }
        
        // Toggle target tag (Blue/Red goal)
        if (gamepad.x && !xPressed) {
            xPressed = true;
            targetTagId = (targetTagId == 20) ? 24 : 20; // Toggle between blue (20) and red (24)
            cameraServo.setTargetTag(targetTagId);
            targetTagToggled = true;
        } else if (!gamepad.x) {
            xPressed = false;
            targetTagToggled = false;
        }
        
        // Reset odometry
        if (gamepad.y && !yPressed) {
            yPressed = true;
            motionExecutor.resetOdometry();
        } else if (!gamepad.y) {
            yPressed = false;
        }
    }
    
    /**
     * Update FTC Dashboard with field visualization
     */
    private void updateDashboard() {
        // Update at configured rate (but ensure we send packets frequently enough)
        if (dashboardTimer.seconds() < (1.0 / DASHBOARD_UPDATE_HZ)) {
            return;
        }
        dashboardTimer.reset();
        
        // Create telemetry packet (keep default field image)
        TelemetryPacket packet = new TelemetryPacket();
        
        // ========== FIELD CANVAS ==========
        
        // Test rectangle at origin (should always be visible)
        packet.fieldOverlay()
            .setFill("#FF0000")
            .fillRect(-10, -10, 20, 20);
        
        // Set up field canvas (144" x 144" FTC field, origin at center)
        packet.fieldOverlay()
            .setStroke("#3F51B5")
            .setStrokeWidth(1)
            .strokeRect(-FIELD_SIZE_INCHES/2, -FIELD_SIZE_INCHES/2, FIELD_SIZE_INCHES, FIELD_SIZE_INCHES);
        
        // Draw field center lines
        packet.fieldOverlay()
            .setStroke("#9E9E9E")
            .setStrokeWidth(1)
            .strokeLine(0, -FIELD_SIZE_INCHES/2, 0, FIELD_SIZE_INCHES/2)  // Vertical center line
            .strokeLine(-FIELD_SIZE_INCHES/2, 0, FIELD_SIZE_INCHES/2, 0); // Horizontal center line
        
        // ========== APRILTAG POSITIONS ==========
        
        // Blue Goal (Tag 20)
        FieldPose blueGoal = FieldPositions.BLUE_GOAL_APRILTAG;
        if (blueGoal != null) {
            packet.fieldOverlay()
                .setFill("#2196F3")
                .fillRect(blueGoal.x - TAG_SIZE_INCHES/2, blueGoal.y - TAG_SIZE_INCHES/2, 
                         TAG_SIZE_INCHES, TAG_SIZE_INCHES);
        }
        
        // Red Goal (Tag 24)
        FieldPose redGoal = FieldPositions.RED_GOAL_APRILTAG;
        if (redGoal != null) {
            packet.fieldOverlay()
                .setFill("#F44336")
                .fillRect(redGoal.x - TAG_SIZE_INCHES/2, redGoal.y - TAG_SIZE_INCHES/2, 
                         TAG_SIZE_INCHES, TAG_SIZE_INCHES);
        }
        
        // Obelisk (Tag 21-23) - approximate position
        FieldPose obelisk = FieldPositions.OBELISK_POSITION;
        if (obelisk != null) {
            packet.fieldOverlay()
                .setFill("#FF9800")
                .fillCircle(obelisk.x, obelisk.y, TAG_SIZE_INCHES/2);
        }
        
        // ========== ROBOT VISUALIZATION ==========
        
        // Robot position (current) - ensure coordinates are reasonable
        double robotX = robotPose.x;
        double robotY = robotPose.y;
        double robotHeading = Math.toRadians(robotPose.heading);
        
        // Clamp robot position to field bounds for visualization
        robotX = Math.max(-FIELD_SIZE_INCHES/2 + ROBOT_SIZE_INCHES, 
                 Math.min(FIELD_SIZE_INCHES/2 - ROBOT_SIZE_INCHES, robotX));
        robotY = Math.max(-FIELD_SIZE_INCHES/2 + ROBOT_SIZE_INCHES, 
                 Math.min(FIELD_SIZE_INCHES/2 - ROBOT_SIZE_INCHES, robotY));
        
        // Robot body (rectangle) - always draw robot even if at origin
        packet.fieldOverlay()
            .setFill("#4CAF50")
            .setStroke("#2E7D32")
            .setStrokeWidth(2)
            .fillRect(robotX - ROBOT_SIZE_INCHES/2, robotY - ROBOT_SIZE_INCHES/2, 
                     ROBOT_SIZE_INCHES, ROBOT_SIZE_INCHES);
        
        // Robot heading indicator (arrow)
        double arrowLength = ROBOT_SIZE_INCHES * 0.6;
        double arrowEndX = robotX + arrowLength * Math.cos(robotHeading);
        double arrowEndY = robotY + arrowLength * Math.sin(robotHeading);
        
        packet.fieldOverlay()
            .setStroke("#1B5E20")
            .setStrokeWidth(3)
            .strokeLine(robotX, robotY, arrowEndX, arrowEndY);
        
        // ========== CAMERA SERVO VISUALIZATION ==========
        
        if (SHOW_CAMERA_SERVO) {
            // Camera servo angle indicator
            double servoAngle = Math.toRadians(robotPose.heading + cameraServo.getCurrentAngle());
            double servoLength = ROBOT_SIZE_INCHES * 0.8;
            double servoEndX = robotX + servoLength * Math.cos(servoAngle);
            double servoEndY = robotY + servoLength * Math.sin(servoAngle);
            
            packet.fieldOverlay()
                .setStroke("#E91E63")
                .setStrokeWidth(2)
                .strokeLine(robotX, robotY, servoEndX, servoEndY);
            
            // Target tag indicator
            FieldPose targetTag = (targetTagId == 20) ? blueGoal : redGoal;
            if (targetTag != null && cameraServo.getTimeSinceLastDetection() < 2000) {
                packet.fieldOverlay()
                    .setStroke("#E91E63")
                    .setStrokeWidth(1)
                    .strokeLine(robotX, robotY, targetTag.x, targetTag.y);
            }
        }
        
        // ========== VISION CORRECTIONS ==========
        
        if (SHOW_VISION_CORRECTIONS && lastVisionPose != null) {
            long timeSinceCorrection = System.currentTimeMillis() - lastVisionCorrectionTime;
            if (timeSinceCorrection < 3000) { // Show for 3 seconds
                // Vision correction indicator
                packet.fieldOverlay()
                    .setStroke("#FF5722")
                    .setStrokeWidth(2)
                    .strokeCircle(robotX, robotY, ROBOT_SIZE_INCHES);
            }
        }
        
        // ========== TELEMETRY DATA ==========
        
        // Robot pose data (original values for debugging)
        packet.put("robot_x_raw", robotPose.x);
        packet.put("robot_y_raw", robotPose.y);
        packet.put("robot_heading", robotPose.heading);
        
        // Robot pose data (clamped values used for visualization)
        packet.put("robot_x_display", robotX);
        packet.put("robot_y_display", robotY);
        
        // Dashboard debug info
        packet.put("dashboard_update_hz", DASHBOARD_UPDATE_HZ);
        packet.put("field_size", FIELD_SIZE_INCHES);
        packet.put("robot_size", ROBOT_SIZE_INCHES);
        
        // Camera servo data
        packet.put("camera_angle", cameraServo.getCurrentAngle());
        packet.put("camera_target_angle", cameraServo.getTargetAngle());
        packet.put("camera_moving", cameraServo.isMoving());
        packet.put("camera_searching", cameraServo.isSearching());
        
        // AprilTag detection data
        packet.put("target_tag_id", targetTagId);
        packet.put("last_detected_tag", cameraServo.getLastDetectedTagId());
        packet.put("detection_distance", cameraServo.getLastDetectedDistance());
        packet.put("time_since_detection", cameraServo.getTimeSinceLastDetection());
        
        // Send packet to dashboard - ensure this always happens
        if (dashboard != null) {
            dashboard.sendTelemetryPacket(packet);
        }
    }
    
    /**
     * Display telemetry information
     */
    private void displayTelemetry() {
        
        // ========== ROBOT POSITION ==========
        telemetry.addLine("=== ROBOT POSITION ===");
        telemetry.addData("X (Raw)", "%.1f inches", robotPose.x);
        telemetry.addData("Y (Raw)", "%.1f inches", robotPose.y);
        telemetry.addData("Heading", "%.1f°", robotPose.heading);
        telemetry.addLine("");
        
        // ========== DASHBOARD DEBUG ==========
        telemetry.addLine("=== DASHBOARD DEBUG ===");
        telemetry.addData("Dashboard Connected", dashboard != null ? "YES" : "NO");
        telemetry.addData("Update Rate", "%.1f Hz", DASHBOARD_UPDATE_HZ);
        telemetry.addData("Timer", "%.2f sec", dashboardTimer.seconds());
        telemetry.addData("Field Size", "%.0f inches", FIELD_SIZE_INCHES);
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
        String targetTagName = (targetTagId == 20) ? "Blue Goal (Tag 20)" : "Red Goal (Tag 24)";
        telemetry.addData("Target Tag", targetTagName);
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
        telemetry.addLine("Left Stick: Drive | Right Stick X: Turn");
        telemetry.addLine("Dpad L/R: Rotate 90° | A: Search | B: Center");
        telemetry.addLine("X: Toggle target | Y: Reset odometry");
        telemetry.addLine("");
        telemetry.addLine("🎯 Check FTC Dashboard for field view!");
        
        if (targetTagToggled) {
            telemetry.addLine("");
            telemetry.addData("🎯 Target Changed", targetTagName);
        }
        
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
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        
        // Initialize vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();
    }
}
