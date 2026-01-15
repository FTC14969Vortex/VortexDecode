package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.subsystems.FlyWheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Flipper;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.utils.RobotOperations;
import org.firstinspires.ftc.teamcode.vision.CameraServo;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


/**
 * FlyWheel Velocity Calibration Test
 * 
 * This test allows manual calibration of flywheel velocity vs distance relationship.
 * 
 * SETUP:
 * 1. Position robot manually at different distances from blue AprilTag (ID 20)
 * 2. Use camera servo to aim at the tag (static positioning)
 * 3. Adjust flywheel velocity using gamepad controls
 * 4. Test shoot and record distance-velocity pairs
 * 
 * CONTROLS:
 * - Dpad Up: Increase flywheel velocity (+50 RPM)
 * - Dpad Down: Decrease flywheel velocity (-50 RPM)
 * - Right Bumper: Continuous shooting test (close gate → set flywheel → shoot → 3 flipper shots → cleanup)
 * 
 * TELEMETRY:
 * - Distance to blue AprilTag
 * - Angle to blue AprilTag  
 * - Current flywheel velocity (target and actual)
 * - Flywheel status
 * - Manual data recording prompts
 */
//@Disabled
@TeleOp(name = "FlyWheel Velocity Test", group = "Debug")
public class FlyWheelVelocityTest extends OpMode {
    
    // Subsystems
    private FlyWheel flyWheel;
    private Intake intake;
    private Flipper flipper;
    private Kicker kicker;
    private CameraServo cameraServo;
    private RobotOperations robotOperations;
    
    // Vision system
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    

    
    // Test parameters
    private double targetVelocity = 1200.0; // Starting velocity (RPM)
    private final double VELOCITY_STEP = 20.0; // Velocity adjustment step
    private final double MIN_VELOCITY = 800.0; // Minimum velocity
    private final double MAX_VELOCITY = 3000.0;
    
    // Control state tracking
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;
    private boolean rightBumperPressed = false;
    
    @Override
    public void init() {
        
        telemetry.addLine("Initializing FlyWheel Velocity Test...");
        telemetry.update();
        
        // Initialize vision system
        initVisionSystem();
        
        // Initialize camera servo system
        cameraServo = new CameraServo();
        cameraServo.init(hardwareMap);
        cameraServo.setTargetTag(20); // Blue AprilTag
        cameraServo.moveToCenter(); // Keep servo at center position for manual robot alignment
        
        // Initialize flywheel subsystem
        flyWheel = new FlyWheel();
        flyWheel.init(this);
        
        // Initialize intake subsystem for shooting tests
        intake = new Intake();
        intake.init(this);
        intake.stopIntake(); // Start with intake off
        
        // Initialize flipper subsystem for shooting tests
        flipper = new Flipper();
        flipper.init(hardwareMap);
        flipper.resetFlipper();
        
        kicker = new Kicker();
        kicker.init(hardwareMap);
        kicker.setGatePosition(Kicker.GATE_INTAKE); // Start in intake position
        
        // Initialize RobotOperations utility
        robotOperations = new RobotOperations();
        robotOperations.init(null, flyWheel, intake, kicker, flipper, cameraServo, null, this);
        
        telemetry.addLine("✅ Systems Ready!");
        telemetry.addLine("");
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("Dpad Up/Down: Adjust velocity (±20 RPM)");
        telemetry.addLine("A: Start flywheel | Y: Fast stop");
        telemetry.addLine("X: Test shoot (3 shots with intake + flipper)");
        telemetry.addLine("");
        telemetry.addLine("Position robot at different distances");
        telemetry.addLine("and record velocity-distance pairs!");
        telemetry.update();
    }
    
    @Override
    public void start() {
        // Camera servo handles vision automatically
        // No additional setup needed
    }
    
    @Override
    public void loop() {
        
        // Update camera servo (handles AprilTag detection)
       // cameraServo.update();
        
        // Handle gamepad controls
        handleControls();
        
        // Display comprehensive telemetry
        displayTelemetry();
    }
    
    @Override
    public void stop() {
        // Clean shutdown
        if (flyWheel != null) {
            flyWheel.stop();
        }
        // Cleanup vision portal
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
    

    
    /**
     * Handle gamepad controls for flywheel velocity adjustment
     */
    private void handleControls() {
        
        // Velocity adjustment controls
        if (gamepad1.dpad_up && !dpadUpPressed) {
            targetVelocity = Math.min(targetVelocity + VELOCITY_STEP, MAX_VELOCITY);
            dpadUpPressed = true;
        } else if (!gamepad1.dpad_up) {
            dpadUpPressed = false;
        }
        
        if (gamepad1.dpad_down && !dpadDownPressed) {
            targetVelocity = Math.max(targetVelocity - VELOCITY_STEP, MIN_VELOCITY);
            dpadDownPressed = true;
        } else if (!gamepad1.dpad_down) {
            dpadDownPressed = false;
        }
        
        // Continuous shooting test
        if (gamepad1.right_bumper && !rightBumperPressed) {
            testShoot();
            rightBumperPressed = true;
        } else if (!gamepad1.right_bumper) {
            rightBumperPressed = false;
        }
    }
    
    /**
     * Simplified shooting test using RobotOperations utility
     * Uses the new overloaded shoot method with custom velocity and no alignment
     */
    private void testShoot() {
        new Thread(() -> {
            try {
                telemetry.addLine("🚀 Starting shooting test with RobotOperations...");
                telemetry.addData("Target Velocity", "%.0f RPM", targetVelocity);
                telemetry.update();

                intake.setIntakePower(1.0); // Turn on intake for shooting

                // Use RobotOperations.shoot with custom velocity and no alignment
                kicker.setGatePosition(kicker.GATE_CLOSE);
                Thread.sleep(200); // Wait for gate to close

                // do alignment, and use cameraservo for alignment
                boolean alignment = true;
                boolean useCameraServo = true;
                robotOperations.shoot(targetVelocity, alignment, useCameraServo);
                
                telemetry.addLine("✅ Shooting test completed!");
                telemetry.update();
                
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                telemetry.addLine("❌ Shooting test interrupted");
                telemetry.update();
            } catch (Exception e) {
                telemetry.addLine("❌ Shooting test error: " + e.getMessage());
                telemetry.update();
            }
        }).start();
    }
    
    /**
     * Display comprehensive telemetry for data collection
     */
    private void displayTelemetry() {
        
        telemetry.addLine("=== FLYWHEEL VELOCITY TEST ===");
        telemetry.addLine("");
        
        // AprilTag detection status from CameraServo
        long timeSinceDetection = cameraServo.getTimeSinceLastDetection();
        double distance = cameraServo.getLastDetectedDistance();
        double misalignmentAngle = cameraServo.getCurrentAngle();  // Servo angle = robot misalignment angle
        int tagId = cameraServo.getLastDetectedTagId();
        
        // Additional diagnostic data
        double bearing = cameraServo.getLastDetectedBearing();
        double yaw = cameraServo.getLastDetectedYaw();
        double elevation = cameraServo.getLastDetectedElevation();
        
        if (timeSinceDetection < 1000) { // Less than 1 second ago
            telemetry.addLine("🎯 APRILTAG DETECTED");
            telemetry.addData("Tag ID", tagId);
            telemetry.addData("Distance", "%.1f inches", distance);
            telemetry.addData("Robot Misalignment", "%.1f° %s", Math.abs(misalignmentAngle), 
                misalignmentAngle > 0 ? "(turn RIGHT)" : misalignmentAngle < 0 ? "(turn LEFT)" : "(ALIGNED)");
            telemetry.addData("Camera Bearing", "%.1f°", bearing);
            telemetry.addData("Tag Yaw", "%.1f°", yaw);
            telemetry.addData("Camera Elevation", "%.1f°", elevation);
        } else if (timeSinceDetection < 3000) { // Lost recently
            telemetry.addData("Tag Status", "Lost %.1fs ago", timeSinceDetection / 1000.0);
            telemetry.addData("Last Tag ID", tagId);
            telemetry.addData("Last Distance", "%.1f inches", distance);
            telemetry.addData("Last Misalignment", "%.1f° %s", Math.abs(misalignmentAngle), 
                misalignmentAngle > 0 ? "(turn RIGHT)" : misalignmentAngle < 0 ? "(turn LEFT)" : "(ALIGNED)");
            telemetry.addData("Last Camera Bearing", "%.1f°", bearing);
            telemetry.addData("Last Tag Yaw", "%.1f°", yaw);
            telemetry.addData("Last Camera Elevation", "%.1f°", elevation);
        } else {
            telemetry.addLine("❌ NO APRILTAG DETECTED");
            telemetry.addLine("Position robot to see AprilTag!");
        }
        
        telemetry.addLine("");
        
        // Flywheel status
        telemetry.addLine("🚁 FLYWHEEL STATUS");
        telemetry.addData("Target Velocity", "%.0f RPM", targetVelocity);
        telemetry.addData("Actual Velocity", "%.0f RPM", flyWheel.getVelocity());
        telemetry.addData("Current Power", "%.2f", flyWheel.getPower());
        
        telemetry.addLine("");
        
        // Data collection prompt
        if (timeSinceDetection < 1000) {
            telemetry.addLine("📊 READY FOR TESTING:");
            telemetry.addData("Distance", "%.1f inches", distance);
            telemetry.addData("Misalignment", "%.1f°", misalignmentAngle);
            telemetry.addData("Target Velocity", "%.0f RPM", targetVelocity);
            telemetry.addData("Bearing", "%.1f°", bearing);
            telemetry.addData("Yaw", "%.1f°", yaw);
            telemetry.addData("Elevation", "%.1f°", elevation);
            telemetry.addLine("Press Right Bumper to test shoot!");
        } else {
            telemetry.addLine("💡 Position robot to see AprilTag");
        }
        
        telemetry.addLine("");
        
        // Control reminders
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("Dpad ↑↓: ±50 RPM");
        telemetry.addLine("Right Bumper: Continuous Shooting Test");
        
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
