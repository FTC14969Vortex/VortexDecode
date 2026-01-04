package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.FlyWheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Flipper;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
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
@TeleOp(name = "FlyWheel Velocity Test", group = "Debug")
public class FlyWheelVelocityTest extends OpMode {
    
    // Subsystems
    private FlyWheel flyWheel;
    private Intake intake;
    private Flipper flipper;
    private Kicker kicker;
    private CameraServo cameraServo;
    
    // Vision system
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    

    
    // Test parameters
    private double targetVelocity = 1200.0; // Starting velocity (RPM)
    private final double VELOCITY_STEP = 50.0; // Velocity adjustment step
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
        cameraServo.init(hardwareMap, aprilTagProcessor);
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
        
        telemetry.addLine("✅ Systems Ready!");
        telemetry.addLine("");
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("Dpad Up/Down: Adjust velocity (±50 RPM)");
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
        cameraServo.update();
        
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
     * Continuous shooting test based on RobotUtil.shoot() pattern
     * Sequence: close gate → set flywheel velocity → open gate → 3 flipper shots → cleanup
     */
    private void testShoot() {
        new Thread(() -> {
            try {
                // Configuration constants (from RobotUtil.shoot)
                final int NUM_SHOTS = 3;
                final double INITIAL_FLIPPER_ANGLE = 120;   // Starting flipper angle in degrees
                final double ANGLE_INCREMENT = 30;          // Angle increase per shot (degrees)
                final int KICKER_OPEN_DELAY_MS = 300;       // Wait time for kicker to open
                final int BASE_FLIPPER_DELAY_MS = 150;      // Base wait time for flipper movement
                final int FLIPPER_DELAY_INCREMENT_MS = 50;  // Additional delay per shot
                final int FLIPPER_RESET_DELAY_MS = 200;     // Wait time for flipper to reset
                
                telemetry.addLine("🚀 Starting continuous shooting test...");
                telemetry.update();
                
                // Step 1: Close gate and reduce intake power
                kicker.setGatePosition(Kicker.GATE_CLOSE);
                intake.setIntakePower(0.5); // Reduced power for shooting
                Thread.sleep(200);
                
                // Step 2: Set flywheel to target velocity
                FlyWheel.FlyWheelSpinUpResult result = flyWheel.setToShootingVelocity(targetVelocity, 3000);
                if (!result.success) {
                    telemetry.addLine("⚠️ Flywheel failed to reach target velocity");
                    telemetry.update();
                }
                
                // Step 3: Open gate for shooting
                kicker.setGatePosition(Kicker.GATE_SHOOT);
                Thread.sleep(KICKER_OPEN_DELAY_MS);
                
                // Step 4: Execute 3 flipper shots
                for (int shotNumber = 0; shotNumber < NUM_SHOTS; shotNumber++) {
                    // Calculate flipper angle for this shot (120°, 150°, 180°)
                    double currentFlipperAngle = INITIAL_FLIPPER_ANGLE + (shotNumber * ANGLE_INCREMENT);
                    
                    telemetry.addData("Shot", "%d/3 at %.0f degrees", shotNumber + 1, currentFlipperAngle);
                    telemetry.update();
                    
                    // Ramp up flywheel before each shot to maintain velocity
                    flyWheel.setToShootingVelocity(targetVelocity, 3000);
                    
                    // Turn flipper to calculated angle
                    flipper.turnFlipper(currentFlipperAngle);
                    
                    // Wait for flipper movement (time increases with shot number)
                    int flipperWaitTime = BASE_FLIPPER_DELAY_MS + (shotNumber * FLIPPER_DELAY_INCREMENT_MS);
                    Thread.sleep(flipperWaitTime);
                    
                    // Reset flipper to starting position
                    flipper.resetFlipper();
                    Thread.sleep(FLIPPER_RESET_DELAY_MS);
                }
                
                // Step 5: Cleanup - return to intake mode
                telemetry.addLine("🧹 Cleaning up...");
                telemetry.update();
                
                // Reset flipper
                flipper.resetFlipper();
                
                // Close gate
                kicker.setGatePosition(Kicker.GATE_CLOSE);
                
                // Stop flywheel with fast stop (active braking)
                FlyWheel.FlyWheelSpinUpResult stopResult = flyWheel.fastStop();
                if (!stopResult.success) {
                    telemetry.addLine("⚠️ Fast stop timeout");
                    telemetry.update();
                }
                
                // Set gate to intake position and start intake
                kicker.setGatePosition(Kicker.GATE_INTAKE);
                intake.startIntake();
                
                telemetry.addLine("✅ Shooting test completed!");
                telemetry.update();
                
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                telemetry.addLine("❌ Shooting test interrupted");
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
        
        if (timeSinceDetection < 1000) { // Less than 1 second ago
            telemetry.addLine("🎯 APRILTAG DETECTED");
            telemetry.addData("Tag ID", tagId);
            telemetry.addData("Distance", "%.1f inches", distance);
            telemetry.addData("Robot Misalignment", "%.1f° %s", Math.abs(misalignmentAngle), 
                misalignmentAngle > 0 ? "(turn RIGHT)" : misalignmentAngle < 0 ? "(turn LEFT)" : "(ALIGNED)");
        } else if (timeSinceDetection < 3000) { // Lost recently
            telemetry.addData("Tag Status", "Lost %.1fs ago", timeSinceDetection / 1000.0);
            telemetry.addData("Last Tag ID", tagId);
            telemetry.addData("Last Distance", "%.1f inches", distance);
            telemetry.addData("Last Misalignment", "%.1f° %s", Math.abs(misalignmentAngle), 
                misalignmentAngle > 0 ? "(turn RIGHT)" : misalignmentAngle < 0 ? "(turn LEFT)" : "(ALIGNED)");
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
