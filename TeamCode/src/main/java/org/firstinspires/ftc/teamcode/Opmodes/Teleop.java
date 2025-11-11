//imports
package org.firstinspires.ftc.teamcode.Opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Helper.*;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import com.qualcomm.robotcore.hardware.VoltageSensor;

//Imports LimeLight
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


@TeleOp(name = "VortexDecodeTeleopV0.9", group = "TeleOp")

public class Teleop extends LinearOpMode {

    Chassis chassis;
    DecodeAprilTag aprilTag;  // Made accessible to inner class
    VoltageSensor voltageSensor;
    private volatile boolean threadIsRunning = true;
    double flyWheelVelocity = 0.0;
    long maxLoopTimeout = 2000;
    private Thread driveThread;
    private String currentAprilTagName;  // Selected alliance AprilTag (BLUE or RED)

    WebcamName webcamName;
    private Limelight3A limelight;



    @Override
    public void runOpMode() throws InterruptedException {
        chassis = new Chassis();
        chassis.init(this);

        //voltageSensor = hardwareMap.voltageSensor.get("Motor Controller 1");
//   chassis.setDriveMode(Chassis.DriveMode.ROBOT_CENTRIC);

        FlyWheel flyWheel = new FlyWheel();
        flyWheel.init(this);

        Intake intake =new Intake();
        intake.init(this);

        Kicker kicker = new Kicker();
        kicker.init(hardwareMap);

        aprilTag = new DecodeAprilTag(this);  // Initialize class field
        aprilTag.initCamera();

        Flipper flipper = new Flipper();
        flipper.init(hardwareMap);

        chassis.odo.resetPosAndIMU();

        // Alliance selection - Prompt driver to select alliance color
        telemetry.addData("Alliance Selection", "Press X for BLUE, B for RED");
        telemetry.update();
        
        // Wait for alliance selection
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) {
                currentAprilTagName = DecodeAprilTag.BLUE_APRIL_TAG;
                telemetry.addData("Alliance Selected", "BLUE");
                telemetry.update();
                sleep(200); // Debounce
                break;
            } else if (gamepad1.b) {
                currentAprilTagName = DecodeAprilTag.RED_APRIL_TAG;
                telemetry.addData("Alliance Selected", "RED");
                telemetry.update();
                sleep(200); // Debounce
                break;
            }
        }
        
        // If no selection made, default to BLUE
        if (currentAprilTagName == null) {
            currentAprilTagName = DecodeAprilTag.BLUE_APRIL_TAG;
            telemetry.addData("Alliance", "BLUE (default)");
        }
        
        telemetry.addData("Status", "Ready - Alliance: " + (currentAprilTagName.equals(DecodeAprilTag.BLUE_APRIL_TAG) ? "BLUE" : "RED"));
        telemetry.update();

        // Define and start the drive thread
        driveThread = new Thread(new DriveTask());

        waitForStart();

        driveThread.start(); // Start the concurrent task

        //telemetry.addData("Status", "Initialized");

        //Util.prepareFlyWheelToShoot(flyWheel, kicker, intake, channelSensor, 1100, telemetry);


        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*
            kicker.setGatePosition(Kicker.GATE_SHOOT);
            flyWheel.setPower(0.45);
            telemetry.addData("FlyWheel Power - ", flyWheel.getPower());
            telemetry.addData("FlyWheel Velocity - ", flyWheel.getVelocity());
            telemetry.addData("Distance - ", "Distance - " + Util.getDistance(frontDistanceSensor, telemetry));
            //Util.telemetryFlyWheelVelocity(flyWheel,0.45, frontDistanceSensor, 30000,telemetry);
            telemetry.update();
            */

            // Calculate robot distance from AprilTag using camera
            Double robotDistanceFromAprilTagUsingCamera = 0.0;
            Double robotDistanceFromAprilTag = 45.0;  // Default distance
            Double bearing = 0.0;  // Bearing angle from camera
            Double yaw = 0.0;      // Yaw angle from camera
            boolean tagDetected = false;

            AprilTagPoseFtc aprilTagPoseFtc = null;

            if(aprilTag.findAprilTag(currentAprilTagName)){
                aprilTagPoseFtc = aprilTag.getCoordinate(currentAprilTagName);
                if(aprilTagPoseFtc !=null) {
                    robotDistanceFromAprilTagUsingCamera = aprilTagPoseFtc.range;
                    bearing = aprilTagPoseFtc.bearing;
                    yaw = aprilTagPoseFtc.yaw;
                    tagDetected = true;
                }
            }

            // Use camera distance if available and reasonable
            if(robotDistanceFromAprilTagUsingCamera != null && robotDistanceFromAprilTagUsingCamera < 180){
                robotDistanceFromAprilTag = robotDistanceFromAprilTagUsingCamera;
            }

            // Calculate required flywheel velocity based on distance
            Integer requiredFlyWheelVelocity = Util.getRequiredFlyWheelVelocity(robotDistanceFromAprilTag);

            // Display telemetry with distance, bearing, yaw, and required velocity
            telemetry.addData("AprilTag Detected", tagDetected ? "YES" : "NO");
            telemetry.addData("Distance (in)", String.format("%.1f", robotDistanceFromAprilTag));
            if (tagDetected) {
                telemetry.addData("Bearing (°)", String.format("%.1f", bearing));
                telemetry.addData("Yaw (°)", String.format("%.1f", yaw));
            }
            telemetry.addData("Required FlyWheel Velocity (RPM)", requiredFlyWheelVelocity);
            telemetry.update();

            /* debug */
            // Kicker
            if(gamepad2.dpad_up) {
                kicker.setPosition(Kicker.gateClose);
                Util.addKickerTelemetry(kicker,telemetry);
                telemetry.update();
                //flipper.resetFlipper();
            }
            if(gamepad2.dpad_down) {
                kicker.setPosition(Kicker.gateShoot);
                Util.addKickerTelemetry(kicker,telemetry);
                telemetry.update();
                //flipper.turnFlipper();
            }
            if(gamepad2.dpad_left) {
                kicker.setPosition(Kicker.gateIntake);
                Util.addKickerTelemetry(kicker,telemetry);
                telemetry.update();
            }
            if(gamepad2.dpad_right) {
                if (kicker.getGatePosition().equals(Kicker.GATE_CLOSE)) {
                    kicker.setPosition(Kicker.gateShoot);
                    sleep(400);
                    kicker.setPosition(Kicker.gateClose);
                }
                Util.addKickerTelemetry(kicker, telemetry);
                telemetry.update();
            }

            /* debug */

            // Shooting - Execute complete 4-shot sequence when right bumper is pressed
            // Distance is updated from AprilTag before each shot for accuracy
            if (gamepad2.right_bumper) {
                Util.shoot(flyWheel, kicker, flipper, intake, robotDistanceFromAprilTag, aprilTag, currentAprilTagName, telemetry);
            }

            if (gamepad2.a){
                Util.prepareFlyWheelToShoot(flyWheel,kicker, intake, robotDistanceFromAprilTag, telemetry);
            }

            if (gamepad2.b) {
                Util.prepareFlyWheelToIntake(flyWheel,kicker,intake,flipper, telemetry);
            }

            if (gamepad2.x){
                intake.startIntake();
            }

            if (gamepad2.y) {
                intake.stopIntake();
            }

        }

        // Clean up the thread
        threadIsRunning = false;
        sleep(2000);
        driveThread.interrupt();
        try {
            driveThread.join();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        }

    private class DriveTask implements Runnable {

        @Override
        public void run() {
            while ( threadIsRunning && !Thread.currentThread().isInterrupted()) {

                // Check for alignment trigger (gamepad1 right bumper)
                if (Teleop.this.gamepad1.right_bumper) {
                    // Perform automatic alignment with AprilTag (using selected alliance)
                    Util.AlignmentResult result = Util.autoAlignWithAprilTag(
                        Teleop.this, Teleop.this.aprilTag, Teleop.this.currentAprilTagName, 
                        Teleop.this.chassis, Teleop.this.telemetry);
                    
                    if (result.success) {
                        Teleop.this.telemetry.addData("Alignment", "SUCCESS - Distance: %.1f inches", result.distance);
                    } else {
                        Teleop.this.telemetry.addData("Alignment", "FAILED");
                    }
                    Teleop.this.telemetry.update();
                    
                    // Brief pause to prevent multiple triggers
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                        return;
                    }
                }

                // Read gamepad input and set drive motor power
                float axial = -Teleop.this.gamepad1.left_stick_y;
                float lateral = -Teleop.this.gamepad1.left_stick_x;
                float yaw = Teleop.this.gamepad1.right_stick_x; // Note: positive yaw is clockwise, previously was negative
                Teleop.this.chassis.moveRobot(axial, lateral, yaw);

                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
        }
    }
}
