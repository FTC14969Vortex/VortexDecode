//imports
package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.Flipper;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.FlyWheel;
import org.firstinspires.ftc.teamcode.vision.LimeLightProcessor;
import org.firstinspires.ftc.teamcode.vision.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.motion.MotionConfig;
import org.firstinspires.ftc.teamcode.motion.MotionExecutor;

import org.firstinspires.ftc.teamcode.utils.RobotUtil;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import com.qualcomm.robotcore.hardware.VoltageSensor;

//Imports LimeLight
import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp(name = "Teleop Blue 0.01", group = "TeleOp")

public class TeleopBlue extends LinearOpMode {

    // Subsystems
    private Chassis chassis;
    private FlyWheel flyWheel;
    private Intake intake;
    private Kicker kicker;
    private Flipper flipper;

    // Vision
    private WebcamName webcamName;
    private Limelight3A limelight;
    private AprilTagProcessor aprilTag;
    private final String currentAprilTagName = AprilTagProcessor.BLUE_APRIL_TAG;

    // Threading
    private Thread driveThread;
    private volatile boolean threadIsRunning = true;


    @Override
    public void runOpMode() throws InterruptedException {

        initializeSubsystems();

        // Setup drive control thread
        driveThread = new Thread(new DriveTask());

        telemetry.addData("Status", "Initialized - Waiting for Start");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            driveThread.start(); // Start the concurrent task
        }


        //RobotUtil.prepareFlyWheelToShoot(flyWheel, kicker, intake, channelSensor, 1100, telemetry);


        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

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
            Integer requiredFlyWheelVelocity = RobotUtil.getRequiredFlyWheelVelocity(robotDistanceFromAprilTag);

            // Display telemetry with distance, bearing, yaw, and required velocity
            telemetry.addData("AprilTag Detected", tagDetected ? "YES" : "NO");
            telemetry.addData("Distance (in)", String.format("%.1f", robotDistanceFromAprilTag));
            if (tagDetected) {
                telemetry.addData("Bearing ( degrees)", String.format("%.1f", bearing));
                telemetry.addData("Yaw ( degrees)", String.format("%.1f", yaw));
            }
            telemetry.addData("Required FlyWheel Velocity (RPM)", requiredFlyWheelVelocity);
            telemetry.update();

            /*
            // Kicker
            if(gamepad2.dpad_up) {
                kicker.setPosition(Kicker.gateClose);
                RobotUtil.addKickerTelemetry(kicker,telemetry);
                telemetry.update();
                //flipper.resetFlipper();
            }
            if(gamepad2.dpad_down) {
                kicker.setPosition(Kicker.gateShoot);
                RobotUtil.addKickerTelemetry(kicker,telemetry);
                telemetry.update();
                //flipper.turnFlipper();
            }
            if(gamepad2.dpad_left) {
                kicker.setPosition(Kicker.gateIntake);
                RobotUtil.addKickerTelemetry(kicker,telemetry);
                telemetry.update();
            }
            if(gamepad2.dpad_right) {
                if (kicker.getGatePosition().equals(Kicker.GATE_CLOSE)) {
                    kicker.setPosition(Kicker.gateShoot);
                    sleep(400);
                    kicker.setPosition(Kicker.gateClose);
                }
                RobotUtil.addKickerTelemetry(kicker, telemetry);
                telemetry.update();
            }

             */

            // Shooting - Execute complete 4-shot sequence when right bumper is pressed
            // Distance is updated from AprilTag before each shot for accuracy
            if (gamepad2.right_bumper) {
                RobotUtil.shoot(flyWheel, kicker, flipper, intake, robotDistanceFromAprilTag, aprilTag, currentAprilTagName, telemetry);
            }

            if (gamepad2.a){
                RobotUtil.prepareFlyWheelToShoot(flyWheel,kicker, intake, robotDistanceFromAprilTag, telemetry);
                RobotUtil.FlyWheelSpinUpResult spinUpResult = RobotUtil.prepareForShooting(flyWheel, kicker, flipper, intake, 40.0, telemetry);
            }

            if (gamepad2.b) {
                RobotUtil.prepareFlyWheelToIntake(flyWheel,kicker,intake,flipper, telemetry);
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
                if (TeleopBlue.this.gamepad1.right_bumper) {
                    // Perform automatic alignment with AprilTag (using selected alliance)
                    RobotUtil.AlignmentResult result = RobotUtil.autoAlignWithAprilTag(
                            TeleopBlue.this, TeleopBlue.this.aprilTag, TeleopBlue.this.currentAprilTagName,
                            TeleopBlue.this.chassis, TeleopBlue.this.telemetry);

                    if (result.success) {
                        TeleopBlue.this.telemetry.addData("Alignment", "SUCCESS - Distance: %.1f inches", result.distance);
                    } else {
                        TeleopBlue.this.telemetry.addData("Alignment", "FAILED");
                    }
                    TeleopBlue.this.telemetry.update();

                    // Brief pause to prevent multiple triggers
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                        return;
                    }
                }

                // Read gamepad input and set drive motor power
                float axial = -TeleopBlue.this.gamepad1.left_stick_y;
                float lateral = -TeleopBlue.this.gamepad1.left_stick_x;
                float yaw = -TeleopBlue.this.gamepad1.right_stick_x; // Note: positive yaw is clockwise, previously was negative
                RobotUtil.setMotorPower(chassis.frontLeftDrive, chassis.backLeftDrive,
                        chassis.frontRightDrive, chassis.backRightDrive,
                        axial, lateral, yaw);

                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
        }
    }

    private void initializeSubsystems() {
        chassis = new Chassis();
        chassis.init(this);
        chassis.odo.resetPosAndIMU();

        flyWheel = new FlyWheel();
        flyWheel.init(this);

        intake = new Intake();
        intake.init(this);

        kicker = new Kicker();
        kicker.init(hardwareMap);

        flipper = new Flipper();
        flipper.init(hardwareMap);

        aprilTag = new AprilTagProcessor(this);
        aprilTag.initCamera();

        aprilTag  = new AprilTagProcessor(this);
        aprilTag.initCamera();
    }

}
