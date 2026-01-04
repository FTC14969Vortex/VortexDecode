package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.utils.RobotUtil;
import org.firstinspires.ftc.teamcode.vision.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@TeleOp(name = "Teleop Blue Iterative 0.01", group = "TeleOp")
public class TeleopBlueIterative extends OpMode {

    // Subsystems
    private Chassis chassis;
    private FlyWheel flyWheel;
    private Intake intake;
    private Kicker kicker;
    private Flipper flipper;
    private AprilTagProcessor aprilTag;

    private final String currentAprilTagName = AprilTagProcessor.BLUE_APRIL_TAG;
    private double robotDistanceFromAprilTag = 45.0;

    @Override
    public void init() {
        // Initialize all hardware
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

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // 1. VISION & SENSING
        updateVisionData();

        // 2. CHASSIS CONTROL (The old DriveTask logic)
        handleDriving();

        // 3. SHOOTING & SUBSYSTEM CONTROL
        handleSubsystems();

        // 4. TELEMETRY
        updateRobotTelemetry();
    }

    private void updateVisionData() {
        if (aprilTag.findAprilTag(currentAprilTagName)) {
            AprilTagPoseFtc pose = aprilTag.getCoordinate(currentAprilTagName);
            if (pose != null && pose.range < 180) {
                robotDistanceFromAprilTag = pose.range;
            }
        }
    }

    private void handleDriving() {
        // Auto-align check (Non-blocking)
        if (gamepad1.right_bumper) {
            // NOTE: autoAlignWithAprilTag needs to be written to be non-blocking
            // if it contains loops/sleeps, it will stutter the robot.
            //RobotUtil.autoAlignWithAprilTag(this, aprilTag, currentAprilTagName, chassis, telemetry);
        } else {
            // Normal Drive
            float axial = -gamepad1.left_stick_y;
            float lateral = -gamepad1.left_stick_x;
            float yaw = -gamepad1.right_stick_x;

            RobotUtil.setMotorPower(chassis.frontLeftDrive, chassis.backLeftDrive,
                    chassis.frontRightDrive, chassis.backRightDrive,
                    axial, lateral, yaw);
        }
    }

    private void handleSubsystems() {
        // Shooting logic
        if (gamepad2.right_bumper) {
            // This call should ideally be a state machine if it takes time
            RobotUtil.shoot(flyWheel, kicker, flipper, intake, robotDistanceFromAprilTag, aprilTag, currentAprilTagName, telemetry);
        }

        if (gamepad2.a) {
            RobotUtil.prepareFlyWheelToShoot(flyWheel, kicker, intake, robotDistanceFromAprilTag, telemetry);
        }

        if (gamepad2.b) {
            RobotUtil.prepareFlyWheelToIntake(flyWheel, kicker, intake, flipper, telemetry);
        }

        // Intake control
        if (gamepad2.x) intake.startIntake();
        if (gamepad2.y) intake.stopIntake();
    }

    private void updateRobotTelemetry() {
        telemetry.addData("Distance", "%.1f", robotDistanceFromAprilTag);
        telemetry.addData("Required RPM", RobotUtil.getRequiredFlyWheelVelocity(robotDistanceFromAprilTag));
        // No need for telemetry.update() in Iterative OpMode; it's called automatically.
    }
}