package org.firstinspires.ftc.teamcode.ReferenceNewTemplateOptionA;
public class VortexAutoOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // --- create hardware-level objects ---
        Chassis chassis = new Chassis();
        chassis.init(this);

        Intake intake = new Intake(hardwareMap);
        FlyWheel flyWheel = new FlyWheel(hardwareMap);
        Flipper flipper = new Flipper(hardwareMap);
        Kicker kicker = new Kicker(hardwareMap);
        DecodeAprilTag aprilTag = new DecodeAprilTag(hardwareMap, telemetry);

        // --- build high-level managers ---
        IntakeManager intakeManager = new IntakeManager(chassis, intake, telemetry);
        ShootManager  shootManager  = new ShootManager(chassis, flyWheel, flipper, kicker, intake, aprilTag, telemetry);

        Pose2D[] BALL_POS = {/* fill from field measurements */};
        Pose2D[] SHOOT_POS = {/* same length or mapping */};
        Pose2D PARK_POS = /* your park pose */ null;

        GameManager gameManager = new GameManager(BALL_POS, SHOOT_POS, PARK_POS,
                intakeManager, shootManager,
                telemetry);

        waitForStart();

        while (opModeIsActive() && !gameManager.isDone()) {
            gameManager.update();   // this calls intakeManager.update and shootManager.update internally
            telemetry.update();
        }
    }
}
