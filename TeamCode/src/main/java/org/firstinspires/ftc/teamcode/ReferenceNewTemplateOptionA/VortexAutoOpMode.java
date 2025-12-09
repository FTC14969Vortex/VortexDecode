package org.firstinspires.ftc.teamcode.ReferenceNewTemplateOptionA;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Helper.Chassis;
import org.firstinspires.ftc.teamcode.Helper.DecodeAprilTag;
import org.firstinspires.ftc.teamcode.Helper.Flipper;
import org.firstinspires.ftc.teamcode.Helper.FlyWheel;
import org.firstinspires.ftc.teamcode.Helper.Intake;
import org.firstinspires.ftc.teamcode.Helper.Kicker;

public class VortexAutoOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // --- create hardware-level objects ---
        Chassis chassis = new Chassis();
        chassis.init(this);

        Intake intake = new Intake();
        FlyWheel flyWheel = new FlyWheel();
        Flipper flipper = new Flipper();
        Kicker kicker = new Kicker();
        DecodeAprilTag aprilTag = new DecodeAprilTag(this);

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