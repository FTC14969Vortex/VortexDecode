package org.firstinspires.ftc.teamcode.ReferenceNewTemplateOptionB;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
        IntakeManager intakeManager = new IntakeManager(intake, telemetry);
        ShootManager shootManager  = new ShootManager(flyWheel,kicker, flipper, intake, telemetry);
        DriveManager driveManager = new DriveManager(chassis, telemetry);

        Pose2D[] BALL_POS = {/* fill from field measurements */};
        Pose2D[] SHOOT_POS = {/* same length or mapping */};
        Pose2D PARK_POS = /* your park pose */ null;

        //TODO: need to review these parameters
        GameManager gameManager = new GameManager(driveManager,intakeManager,shootManager, telemetry, BALL_POS, SHOOT_POS, PARK_POS,
                30,5,3,3,3,3);

        waitForStart();

        while (opModeIsActive() && !gameManager.isDone()) {
            double nowSec = getRuntime();

            gameManager.update(nowSec);   // global phase FSM
            driveManager.update(nowSec);     // local FSM: drive
            intakeManager.update(nowSec);    // local FSM: intake
            shootManager.update(nowSec);   // local FSM: shooter

            telemetry.update();
        }
    }
}