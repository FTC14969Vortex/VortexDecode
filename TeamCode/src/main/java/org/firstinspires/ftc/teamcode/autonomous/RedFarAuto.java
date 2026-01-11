package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.autonomous.VisionUtil.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.vision.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.autonomous.VisionUtil;
import org.firstinspires.ftc.teamcode.subsystems.Flipper;
import org.firstinspires.ftc.teamcode.subsystems.FlyWheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.utils.RobotUtil;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
@Disabled
@Autonomous(name = "Red Far Auto 4.68", group = "Autonomous")

public class RedFarAuto extends LinearOpMode {

    // |----------------------------------------|
    // |    Variable for auto mode selection    |
    // |----------------------------------------|
    AutoType autoType = AutoType.RED_FAR;
    // |----------------------------------------|


    //Variable for tracking of current stage
    VisionUtil.NearAutoStages currentNearAutoStage = VisionUtil.NearAutoStages.BACK_UP;
    VisionUtil.FarAutoStages currentFarAutoStage = VisionUtil.FarAutoStages.MOVE_TO_SHOOTING_ZONE;

    Chassis chassis;
    FlyWheel flyWheel;
    Kicker kicker;
    Intake intake;
    Flipper flipper;
    AprilTagProcessor aprilTag;

    double gateClose = 0.4;
    double gateShooting = 0.25;
    double gateIntake = 0.6;


    enum Autostages {
        Drive_To_Shooting_Zone,
        Turn_To_Shoot,

    }

    @Override
    public void runOpMode() throws InterruptedException {

        chassis = new Chassis();
        flyWheel = new FlyWheel();
        kicker = new Kicker();
        intake = new Intake();
        flipper = new Flipper();
        aprilTag = new AprilTagProcessor(this);

        chassis.init(this);
        flyWheel.init(this);
        kicker.init(hardwareMap);
        intake.init(this);
        flipper.init(hardwareMap);
        aprilTag.initCamera();

        RobotUtil.resetToDefaultSpeed();
        chassis.resetODOPosAndIMU();


        waitForStart();

        while (opModeIsActive()) {

            RobotUtil.AlignmentResult alignmentResult;
            Double robotDistanceFromAprilTag = 0.0;
            AprilTagPoseFtc aprilTagPoseFtc = null;


            if (aprilTag.findAprilTag(getAprilTagType(autoType))) {
                aprilTagPoseFtc = aprilTag.getCoordinate(getAprilTagType(autoType));
                if (aprilTagPoseFtc != null) {
                    robotDistanceFromAprilTag = aprilTagPoseFtc.range;
                    telemetry.addData("April Tag Distance", robotDistanceFromAprilTag);
                    telemetry.update();
                }
            }

            if (autoType == AutoType.RED_FAR) {
                switch (currentNearAutoStage) {
                    case BACK_UP:
                        RobotUtil.setSpeed(0.2, 0.8);
                        intake.setIntakePower(0.5);
                        chassis.drive(-42);
                        sleep(200);
                        currentNearAutoStage = NearAutoStages.SHOOT;
                        break;

                    case SHOOT:
                        // alignmentResult = RobotUtil.autoAlignWithAprilTag(this, aprilTag, AprilTagProcessor.BLUE_APRIL_TAG, chassis, telemetry);
                        chassis.turn(42);
                        chassis.drive(-22);
                        chassis.strafe(-4);
                        currentNearAutoStage = NearAutoStages.GET_MORE_BALLS;
                        break;

                    case GET_MORE_BALLS:

                        robotDistanceFromAprilTag = VisionUtil.findRobotDistanceFromAprilTag(aprilTag, autoType);

                        sleep(100);

                        RobotUtil.shoot(flyWheel, kicker, flipper, intake,robotDistanceFromAprilTag, aprilTag, VisionUtil.getAprilTagType(autoType), telemetry);

                        currentNearAutoStage = NearAutoStages.END;
                        break;

                    case END:
                        break;

                    default:
                        throw new IllegalStateException("Unexpected value: " + currentNearAutoStage.toString());
                }
            }else if(autoType == AutoType.BLUE_FAR || autoType == AutoType.RED_FAR){
                switch (currentFarAutoStage) {
                    case MOVE_TO_SHOOTING_ZONE:
                        RobotUtil.setSpeed(0.3, 0.8);
                        chassis.strafe(60);
                }
            } else{
                throw new IllegalStateException("Unexpected value: " + autoType.toString());
            }
        }
    }
}
