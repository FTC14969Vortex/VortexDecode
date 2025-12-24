package org.firstinspires.ftc.teamcode.Opmodes;

import static org.firstinspires.ftc.teamcode.helper.DecodeUtil.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.helper.Chassis;
import org.firstinspires.ftc.teamcode.helper.DecodeAprilTag;
import org.firstinspires.ftc.teamcode.helper.DecodeUtil;
import org.firstinspires.ftc.teamcode.helper.Flipper;
import org.firstinspires.ftc.teamcode.helper.FlyWheel;
import org.firstinspires.ftc.teamcode.helper.Intake;
import org.firstinspires.ftc.teamcode.helper.Kicker;
import org.firstinspires.ftc.teamcode.helper.Util;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
@Autonomous(name = "Red Far Auto 4.68", group = "Autonomous")

public class RedFarAuto extends LinearOpMode {

    // |----------------------------------------|
    // |    Variable for auto mode selection    |
    // |----------------------------------------|
    AutoType autoType = AutoType.RED_FAR;
    // |----------------------------------------|


    //Variable for tracking of current stage
    DecodeUtil.NearAutoStages currentNearAutoStage = DecodeUtil.NearAutoStages.BACK_UP;
    DecodeUtil.FarAutoStages currentFarAutoStage = DecodeUtil.FarAutoStages.MOVE_TO_SHOOTING_ZONE;

    Chassis chassis;
    FlyWheel flyWheel;
    Kicker kicker;
    Intake intake;
    Flipper flipper;
    DecodeAprilTag aprilTag;

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
        aprilTag = new DecodeAprilTag(this);

        chassis.init(this);
        flyWheel.init(this);
        kicker.init(hardwareMap);
        intake.init(this);
        flipper.init(hardwareMap);
        aprilTag.initCamera();

        Util.resetToDefaultSpeed();
        chassis.resetODOPosAndIMU();


        waitForStart();

        while (opModeIsActive()) {

            Util.AlignmentResult alignmentResult;
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
                        Util.setSpeed(0.2, 0.8);
                        intake.setIntakePower(0.5);
                        chassis.drive(-42);
                        sleep(200);
                        currentNearAutoStage = NearAutoStages.SHOOT;
                        break;

                    case SHOOT:
                        // alignmentResult = Util.autoAlignWithAprilTag(this, aprilTag, DecodeAprilTag.BLUE_APRIL_TAG, chassis, telemetry);
                        chassis.turn(42);
                        chassis.drive(-22);
                        chassis.strafe(-4);
                        currentNearAutoStage = NearAutoStages.GET_MORE_BALLS;
                        break;

                    case GET_MORE_BALLS:

                        robotDistanceFromAprilTag = DecodeUtil.findRobotDistanceFromAprilTag(aprilTag, autoType);

                        sleep(100);

                        Util.shoot(flyWheel, kicker, flipper, intake,robotDistanceFromAprilTag, aprilTag, DecodeUtil.getAprilTagType(autoType), telemetry);

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
                        Util.setSpeed(0.3, 0.8);
                        chassis.strafe(60);
                }
            } else{
                throw new IllegalStateException("Unexpected value: " + autoType.toString());
            }
        }
    }
}

