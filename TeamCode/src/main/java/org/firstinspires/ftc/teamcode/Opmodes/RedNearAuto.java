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

@Autonomous(name = "Red Near Auto 4.63", group = "Autonomous")

public class RedNearAuto extends LinearOpMode {

    // |----------------------------------------|
    // |    Variable for auto mode selection    |
    // |----------------------------------------|
    AutoType autoType = AutoType.RED_NEAR;
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

            if (autoType == AutoType.RED_NEAR) {
                switch (currentNearAutoStage) {
                    case BACK_UP:
                        Util.setSpeed(0.2, 0.8);
                        intake.setIntakePower(0.5);
                        chassis.drive(30);
                        sleep(200);
                        currentNearAutoStage = NearAutoStages.SHOOT;
                        break;

                    case SHOOT:
                        // alignmentResult = Util.autoAlignWithAprilTag(this, aprilTag, DecodeAprilTag.BLUE_APRIL_TAG, chassis, telemetry);
                        robotDistanceFromAprilTag = DecodeUtil.findRobotDistanceFromAprilTag(aprilTag, autoType);

                        Util.shoot(flyWheel, kicker, flipper, intake,robotDistanceFromAprilTag, aprilTag, DecodeUtil.getAprilTagType(autoType), telemetry);
                        currentNearAutoStage = NearAutoStages.GET_MORE_BALLS;
                        break;

                    case GET_MORE_BALLS:

                        chassis.turn(-135);

                        sleep(100);
                        chassis.strafe(11.5);
                        sleep(100);
                        intake.startIntake();
                        Util.setSpeed(0.2, 0.2);
                        chassis.drive(20);
                        Util.setSpeed(0.3, 0.6);
                        chassis.drive(-20);
                        sleep(100);
                        currentNearAutoStage = NearAutoStages.GO_BACK_TO_SHOOTING_ZONE;
                        break;

                    case GO_BACK_TO_SHOOTING_ZONE:
                        chassis.strafe(-12);
                        Util.prepareFlyWheelToShoot(flyWheel, kicker, intake, robotDistanceFromAprilTag, telemetry);
                        sleep(100);
                        chassis.turn(135);
                        sleep(100);
                        currentNearAutoStage = NearAutoStages.SHOOT_AGAIN;
                        break;

                    case SHOOT_AGAIN:
                        alignmentResult = Util.autoAlignWithAprilTag(this, aprilTag, DecodeUtil.getAprilTagType(autoType), chassis, telemetry);
                        Util.shoot(flyWheel, kicker, flipper, intake, alignmentResult.distance, aprilTag,DecodeUtil.getAprilTagType(autoType) , telemetry);
                        currentNearAutoStage = NearAutoStages.MOVE_OUT_OF_SHOOTING_ZONE;
                        break;

                    case MOVE_OUT_OF_SHOOTING_ZONE:
                        Util.setSpeed(0.3, 0.8);
                        chassis.strafe(-36);
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

