package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.autonomous.VisionUtil.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name = "Red Near Auto 4.63", group = "Autonomous")

public class RedNearAuto extends LinearOpMode {

    // |----------------------------------------|
    // |    Variable for auto mode selection    |
    // |----------------------------------------|
    AutoType autoType = AutoType.RED_NEAR;
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

            if (autoType == AutoType.RED_NEAR) {
                switch (currentNearAutoStage) {
                    case BACK_UP:
                        RobotUtil.setSpeed(0.2, 0.8);
                        intake.setIntakePower(0.5);
                        chassis.drive(30);
                        sleep(200);
                        currentNearAutoStage = NearAutoStages.SHOOT;
                        break;

                    case SHOOT:
                        // alignmentResult = RobotUtil.autoAlignWithAprilTag(this, aprilTag, AprilTagProcessor.BLUE_APRIL_TAG, chassis, telemetry);
                        robotDistanceFromAprilTag = VisionUtil.findRobotDistanceFromAprilTag(aprilTag, autoType);

                        RobotUtil.shoot(flyWheel, kicker, flipper, intake,robotDistanceFromAprilTag, aprilTag, VisionUtil.getAprilTagType(autoType), telemetry);
                        currentNearAutoStage = NearAutoStages.GET_MORE_BALLS;
                        break;

                    case GET_MORE_BALLS:

                        chassis.turn(-135);

                        sleep(100);
                        chassis.strafe(11.5);
                        sleep(100);
                        intake.startIntake();
                        RobotUtil.setSpeed(0.2, 0.2);
                        chassis.drive(20);
                        RobotUtil.setSpeed(0.3, 0.6);
                        chassis.drive(-20);
                        sleep(100);
                        currentNearAutoStage = NearAutoStages.GO_BACK_TO_SHOOTING_ZONE;
                        break;

                    case GO_BACK_TO_SHOOTING_ZONE:
                        chassis.strafe(-12);
                        RobotUtil.prepareFlyWheelToShoot(flyWheel, kicker, intake, robotDistanceFromAprilTag, telemetry);
                        sleep(100);
                        chassis.turn(135);
                        sleep(100);
                        currentNearAutoStage = NearAutoStages.SHOOT_AGAIN;
                        break;

                    case SHOOT_AGAIN:
                        alignmentResult = RobotUtil.autoAlignWithAprilTag(this, aprilTag, VisionUtil.getAprilTagType(autoType), chassis, telemetry);
                        RobotUtil.shoot(flyWheel, kicker, flipper, intake, alignmentResult.distance, aprilTag,VisionUtil.getAprilTagType(autoType) , telemetry);
                        currentNearAutoStage = NearAutoStages.MOVE_OUT_OF_SHOOTING_ZONE;
                        break;

                    case MOVE_OUT_OF_SHOOTING_ZONE:
                        RobotUtil.setSpeed(0.3, 0.8);
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
                        RobotUtil.setSpeed(0.3, 0.8);
                        chassis.strafe(60);
                }
            } else{
                throw new IllegalStateException("Unexpected value: " + autoType.toString());
            }
        }
    }
}
