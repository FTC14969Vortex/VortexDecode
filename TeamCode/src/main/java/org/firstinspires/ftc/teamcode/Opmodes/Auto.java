package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.helper.Chassis;
import org.firstinspires.ftc.teamcode.helper.FlyWheel;
import org.firstinspires.ftc.teamcode.helper.Gate;
import org.firstinspires.ftc.teamcode.helper.DecodeAprilTag;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Alaqmar Auto 2.2;")
public class Auto extends LinearOpMode {

    Chassis chassis = null;
    double leftFrontPower;
    double leftBackPower;
    double rightFrontPower;
    double rightBackPower;


    // Reference to GoBilda's Pinpoint odometry driver


    // Drive motor references
    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;
    private IMU imu;

    FlyWheel flyWheel = new FlyWheel();
    Gate gate = new Gate();
    DecodeAprilTag aprilTag = new DecodeAprilTag(this);

    @Override
    public void runOpMode() throws InterruptedException {

        chassis = new Chassis();
        chassis.init(this);

        flyWheel = new FlyWheel();
        flyWheel.init(this);

        gate = new Gate();
        gate.init(this);

        aprilTag = new DecodeAprilTag(this);
        aprilTag.initCamera();


        //chassis.turnToHeading(180,0.7,10000);
        //sleep(1000);
        //chassis.turnToHeading(-180,0.7,10000);
        chassis.resetIMU();
        telemetry.addData("odo pos", chassis.getPoseEstimate().getHeading(AngleUnit.DEGREES));
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
//                aprilTag.findAprilTag("BlueAllianceLeft");
//                AprilTagPoseFtc aprilTagPoseFtc = aprilTag.getCoordinate("BlueAllianceLeft");
//                sleep(1000);


            chassis.turnToAngle(10);
            sleep(1000);
            chassis.turnToAngle(-90);

        }
    }
}
