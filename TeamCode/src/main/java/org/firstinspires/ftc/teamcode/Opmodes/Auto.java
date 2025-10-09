package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Helper.Chassis;
import org.firstinspires.ftc.teamcode.Helper.FlyWheel;
import org.firstinspires.ftc.teamcode.Helper.Gate;
import org.firstinspires.ftc.teamcode.Helper.Util;

@Autonomous(name = "Alaqmar Auto 2.72;")

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

    FlyWheel flyWheel = new FlyWheel();
    Gate gate = new Gate();

    //DecodeAprilTag aprilTag = new DecodeAprilTag(this);

    @Override
    public void runOpMode() throws InterruptedException {

        chassis = new Chassis();
        chassis.init(this);

        flyWheel = new FlyWheel();
        flyWheel.init(this);

        gate = new Gate();
        gate.init(this);

        //DecodeAprilTag aprilTag = new DecodeAprilTag(this);
        //aprilTag.initCamera();


        while (opModeInInit()) {
            if (opModeInInit()) {
                chassis.odo.update();
                telemetry.addData("Odo x", chassis.odo.getEncoderX());
                telemetry.addData("Odo y", chassis.odo.getEncoderY());
                telemetry.update();
            }
        }
        waitForStart();

        while (opModeIsActive()) {


            // for(int i = 15; i <= 180; i += 15){
            //   chassis.turnToAngle(i);
            //  sleep(1500);
            //}
            //for(int i = -15; i >= -180; i -= 15) {
            //   chassis.turnToAngle(i);
            // sleep(1500);

            //chassis.moveDistance(0,20,0);
            //telemetry.addData("x:0,y:20,yaw:0","");
            //telemetry.update();
            //sleep(2000);
            //chassis.moveDistance(0,20,0);
            //telemetry.addData("x:0,y:-20,yaw:0","");
            //telemetry.update();
            //sleep(2000);
            //chassis.imuTurn(90);
            //chassis.moveDistance(10,10,0);
            //chassis.imuTurn(-90);
            //chassis.moveDistance(10,0,0);
            //chassis.imuTurn(90);
            //chassis.moveDistance(-10,0,0);


            while (opModeIsActive()) {


                //chassis.printOdoTelemetry();
                //chassis.printIMUTelemetry();
                //  telemetry.update();


//                aprilTag.findAprilTag("BlueAllianceLeft");
//                AprilTagPoseFtc aprilTagPoseFtc = aprilTag.getCoordinate("BlueAllianceLeft");
//                sleep(1000);

                //for (int i = 0; i < 5; i++){
                //chassis.imuTurnRight(90);
                //chassis.turnToHeadingWithImuDegrees(90, 0.5, 30000);
//                if (i % 2 == 0) {
//                    chassis.turnToHeadingWithImuDegrees(90, 0.5, 30000);
//                }else{
//                    chassis.turnToHeadingWithImuDegrees(-90, 0.5, 30000);
//                }
            }
        }
    }
}
