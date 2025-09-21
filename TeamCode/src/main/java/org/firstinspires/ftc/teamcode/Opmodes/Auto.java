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


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Alaqmar Auto;")
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

    @Override
    public void runOpMode() throws InterruptedException {

        chassis = new Chassis();
        chassis.init(this);

         flyWheel = new FlyWheel();
        flyWheel.init(this);

        gate = new Gate();
        gate.init(this);

        waitForStart();

        if (opModeIsActive()) {

        chassis.moveRobot(0,0,180);


        }
    }
}