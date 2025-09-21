package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Helper.Chassis;
import org.firstinspires.ftc.teamcode.Helper.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Helper.Intake;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "TestAutonomousMovements9;")
public class TestAutonomousMovements extends LinearOpMode {

    Chassis chassis = null;
    Intake intake = null;
    double leftFrontPower;
    double leftBackPower;
    double rightFrontPower;
    double rightBackPower;


    // Reference to GoBilda's Pinpoint odometry driver
    public GoBildaPinpointDriver odo;


    // Drive motor references
    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;
    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        chassis = new Chassis();
        chassis.init(this);

        intake = new Intake(this);
        intake.init();



        waitForStart();

        if (opModeIsActive()) {




        }
    }
}