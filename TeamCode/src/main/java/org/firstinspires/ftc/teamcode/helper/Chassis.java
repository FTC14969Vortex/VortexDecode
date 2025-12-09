package org.firstinspires.ftc.teamcode.Helper;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.teamcode.Helper.Util;

public class Chassis {
    double leftFrontPower;
    double leftBackPower;
    double rightFrontPower;
    double rightBackPower;
    public GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    RevHubOrientationOnRobot hubOrientation;
    double COUNTS_PER_MOTOR_REV = 537.7;
    double DRIVE_GEAR_REDUCTION = 1.0;
    double WHEEL_DIAMETER_INCHES = 4.0;
    double COUNTS_PER_INCH;

    public DcMotor frontLeftDrive;
    public DcMotor backLeftDrive;
    public DcMotor frontRightDrive;
    public DcMotor backRightDrive;
    public IMU imu;

    private OpMode opMode;
    private DriveMode driveMode;

   private LinearOpMode linearOpMode;

    final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)


    public enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }

    public static enum Direction {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT
    }

    public void init(OpMode opMode) {

        this.opMode = opMode;
        frontLeftDrive = opMode.hardwareMap.get(DcMotor.class, "frontLeftDrive");
        backLeftDrive = opMode.hardwareMap.get(DcMotor.class, "backLeftDrive");
        frontRightDrive = opMode.hardwareMap.get(DcMotor.class, "frontRightDrive");
        backRightDrive = opMode.hardwareMap.get(DcMotor.class, "backRightDrive");

        odo = opMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        imu = opMode.hardwareMap.get(IMU.class, "imu");

        hubOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        imu.initialize(new IMU.Parameters(hubOrientation));
        imu.resetYaw();

        linearOpMode = (LinearOpMode)opMode;
        COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_INCHES * Math.PI);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        //Make these values more accurate to make our heading more accurate
        odo.setOffsets(-4.5, 8, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();


    }

    public LinearOpMode getLinearOpMode() {
        return linearOpMode;
    }

    public void setDriveMode(DriveMode driveMode) {
        // save driveMode to use in drive()
        this.driveMode = driveMode;

    }

    public void resetODOPosAndIMU() {
        odo.resetPosAndIMU();
    }

    public void updateTelemetry() {
        // opMode.telemetry.addData("Status", "Run Time: " + runtime);
        opMode.telemetry.addData("Drive Mode", driveMode);
        opMode.telemetry.addData("Front left/Right", JavaUtil.formatNumber(leftFrontPower, 4, 2) + ", " + JavaUtil.formatNumber(rightFrontPower, 4, 2));
        opMode.telemetry.addData("Back  left/Right", JavaUtil.formatNumber(leftBackPower, 4, 2) + ", " + JavaUtil.formatNumber(rightBackPower, 4, 2));
        odo.update();
        opMode.telemetry.addData("botHeading", JavaUtil.formatNumber(odo.getHeading(AngleUnit.RADIANS), 4, 2));
        opMode.telemetry.addData("botX", JavaUtil.formatNumber(odo.getPosX(DistanceUnit.CM), 4, 2));
        opMode.telemetry.addData("botY", JavaUtil.formatNumber(odo.getPosY(DistanceUnit.CM), 4, 2));
        opMode.telemetry.update();

    }

    public void drive(double distance) {
        if(distance < 0) {
            Util.moveRobot(frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive, odo, imu, Util.MovementDirection.BACKWARD, -distance, 0, opMode.telemetry);
        } else {
            Util.moveRobot(frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive, odo, imu, Util.MovementDirection.FORWARD, distance, 0, opMode.telemetry);
        }
    }

    public void strafe(double distance) {
        if(distance < 0) {
            Util.moveRobot(frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive, odo, imu, Util.MovementDirection.STRAFE_LEFT, -distance, 0, opMode.telemetry);
        } else {
            Util.moveRobot(frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive, odo, imu, Util.MovementDirection.STRAFE_RIGHT, distance, 0, opMode.telemetry);
        }
    }

    public void turn(double angle) {
        if(angle < 0) {
            Util.moveRobot(frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive, odo, imu, Util.MovementDirection.TURN_LEFT, 0, -angle, opMode.telemetry);
        } else {
            Util.moveRobot(frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive, odo, imu, Util.MovementDirection.TURN_RIGHT, 0, -angle, opMode.telemetry);
        }
    }


    // Returns the current pose from odometry in mm and radians
    public Pose2D getPoseEstimate() {
        odo.update();
        double x = odo.getPosX(DistanceUnit.MM);
        double y = odo.getPosY(DistanceUnit.MM);
        double heading = odo.getHeading(AngleUnit.RADIANS);
        return new Pose2D(DistanceUnit.MM, x, y, AngleUnit.RADIANS, heading);
    }

    public void stopRobot() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }



    //Gyroscope
    public static double HEADING_THRESHOLD = 1.0;
    public static double P_TURN_COEFF = 0.03;


    public void printOdoTelemetry(){
        Util.printAllOdoTelemetry(odo, opMode.telemetry);
    }

    public void printIMUTelemetry(){
        Util.printIMUTelemetry(imu, opMode.telemetry);
    }
}
