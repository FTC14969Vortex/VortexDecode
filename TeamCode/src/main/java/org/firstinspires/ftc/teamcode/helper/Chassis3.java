package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Chassis3 {
    public DcMotor FLMotor = null;
    public DcMotor FRMotor = null;
    public DcMotor BLMotor = null;
    public DcMotor BRMotor = null;
    //IMU
    public GoBildaPinpointDriver odo;

    private LinearOpMode myOpMode;

    private PIDControl pid;

    private ElapsedTime holdTimer;

    public Chassis3(LinearOpMode opmode) {
        pid = new PIDControl();
        holdTimer = new ElapsedTime();
        myOpMode = opmode;
    }


    public void init() throws InterruptedException {

        // !!!  Set the drive direction to ensure positive power drives each wheel forward.
        FLMotor = setupDriveMotor("frontLeftDrive", DcMotor.Direction.REVERSE);
        FRMotor = setupDriveMotor("frontRightDrive", DcMotor.Direction.FORWARD);
        BLMotor = setupDriveMotor("backLeftDrive", DcMotor.Direction.REVERSE);
        BRMotor = setupDriveMotor("backRightDrive", DcMotor.Direction.FORWARD);


        odo = myOpMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-0.5, 1.5, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);


        // zero out all the odometry readings.
        odo.resetPosAndIMU();

        //Telemetry
        myOpMode.telemetry.addData("Odo X", odo.getEncoderY());
        myOpMode.telemetry.addData("Odo Y", odo.getEncoderY());
        myOpMode.telemetry.addData("Heading", odo.getHeading(AngleUnit.DEGREES));
    }

    private DcMotor setupDriveMotor(String deviceName, DcMotor.Direction direction) {
        DcMotor aMotor = myOpMode.hardwareMap.get(DcMotor.class, deviceName);
        aMotor.setDirection(direction);
        aMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset Encoders to zero
        aMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //TODO: Check if this needs to be changed to run without encoder
        aMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // Requires motor encoder cables to be hooked up.
        return aMotor;
    }

    public void drive(double distanceInches, double power, double holdTime) {
        odo.resetPosAndIMU();
        odo.update();

        holdTimer.reset();

        while(myOpMode.opModeIsActive()) {
            double powerX = pid.getOutput(distanceInches, odo.getPosX(DistanceUnit.INCH));
            double powerY = pid.getOutput(odo.getPosY(DistanceUnit.INCH), odo.getPosY(DistanceUnit.INCH));
            double powerHeading = pid.getOutput(odo.getHeading(AngleUnit.DEGREES), odo.getHeading(AngleUnit.DEGREES));

            moveRobot(powerX, powerY, powerHeading);

            // Time to exit?
            if (pid.isInPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            myOpMode.sleep(10);
        }

        myOpMode.sleep(10);
    }

    public void moveRobot(double drive, double strafe, double yaw){

        double FLPower = drive - strafe - yaw;
        double FRPower = drive + strafe + yaw;
        double BLPower = drive + strafe - yaw;
        double BRPower = drive - strafe + yaw;

        double max = Math.max(Math.abs(FLPower), Math.abs(FRPower));
        max = Math.max(max, Math.abs(BLPower));
        max = Math.max(max, Math.abs(BRPower));

        //normalize the motor values
        if (max > 1.0)  {
            FLPower /= max;
            FRPower /= max;
            BLPower /= max;
            BRPower /= max;
        }

        //send power to the motors
        FLMotor.setPower(FLPower);
        FRMotor.setPower(FRPower);
        BLMotor.setPower(BLPower);
        BRMotor.setPower(BRPower);
    }
}
