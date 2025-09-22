package org.firstinspires.ftc.teamcode.helper;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Chassis {
    double leftFrontPower;
    double leftBackPower;
    double rightFrontPower;
    double rightBackPower;
    private GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;

    private OpMode opMode;
    private DriveMode driveMode;

    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)


    public enum DriveMode {

        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }

    public void init(OpMode opMode) {

        this.opMode = opMode;
        frontLeftDrive = opMode.hardwareMap.get(DcMotor.class, "leftFront");
        backLeftDrive = opMode.hardwareMap.get(DcMotor.class, "leftBack");
        frontRightDrive = opMode.hardwareMap.get(DcMotor.class, "rightFront");
        backRightDrive = opMode.hardwareMap.get(DcMotor.class, "rightBack");
        odo = opMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");


        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        odo.recalibrateIMU();
        odo.resetPosAndIMU();
    }

    public void setDriveMode(DriveMode driveMode) {
        // save driveMode to use in drive()
        this.driveMode = driveMode;

    }

    public void resetIMU() {
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
    // TeleOp Mode Methods

    /**
     * @param axial   The forward/backward power from left joystick Y direction (-1.0 to 1.0).
     * @param lateral The strafing (left/right) power from left joystick X direction (-1.0 to 1.0).
     * @param yaw     The turning/rotational power from right joystick X direction (-1.0 to 1.0).
     */
    public void drive(double axial, double lateral, double yaw) {
        // If in field centric mode read botHeading from odo otherwise set botHeading equal to zero
        double botHeading;
        if (driveMode == DriveMode.FIELD_CENTRIC) {
            odo.update();
            botHeading = odo.getHeading(AngleUnit.RADIANS);// Get the robot's heading in radians

        } else {
            botHeading = 0;
        }


        /*// Rotate the movement direction counter to the bot's rotation
        double rotX = lateral * Math.cos(botHeading) - axial * Math.sin(botHeading);
        double rotY = lateral * Math.sin(botHeading) + axial * Math.cos(botHeading);
        rotX = rotX * 1.1; //counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double speed = 1.7;
        leftFrontPower = (rotY + rotX + yaw);
        rightFrontPower = (rotY - rotX - yaw);
        leftBackPower = (rotY - rotX + yaw);
        rightBackPower = (rotY + rotX - yaw);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(yaw), 1) * speed; //Multiply by 1.7 to reduce speed

        leftFrontPower = leftFrontPower / denominator;
        rightFrontPower = rightFrontPower / denominator;
        leftBackPower = leftBackPower / denominator;
        rightBackPower = rightBackPower / denominator;*/


        double max;
        double lateral_1 = lateral * Math.cos(botHeading) - axial * Math.sin(botHeading);
        double axial_1 = lateral * Math.sin(botHeading) + axial * Math.cos(botHeading);

        leftFrontPower = axial_1 + lateral_1 + yaw;
        rightFrontPower = axial_1 - lateral_1 - yaw;
        leftBackPower = axial_1 - lateral_1 + yaw;
        rightBackPower = axial_1 + lateral_1 - yaw;
        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(leftFrontPower), Math.abs(rightFrontPower), Math.abs(leftBackPower), Math.abs(rightBackPower)));
        if (max > 1) {
            leftFrontPower = leftFrontPower / max;
            rightFrontPower = rightFrontPower / max;
            leftBackPower = leftBackPower / max;
            rightBackPower = rightBackPower / max;
        }


        // Send calculated power to wheels.
        setPowerToWheels(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        //setSmoothPowerToWheels(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }


    public double smoothPower(double currentPower, double targetPower) {
            double sigma = 0.1;

        if (currentPower < targetPower) {
            return currentPower + sigma;
        } else if (currentPower > targetPower) {
            return currentPower - sigma;
        } else if (Math.abs(currentPower - targetPower) <= sigma) {
            return currentPower;
        }
        return currentPower;
    }

    public void setSmoothPowerToWheels(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {

        leftFrontPower = smoothPower(frontLeftDrive.getPower(), leftFrontPower);
        rightFrontPower = smoothPower(frontRightDrive.getPower(), rightFrontPower);
        leftBackPower = smoothPower(backLeftDrive.getPower(), leftBackPower);
        rightBackPower = smoothPower(backRightDrive.getPower(), rightBackPower);

        setPowerToWheels(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    public void setPowerToWheels(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        frontLeftDrive.setPower(leftFrontPower);
        frontRightDrive.setPower(rightFrontPower);
        backLeftDrive.setPower(leftBackPower);
        backRightDrive.setPower(rightBackPower);
    }

    public void moveDistance(double x, double y, double yaw) {

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        double drive  = Range.clip(y * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        double turn   = Range.clip(-yaw * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
        double strafe = Range.clip(x * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

        moveRobot(drive, strafe, turn);
    }
    public void moveToPosition(double x, double y, double yaw) {


        double xErr = x - odo.getEncoderX();
        double yErr = y - odo.getEncoderY();
        double yawErr = angleWrap(yaw - odo.getHeading(AngleUnit.RADIANS));

//        // Use the speed and turn "gains" to calculate how we want the robot to move.
//        double drive  = Range.clip(yErr * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
//        double turn   = Range.clip(-yawErr * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
//        double strafe = Range.clip(xErr * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
//
//        moveRobot(drive, strafe, turn);

        while(xErr!= 0 && yErr != 0 && yawErr!=0) {
            xErr = x - odo.getEncoderX();
            yErr = y - odo.getEncoderY();
            yawErr = angleWrap(yaw - odo.getHeading(AngleUnit.RADIANS));

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            double drive  = Range.clip(yErr * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double turn   = Range.clip(-yawErr * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            double strafe = Range.clip(xErr * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            moveRobot(drive, strafe, turn);
        }
    }
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double frontLeftPower    =  x - y - yaw;
        double frontRightPower   =  x + y + yaw;
        double backLeftPower     =  x + y - yaw;
        double backRightPower    =  x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Send powers to the wheels.
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
    }

    // Turn robot in place to desired heading using proportional control
    public void turnToHeading(double targetHeadingDeg, double maxTurnSpeed, int timeoutMillis) {

        double currentHeading = odo.getHeading(AngleUnit.RADIANS);

        opMode.telemetry.addData("Current Heading in Radian 1 = ", currentHeading);

        odo.recalibrateIMU();
        odo.resetPosAndIMU();
        //odo.setHeading(0, AngleUnit.RADIANS);
        odo.update();

        currentHeading = odo.getHeading(AngleUnit.RADIANS);
        opMode.telemetry.addData("Current Heading in Radian 2 = ", currentHeading);
        opMode.telemetry.update();

        putThreadToSleep(3000);

        double targetHeadingRad = Math.toRadians(targetHeadingDeg);


        final double ANGLE_TOLERANCE_RAD = Math.toRadians(4);
        ElapsedTime timer = new ElapsedTime();


        while (((LinearOpMode)opMode).opModeIsActive() && timer.milliseconds() < timeoutMillis) {
            currentHeading = odo.getHeading(AngleUnit.RADIANS);
            //opMode.telemetry.addData("Current Heading in Radian=",currentHeading);
            //opMode.telemetry.update();

            double error = angleWrap(targetHeadingRad - currentHeading);
            //opMode.telemetry.addData("Error value=",error);
            //opMode.telemetry.update();

            if (Math.abs(error) < ANGLE_TOLERANCE_RAD){
                break;
            }else {
                opMode.telemetry.addData("Current Heading in Radian=",currentHeading);
                opMode.telemetry.addData("Absolute Error value=", Math.abs(error));
                opMode.telemetry.update();
                putThreadToSleep(3000);
            }


            double turnPower = clip(error * 0.8, -maxTurnSpeed, maxTurnSpeed);

            drive(0, 0, turnPower, 0.5);
        }

       setPowerToWheels(0,0,0,0);
    }
    // Returns the current pose from odometry in mm and radians
    public Pose2D getPoseEstimate() {
        double x = odo.getPosX(DistanceUnit.MM);
        double y = odo.getPosY(DistanceUnit.MM);
        double heading = odo.getHeading(AngleUnit.RADIANS);
        return new Pose2D(DistanceUnit.MM, x, y, AngleUnit.RADIANS, heading);
    }


    // Utility method to constrain values
    private double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }


    private void putThreadToSleep(int millis){
        try {
            Thread.sleep(millis);
        } catch (InterruptedException ignored) {

        }
    }

    // Wrap angle between -PI and +PI
    private double angleWrap(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }

    public void turnToAngle(double targetAngle) {
        targetAngle = normalizeAngle(targetAngle);

        while(((LinearOpMode)opMode).opModeIsActive()) {
            double currentAngle = odo.getHeading(AngleUnit.DEGREES);
            double error = normalizeAngle(targetAngle - currentAngle);

            opMode.telemetry.addData("Current Angle", currentAngle);
            opMode.telemetry.addData("Error", error);
            opMode.telemetry.update();

            if(Math.abs(error) < 2) {
                setPowerToWheels(0,0,0,0);
                break;
            }

            double turnPower = 0.01 * error;
            turnPower = Math.max(-0.5, Math.min(turnPower, 0.5));

            drive(0,0,turnPower);
        }
    }
    double normalizeAngle(double angle) {
        while(angle > 180) angle -=360;
        while(angle < 180) angle += 360;
        return angle;
    }


    public void drive(double axial, double lateral, double yaw, double speed) {
        // If field-centric, adjust input based on robot heading
        double botHeading = 0;
        if(driveMode == DriveMode.FIELD_CENTRIC) {
            odo.update();
            botHeading = -odo.getHeading(AngleUnit.RADIANS);
            // This will likely change if the odometry unit is mounted differently.
        }


        // Apply heading rotation to axial/lateral values
        double lateral_1 = lateral * Math.cos(botHeading) - axial * Math.sin(botHeading);
        double axial_1 = lateral * Math.sin(botHeading) + axial * Math.cos(botHeading);


        // Calculate raw motor powers
        leftFrontPower = speed*(axial_1 + lateral_1 + yaw);
        rightFrontPower = speed*(axial_1 - lateral_1 - yaw);
        leftBackPower = speed*(axial_1 - lateral_1 + yaw);
        rightBackPower = speed*(axial_1 + lateral_1 - yaw);


        // Normalize powers to stay within [-1, 1]
//        double max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(leftFrontPower), Math.abs(rightFrontPower), Math.abs(leftBackPower), Math.abs(rightBackPower)));
//        if (max > 1) {
//            leftFrontPower /= max;
//            rightFrontPower /= max;
//            leftBackPower /= max;
//            rightBackPower /= max;
//        }


        // Apply final power values to motors
        frontLeftDrive.setPower(leftFrontPower);
        frontRightDrive.setPower(rightFrontPower);
        backLeftDrive.setPower(leftBackPower);
        backRightDrive.setPower(rightBackPower);
    }


    public void setPowerToWheels(double power) {
        setPowerToWheels(power, power, power, power);
    }

}

