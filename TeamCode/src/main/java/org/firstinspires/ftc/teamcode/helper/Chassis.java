package org.firstinspires.ftc.teamcode.Helper;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

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
    private GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    RevHubOrientationOnRobot hubOrientation;
    double COUNTS_PER_MOTOR_REV = 537.7;
    double DRIVE_GEAR_REDUCTION = 1.0;
    double WHEEL_DIAMETER_INCHES = 4.0;
    double COUNTS_PER_INCH;

    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;
    private IMU imu;

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
        frontLeftDrive = opMode.hardwareMap.get(DcMotor.class, "leftFront");
        backLeftDrive = opMode.hardwareMap.get(DcMotor.class, "leftBack");
        frontRightDrive = opMode.hardwareMap.get(DcMotor.class, "rightFront");
        backRightDrive = opMode.hardwareMap.get(DcMotor.class, "rightBack");
        odo = opMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        imu = opMode.hardwareMap.get(IMU.class, "imu");

        hubOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );

        imu.initialize(new IMU.Parameters(hubOrientation));
        linearOpMode = (LinearOpMode)opMode;
        COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_INCHES * Math.PI);
        //Make these values more accurate to make our heading more accurate
        odo.setOffsets(-4.5, 8, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);


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
        double drive = Range.clip(y * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        double turn = Range.clip(-yaw * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
        double strafe = Range.clip(x * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

        moveRobot(drive, strafe, turn);
    }

    public void moveToPosition(double x, double y, double yaw) {


        double xErr = x - odo.getEncoderX();
        double yErr = y - odo.getEncoderY();
        double yawErr = Util.angleWrap(yaw - odo.getHeading(AngleUnit.RADIANS));

//        // Use the speed and turn "gains" to calculate how we want the robot to move.
//        double drive  = Range.clip(yErr * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
//        double turn   = Range.clip(-yawErr * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
//        double strafe = Range.clip(xErr * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
//
//        moveRobot(drive, strafe, turn);

        while (xErr != 0 && yErr != 0 && yawErr != 0) {
            xErr = x - odo.getEncoderX();
            yErr = y - odo.getEncoderY();
            yawErr = Util.angleWrap(yaw - odo.getHeading(AngleUnit.RADIANS));

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            double drive = Range.clip(yErr * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double turn = Range.clip(-yawErr * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            double strafe = Range.clip(xErr * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            moveRobot(drive, strafe, turn);
        }
    }
    public void setMotorWheelMode(DcMotor.RunMode runMode){
        frontLeftDrive.setMode(runMode);
        frontRightDrive.setMode(runMode);
        backLeftDrive.setMode(runMode);
        backRightDrive.setMode(runMode);
    }
    public int getAverageCurrentPositionAllWheels(){


        int fl = Math.abs(frontLeftDrive.getCurrentPosition());
        int bl = Math.abs(backLeftDrive.getCurrentPosition());
        int fr = Math.abs(frontRightDrive.getCurrentPosition());
        int br = Math.abs(backRightDrive.getCurrentPosition());


        int averageCurrentPosition = (fl + bl + fr + br)/4;
        return averageCurrentPosition;


    }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double frontLeftPower = x - y - yaw;
        double frontRightPower = x + y + yaw;
        double backLeftPower = x + y - yaw;
        double backRightPower = x - y + yaw;

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

    // Returns the current pose from odometry in mm and radians
    public Pose2D getPoseEstimate() {
        odo.update();
        double x = odo.getPosX(DistanceUnit.MM);
        double y = odo.getPosY(DistanceUnit.MM);
        double heading = odo.getHeading(AngleUnit.RADIANS);
        return new Pose2D(DistanceUnit.MM, x, y, AngleUnit.RADIANS, heading);
    }


    public double wrap180(double deg) {
        double x = (deg + 180.0) % 360.0;
        if (x < 0) x += 360.0;
        return x - 180.0;
    }

    final double MIN_TURN_PWR = 0.1;
    final double MAX_TURN_PWR = 0.8;
    final double ERROR_RANGE_DEGREE = 2;

    public void turnToAngle(double targetAngle) {

        while (((LinearOpMode) opMode).opModeIsActive()) {
            odo.update();
            double currentAngle = odo.getHeading(AngleUnit.DEGREES);
            double error = (targetAngle - currentAngle);

            opMode.telemetry.addData("Current Angle", currentAngle);
            opMode.telemetry.addData("Error", error);
            opMode.telemetry.update();

            if (Math.abs(error) < 2) {
                setPowerToWheels(0, 0, 0, 0);
                break;
            }

            double turnPower = 0.01 * error;
            turnPower = Math.max(-0.5, Math.min(turnPower, 0.5));

            drive(0, 0, turnPower);
        }
    }
    public static double P_DRIVE_COEFF = 0.05;
    public static final double minPower = 0.08;

    public void moveWithProportionalDeceleration(Direction direction, double maxPower, double distanceInches) {
        if (!linearOpMode.opModeIsActive()) return;

        int targetTicks = (int) (distanceInches * COUNTS_PER_INCH);


        setMotorWheelMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorWheelMode(DcMotor.RunMode.RUN_USING_ENCODER);


        while (linearOpMode.opModeIsActive()) {


            int remainingTicks = targetTicks - getAverageCurrentPositionAllWheels();
            if (remainingTicks <= 0) break;


            double progress = (double) remainingTicks / (double) targetTicks;
            double drivePower = minPower + (maxPower - minPower) * Math.pow(progress, 0.5);
            drivePower = Math.max(minPower, Math.min(drivePower, maxPower));




            //double calculatedPower = remainingTicks * P_DRIVE_COEFF;
            //double drivePower = Math.max(0.1, Math.min(Math.abs(calculatedPower), maxPower));




            switch (direction) {
                case FORWARD:
                    setPowerToWheels(drivePower, drivePower, drivePower, drivePower);
                    break;
                case BACKWARD:
                    setPowerToWheels(-drivePower, -drivePower, -drivePower, -drivePower);
                    break;
                case LEFT:
                    setPowerToWheels(-drivePower, drivePower, drivePower, -drivePower);
                    break;
                case RIGHT:
                    setPowerToWheels(drivePower, -drivePower, -drivePower, drivePower);
                    break;
            }
        }
        setPowerToWheels(0,0,0,0);
    }




    //Gyroscope
    public static double HEADING_THRESHOLD = 1.0;
    public static double P_TURN_COEFF = 0.03;


    public void moveWithProportionalDecelerationAndHeading(Direction direction, double maxPower, double distanceInches, Double holdHeadingDeg){

        double COUNTS_PER_MOTOR_REV = 537.7;
        double DRIVE_GEAR_REDUCTION = 1.0;
        double WHEEL_DIAMETER_INCHES = 4.0;
        double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_INCHES * Math.PI);




        double MIN_POWER       = 0.08;  // just above stall for your drivetrain
        double KP_HEADING      = 0.012; // start here; tune on the field
        double KI_HEADING      = 0.000; // optional (keep 0 to start)
        double KD_HEADING      = 0.000; // optional (keep 0 to start)
        double MAX_YAW_CORR    = 0.25;  // cap on turn correction (0..1)
        int    STOP_TOL_TICKS  = 20;    // ~0.4 in @ 45 cpi


    LinearOpMode linearOpMode = (LinearOpMode)opMode;

        if (!linearOpMode.opModeIsActive()) return;


        // --- Prep encoders ---
        int targetTicks = (int) Math.round(Math.abs(distanceInches) * COUNTS_PER_INCH);
        setMotorWheelMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorWheelMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // --- Prep IMU ---
        // Assuming "imu" is com.qualcomm.hardware.bosch.BNO055IMU or the newer IMU interface and already initialized.
        // Choose degrees for angle un` it when you init IMU elsewhere.
        imu.resetYaw();
        double yaw0 = -odo.getHeading(AngleUnit.DEGREES); // your helper that reads yaw in degrees
        double headingSetpoint = (holdHeadingDeg != null) ? holdHeadingDeg : yaw0;


        // Heading PID state
        double lastErr = 0, errI = 0;
        long   lastT   = System.nanoTime();


        while (linearOpMode.opModeIsActive()) {


            int remaining = targetTicks - getAverageCurrentPositionAllWheels();
            if (remaining <= STOP_TOL_TICKS) break;


            // Linear taper (1 -> 0)
            double progress = Math.max(0.0, Math.min(1.0, (double) remaining / (double) targetTicks));
            // Optional easing for earlier slowdown (gamma<1 decelerates earlier)
            double gamma = 0.7;
            double tapered = Math.pow(progress, gamma);


            double drivePower = MIN_POWER + (maxPower - MIN_POWER) * tapered;
            drivePower = Range.clip(drivePower, MIN_POWER, maxPower);


            // --- Heading hold (PID) ---
            double yaw = -odo.getHeading(AngleUnit.DEGREES); // current yaw in degrees
            double err = wrap180(headingSetpoint - yaw);


            long now = System.nanoTime();
            double dt = Math.max(1e-6, (now - lastT) / 1e9); // seconds
            lastT = now;


            // PI(D)
            errI += err * dt;
            // simple anti-windup
            errI = Range.clip(errI, -50.0, 50.0);


            double errD = (err - lastErr) / dt;
            lastErr = err;


            double turnCorr = KP_HEADING * err + KI_HEADING * errI + KD_HEADING * errD;
            turnCorr = Range.clip(turnCorr, -MAX_YAW_CORR, MAX_YAW_CORR);


            // --- Command mix ---
            // axial: forward/back; lateral: strafe; yaw: heading correction
            double axial = 0, lateral = 0;
            switch (direction) {
                case FORWARD:  axial =  drivePower; break;
                case BACKWARD: axial = -drivePower; break;
                case LEFT:     lateral =  drivePower; break;
                case RIGHT:    lateral = -drivePower; break;
            }
            double yawCmd = turnCorr;


            // mecanum mix
            double flPower = axial + lateral + yawCmd;
            double frPower = axial - lateral - yawCmd;
            double blPower = axial - lateral + yawCmd;
            double brPower = axial + lateral - yawCmd;


            // Normalize if any exceeds 1.0
            double maxAbs = Math.max(1.0,
                    Math.max(Math.abs(flPower),
                            Math.max(Math.abs(frPower),
                                    Math.max(Math.abs(blPower), Math.abs(brPower)))));
            flPower /= maxAbs; frPower /= maxAbs; blPower /= maxAbs; brPower /= maxAbs;


            // Scale to preserve the intended drive magnitude (keep <= maxPower)
            flPower *= maxPower; frPower *= maxPower; blPower *= maxPower; brPower *= maxPower;


            // Ensure we don't drop below MIN_POWER along the commanded axis (helps overcome static friction)
            // but allow the heading correction to modulate around it.
            // Only enforce MIN_POWER on the dominant drive component:
            if (direction == Direction.FORWARD || direction == Direction.BACKWARD) {
                double sign = Math.signum(axial);
                flPower = sign * Math.max(MIN_POWER, Math.abs(flPower));
                frPower = sign * Math.max(MIN_POWER, Math.abs(frPower));
                blPower = sign * Math.max(MIN_POWER, Math.abs(blPower));
                brPower = sign * Math.max(MIN_POWER, Math.abs(brPower));
            } else {
                double sign = Math.signum(lateral);
                // For strafes, friction is higher—MIN_POWER helps a lot
                flPower = (Math.signum(flPower) == 0 ? sign : Math.signum(flPower)) * Math.max(MIN_POWER, Math.abs(flPower));
                frPower = (Math.signum(frPower) == 0 ? sign : Math.signum(frPower)) * Math.max(MIN_POWER, Math.abs(frPower));
                blPower = (Math.signum(blPower) == 0 ? sign : Math.signum(blPower)) * Math.max(MIN_POWER, Math.abs(blPower));
                brPower = (Math.signum(brPower) == 0 ? sign : Math.signum(brPower)) * Math.max(MIN_POWER, Math.abs(brPower));
            }


            // Apply powers
            setPowerToWheels( flPower, frPower, blPower, brPower);


        }


        // Stop hard
        setPowerToWheels( 0, 0, 0, 0);
    }
    public void turnToHeading2(double targetHeadingDeg, double maxTurnSpeed, int timeoutMillis){
        //Util.turnToAngleIMU(targetHeadingDeg,maxTurnSpeed, );
    }

    public void printOdoTelemetry(){
        Util.printOdoTelemetry(odo, opMode.telemetry);
    }

    public void printIMUTelemetry(){
        Util.printIMUTelemetry(imu, opMode.telemetry);
    }
}
