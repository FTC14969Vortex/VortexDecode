package org.firstinspires.ftc.teamcode.helper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Util {
    public static double angleWrap(double degrees) {
        while (degrees > 180) {
            degrees -= 360;
        }
        while (degrees < -180) {
            degrees += 360;
        }
        return degrees;
    }
    /**
     * Clips a value to be within a minimum and maximum range.
     */
    public static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public static void sleepThread(int millis){
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
        }
    }

    public static void printOdoTelemetry(GoBildaPinpointDriver odo, Telemetry telemetry ){
        telemetry.addData("Odo Device Name: ", odo.getDeviceName());
        telemetry.addData("Odo Device Version: ", odo.getDeviceVersion());
        telemetry.addData("Odo Connection Info: ", odo.getConnectionInfo());
        telemetry.addData("Odo Device Status: ", odo.getDeviceStatus());
        telemetry.addData("Odo Position X (cms): ", odo.getPosX(DistanceUnit.CM));
        telemetry.addData("Odo Position Y (cms): ", odo.getPosY(DistanceUnit.CM));
        telemetry.addData("Odo Heading (degrees): ", odo.getHeading(AngleUnit.DEGREES));
    }

    public static void printIMUTelemetry(IMU imu, Telemetry telemetry ){
        telemetry.addData("IMU Device Name: ", imu.getDeviceName());
        telemetry.addData("IMU Device Version: ", imu.getVersion());
        telemetry.addData("IMU Connection Info: ", imu.getConnectionInfo());
        telemetry.addData("IMU Yaw: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("IMU Pitch: ", imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
        telemetry.addData("IMU Roll: ", imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));
    }



    /**
     * Turns the robot to a specific angle using the IMU Yaw.
     * This is a self-contained method with no external utility calls.
     *
     * @param targetAngle The absolute angle in degrees to turn to (-180 to 180).
     * @param turnPower   The power at which to turn (0.0 to 1.0).
     * @param leftFront   The front left drive motor.
     * @param leftBack    The back left drive motor.
     * @param rightFront  The front right drive motor.
     * @param rightBack   The back right drive motor.
     * @param imu         The IMU sensor.
     * @param opMode      The LinearOpMode instance, used to check if the OpMode is still active.
     * @param telemetry   The telemetry object for displaying debug information.
     */
    public static void turnToAngleIMU(double targetAngle, double turnPower,
                             DcMotor leftFront, DcMotor leftBack,
                             DcMotor rightFront, DcMotor rightBack,
                             IMU imu, LinearOpMode opMode, Telemetry telemetry) {

        // Ensure the opMode is still active before starting.
        if (!opMode.opModeIsActive()) {
            return;
        }

        // --- Constants ---
        // How close to the target angle we need to be to stop.
        final double HEADING_TOLERANCE = 1.0; // degrees

        // --- Main Turn Logic ---

        // Reset the IMU's Yaw angle to 0. This makes the robot's current heading the new reference point.
        // All turns will be relative to this new zero heading.
        imu.resetYaw();

        // Get the current robot orientation from the IMU.
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double currentAngle = orientation.getYaw(AngleUnit.DEGREES);

        // Calculate the error, which is the difference between where we want to be and where we are.
        double error = targetAngle - currentAngle;

        // Loop until the error is within our tolerance and the OpMode is still running.
        while (opMode.opModeIsActive() && Math.abs(error) > HEADING_TOLERANCE) {
            // This logic handles "wrap-around" on the -180 to 180 degree range.
            // For example, turning from -170 to 170 degrees should be a 20-degree right turn, not a 340-degree left turn.
            if (error > 180) {
                error -= 360;
            }
            if (error < -180) {
                error += 360;
            }

            // Determine the turn direction based on the sign of the error.
            // If error is positive, we need to turn left (counter-clockwise).
            // If error is negative, we need to turn right (clockwise).
            double directionSign = Math.signum(error);

            // Set the power to the motors. Left motors get negative power for a left turn,
            // and positive for a right turn. Right motors do the opposite.
            leftFront.setPower(-turnPower * directionSign);
            leftBack.setPower(-turnPower * directionSign);
            rightFront.setPower(turnPower * directionSign);
            rightBack.setPower(turnPower * directionSign);

            // --- Telemetry and Loop Update ---
            // Display debug information on the Driver Station.
            telemetry.addData("Target Angle", "%.2f", targetAngle);
            telemetry.addData("Current Angle", "%.2f", currentAngle);
            telemetry.addData("Error", "%.2f", error);
            telemetry.update();

            // Re-calculate the error for the next loop iteration.
            orientation = imu.getRobotYawPitchRollAngles();
            currentAngle = orientation.getYaw(AngleUnit.DEGREES);
            error = targetAngle - currentAngle;
        }

        // --- Stop the Robot ---
        // Once the loop is finished, stop all drive motors.
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}
