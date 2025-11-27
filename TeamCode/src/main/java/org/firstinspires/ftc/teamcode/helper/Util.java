package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

public class Util {

    public static class AlignmentResult {
        public boolean success;
        public double distance;
        public double bearing;
        public double yaw;

        public AlignmentResult(boolean success, double distance, double bearing, double yaw) {
            this.success = success;
            this.distance = distance;
            this.bearing = bearing;
            this.yaw = yaw;
        }
    }

    public static class FlyWheelSpinUpResult {
        public boolean success;
        public long durationMs;
        public double achievedVelocity;
        public double targetVelocity;

        public FlyWheelSpinUpResult(boolean success, long durationMs, double achievedVelocity, double targetVelocity) {
            this.success = success;
            this.durationMs = durationMs;
            this.achievedVelocity = achievedVelocity;
            this.targetVelocity = targetVelocity;
        }
    }

    /**
     * Waits for the flywheel to reach the target shooting velocity with configurable tolerance.
     * Ramps up at full power, then fine-tunes down if overshooting, and finally sets to shooting power.
     * Total time for both phases combined will not exceed maxWaitTime.
     */
    public static FlyWheelSpinUpResult waitForFlyWheelShootingVelocity(FlyWheel flyWheel, long velocity, double maxWaitTime, Telemetry telemetry) {
        // Configuration constants
        final double VELOCITY_TOLERANCE_PERCENT = 2.0;  // Default 2% tolerance
        final double SHOOTING_POWER_BOOST_PERCENT = 30.0;  // Default 20% boost for shooting
        final double JAM_DETECTION_THRESHOLD = 0.75;  // 75% of target velocity

        // Power calculation constants (for velocity-to-power conversion)
        final double BASE_POWER = 0.45;  // Motor power at reference velocity (1200 RPM)
        final int REFERENCE_VELOCITY = 1200;  // Reference velocity in RPM for base power
        final double POWER_INCREMENT_PER_1000_RPM = 0.1;  // Power increases by 0.1 for every 1000 RPM above reference

        long startTime = System.currentTimeMillis();
        long durationInMillis = 0;
        int loopCounter = 0;

        // Calculate tolerance thresholds
        double lowerThreshold = (1.0 - VELOCITY_TOLERANCE_PERCENT / 100.0) * velocity;  // 98% of target
        double upperThreshold = (1.0 + VELOCITY_TOLERANCE_PERCENT / 100.0) * velocity;  // 102% of target
        double jamThreshold = JAM_DETECTION_THRESHOLD * velocity;  // 90% of target
        double boostMultiplier = 1.0 + SHOOTING_POWER_BOOST_PERCENT / 100.0;  // 1.2 for 20% boost

        double initVelocity = flyWheel.getVelocity();
        telemetry.addData("Init Velocity", "%.0f RPM (%.0f%% of target), initVelocity, (initVelocity / velocity) * 100);

        // Phase 1: Ramp up to target velocity (lower threshold)
        flyWheel.setPower(1.0); // Ramp up at full power

        while (flyWheel.getVelocity() < lowerThreshold) {
            loopCounter++;
            sleepThread(50);

            durationInMillis = System.currentTimeMillis() - startTime;
            if (durationInMillis > maxWaitTime) {
                telemetry.addData("Warning", "Timeout during ramp-up");
                double achievedVelocity = flyWheel.getVelocity();
                boolean success = achievedVelocity >= jamThreshold;
                telemetry.addData("Achieved Velocity", "%.0f RPM (%.0f%% of target)", achievedVelocity, (achievedVelocity / velocity) * 100);
                return new FlyWheelSpinUpResult(success, durationInMillis, achievedVelocity, velocity);
            }
        }

        // Phase 2: Fine-tune down if overshooting (above upper threshold)
        while (flyWheel.getVelocity() > upperThreshold) {
            durationInMillis = System.currentTimeMillis() - startTime;
            if (durationInMillis > maxWaitTime) {
                telemetry.addData("Warning", "Timeout during fine-tuning");
                break;
            }

            flyWheel.setPower(-1); // Brake to slow down
            sleepThread(50);
        }

        // Phase 3: Set to calculated shooting power with boost
        // Formula: base_power + (target_velocity - reference_velocity) * power_per_rpm
        // Example: For 1200 RPM: 0.45 + (1200-1200)/1000 = 0.45, then * 1.2 = 0.54
        // Example: For 1400 RPM: 0.45 + (1400-1200)/1000 = 0.65, then * 1.2 = 0.78
        double calculatedPower = BASE_POWER + (velocity - REFERENCE_VELOCITY) / (1000.0 / POWER_INCREMENT_PER_1000_RPM);
        flyWheel.setPower(calculatedPower * boostMultiplier);

        durationInMillis = System.currentTimeMillis() - startTime;
        double achievedVelocity = flyWheel.getVelocity();
        telemetry.addData("Ramp-up loops", loopCounter);
        telemetry.addData("Time to velocity (ms)", durationInMillis);

        return new FlyWheelSpinUpResult(true, durationInMillis, achievedVelocity, velocity);
    }

    /**
     * Waits for the flywheel to decelerate to or below the target velocity.
     * Applies reverse power to actively brake the flywheel until it slows down.
     * Total time will not exceed maxWaitTime.
     */
    public static long waitForFlyWheelStopVelocity(FlyWheel flyWheel, long velocity, double maxWaitTime, Telemetry telemetry) {
        long startTime = System.currentTimeMillis();
        double startFlyWheelVelocity = flyWheel.getVelocity();
        double currentFlyWheelVelocity = flyWheel.getVelocity();
        long durationInMillis = 0;

        telemetry.addData("Start FlyWheel Velocity", startFlyWheelVelocity);

        // Apply reverse power to actively brake the flywheel
        while (currentFlyWheelVelocity > velocity) {
            flyWheel.setPower(-1); // Full reverse power for braking
            sleepThread(50);
            currentFlyWheelVelocity = flyWheel.getVelocity();

            // Check for timeout
            durationInMillis = System.currentTimeMillis() - startTime;
            if (durationInMillis > maxWaitTime) {
                telemetry.addData("Warning", "Timeout waiting for flywheel to stop");
                break;
            }
        }

        // Stop the flywheel motor
        flyWheel.setPower(0.0);

        // Calculate final duration and log results
        durationInMillis = System.currentTimeMillis() - startTime;
        double endFlyWheelVelocity = flyWheel.getVelocity();
        telemetry.addData("End FlyWheel Velocity", endFlyWheelVelocity);
        telemetry.addData("Stop Duration (ms)", durationInMillis);

        return durationInMillis;
    }


    public static void sleepThread(int millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
        }
    }

    public static void printAllOdoTelemetry(GoBildaPinpointDriver odo, Telemetry telemetry) {
        telemetry.addData("Odo Device Name: ", odo.getDeviceName());
        telemetry.addData("Odo Device Version: ", odo.getDeviceVersion());
        telemetry.addData("Odo Connection Info: ", odo.getConnectionInfo());
        telemetry.addData("Odo Device Status: ", odo.getDeviceStatus());
        telemetry.addData("Odo Position X (cms): ", odo.getPosX(DistanceUnit.CM));
        telemetry.addData("Odo Position Y (cms): ", odo.getPosY(DistanceUnit.CM));
        telemetry.addData("Odo Heading (radians): ", odo.getHeading(AngleUnit.RADIANS));
        telemetry.update();
    }

    public static void printOdoPositionTelemetry(GoBildaPinpointDriver odo, Telemetry telemetry) {
        odo.update();

        Pose2D pose = odo.getPosition();

        telemetry.addData("Odo Encoder X : ", odo.getEncoderX());
        telemetry.addData("Odo Encoder Y : ", odo.getEncoderY());
        telemetry.addData("Odo Pose X (cms): ", pose.getX(DistanceUnit.CM));
        telemetry.addData("Odo Pose Y (cms): ", pose.getY(DistanceUnit.CM));
        telemetry.addData("Odo Position X (cms): ", odo.getPosX(DistanceUnit.CM));
        telemetry.addData("Odo Position Y (cms): ", odo.getPosY(DistanceUnit.CM));
        //telemetry.addData("Odo Heading (degrees): ", odo.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Odo Heading (radians): ", odo.getHeading(AngleUnit.RADIANS));
        telemetry.addData("Odo Heading (rad to deg): ", Math.toDegrees(odo.getHeading(AngleUnit.RADIANS)));

    }

    public static void printIMUTelemetry(IMU imu, Telemetry telemetry) {
        //telemetry.addData("IMU Device Name: ", imu.getDeviceName());
        //telemetry.addData("IMU Device Version: ", imu.getVersion());
        //telemetry.addData("IMU Connection Info: ", imu.getConnectionInfo());

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

    public static void addKickerTelemetry(Kicker kicker, Telemetry telemetry) {
        telemetry.addData("kicker.getPosition() - ", kicker.getPosition());
        telemetry.addData("kicker.getGatePosition() - ", kicker.getGatePosition());
    }

    public static void telemetryFlyWheelVelocity(FlyWheel flyWheel, double flyWheelPower, DistanceSensor frontDistanceSensor, int runForMS, Telemetry telemetry) {

        telemetry.addData("Power - " + flyWheelPower, "Distance - " + Util.getDistance(frontDistanceSensor, telemetry));
        long startTime = System.currentTimeMillis();
        long intermidiateTime = System.currentTimeMillis();
        long endTime = 0;
        long durationInMillis = intermidiateTime - startTime;

        double flyWheelVelocity = 0.0;

        flyWheel.setPower(flyWheelPower);
        //telemetry.addData("Flywheel StartTime: ", startTime);
        while (durationInMillis <= runForMS) {
            intermidiateTime = System.currentTimeMillis();
            durationInMillis = intermidiateTime - startTime;
            flyWheelVelocity = flyWheel.getVelocity();
            //telemetry.addData("Flywheel Intermidiate Time (ms): ", durationInMillis);
            telemetry.addData("Flywheel Velocity: " + flyWheelVelocity + " in time(ms): ", durationInMillis);
            sleepThread(1000);
        }
        endTime = System.currentTimeMillis();
        durationInMillis = endTime - startTime;

        telemetry.addData("Flywheel Total Time: ", durationInMillis);

        flyWheel.stop();
    }

    public static boolean waitForMotor(DcMotorEx dcWheel, long timeoutMs, double ticksPerSecond) {
        boolean ret = false;
        long sleepInMs = 200;

        for (long i = 0; i < timeoutMs; i++) {
            if (dcWheel.getVelocity() >= Math.abs(ticksPerSecond)) {
                ret = true;
                break;
            }

            try {
                Thread.sleep(sleepInMs);
            } catch (InterruptedException e) {
            }

            i = i + sleepInMs;
        }

        return ret;
    }

    public static boolean isObjectDetected(DistanceSensor channelSensor, Telemetry telemetry) {
        boolean ret = false;

        // Get distance reading from 2M sensor
        double dDistance = channelSensor.getDistance(DistanceUnit.INCH);
        //telemetry.addData("distanceSensor - ",dDistance);
        //telemetry.update();
        // Check if distance is less than 10 cm
        if (dDistance < 14 / 2.54) ret = true;

        return ret;
    }

    public static double getDistance(DistanceSensor distanceSensor, Telemetry telemetry) {

        double dDistance = distanceSensor.getDistance(DistanceUnit.INCH);

        return dDistance;
    }


    public static Double getRequiredFlyWheelPower(double distanceInInchFromAprilTag) {
        Double doubleDesiredFlyWheelPower = 0.45;

        if (distanceInInchFromAprilTag >= 25 && distanceInInchFromAprilTag <= 50) {
            doubleDesiredFlyWheelPower = 0.45;
        } else if (distanceInInchFromAprilTag > 50 && distanceInInchFromAprilTag <= 70) {
            doubleDesiredFlyWheelPower = 0.55;
        } else if (distanceInInchFromAprilTag > 70 && distanceInInchFromAprilTag <= 85) {
            doubleDesiredFlyWheelPower = 0.65;
        }

        return doubleDesiredFlyWheelPower;
    }

    public static Integer getRequiredFlyWheelVelocity(Double distanceInInchFromAprilTag) {
        Integer flyWheelVelocityTunning = 100;
        Integer integerDesiredFlyWheelVelocity;

        /* measurement data (velocity_rpm vs distance_inch)
        [950 1000 1100 1200 1300 1400] rpm
        [37  43.5    51 54.5 67.5 77] - inch

        velocity = 10.25*distance + 587.4, R^2=0.992

        minmum shooting distance is 37 inch
         */

        integerDesiredFlyWheelVelocity = (int) Math.max(1050 + flyWheelVelocityTunning, Math.ceil(10.25 * distanceInInchFromAprilTag + 587.4));


        /*
        if (distanceInInchFromAprilTag >= 25 && distanceInInchFromAprilTag <= 50){
            integerDesiredFlyWheelVelocity = 1200;
        }
        else if (distanceInInchFromAprilTag>50 && distanceInInchFromAprilTag<=70){
            integerDesiredFlyWheelVelocity = 1300;
        }
        else if (distanceInInchFromAprilTag>70 && distanceInInchFromAprilTag<=85){
            integerDesiredFlyWheelVelocity = 1400;
        }

         */
        return integerDesiredFlyWheelVelocity;
    }


    /*
    public static int getRequiredFlyWheelVelocityRequiredForDistance(Double distanceInInchFromAprilTag){
        int desiredFlyWheelVelocity = FlyWheel.FLYWHEEL_SHOOTING_VELOCITY;

        Double doubleDesiredFlyWheelPower = 0.004*(distanceInInchFromAprilTag)+0.399;
        Double doubleDesiredFlyWheelVelocity = (doubleDesiredFlyWheelPower * 1500)/0.65;
        doubleDesiredFlyWheelPower.compareTo()


         desiredFlyWheelVelocity = doubleDesiredFlyWheelVelocity.intValue();


        return desiredFlyWheelVelocity;
    }

     */


    public static void threadSleep(int millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            return;
        }
    }

    public static void prepareFlyWheelToShoot(FlyWheel flyWheel, Kicker kicker, Intake intake, Double distance, Telemetry telemetry) {

        intake.setIntakePower(0.5); //reduce intake power to avoid jam
        kicker.setPosition(Kicker.gateClose); // close gate to clear ball for ramping up flywheel
        sleepThread(400);       // wait for kicker to close

    }

    public static void prepareFlyWheelToIntake(FlyWheel flyWheel, Kicker kicker, Intake intake, Flipper flipper, Telemetry telemetry) {
        flyWheel.stop();
        kicker.setGatePosition(Kicker.GATE_CLOSE);
        flipper.resetFlipper();
        Util.waitForFlyWheelStopVelocity(flyWheel, 200, 3000, telemetry);
        kicker.setPosition(Kicker.gateIntake);
        intake.startIntake();
    }

    /**
     * Attempts to clear a jam by cycling the kicker gate rapidly.
     * Moves kicker between intake and close positions to dislodge stuck game pieces.
     * Also stops the flywheel during clearing to prevent further jamming.
     */
    public static void clearJam(FlyWheel flyWheel, Kicker kicker, int numCycles, Telemetry telemetry) {
        final int CYCLE_DELAY_MS = 200;  // Time for each position change
        final long STOP_VELOCITY = 200;   // Target velocity for flywheel to be considered stopped
        final double STOP_TIMEOUT_MS = 2000;  // Maximum time to wait for flywheel to stop

        telemetry.addData("Jam Detected", "Attempting to clear...");
        telemetry.update();

        // Stop flywheel using waitForFlyWheelStopVelocity to actively brake
        waitForFlyWheelStopVelocity(flyWheel, STOP_VELOCITY, STOP_TIMEOUT_MS, telemetry);

        for (int i = 0; i < numCycles; i++) {
            // Move to intake position to pull/push jam
            kicker.setPosition(Kicker.gateIntake);
            threadSleep(CYCLE_DELAY_MS);

            // Move to close position
            kicker.setPosition(Kicker.gateClose);
            threadSleep(CYCLE_DELAY_MS);

            telemetry.addData("Jam Clear Cycle", String.format("%d / %d", i + 1, numCycles));
            telemetry.update();
        }

        telemetry.addData("Jam Clear", "Complete");
        telemetry.update();
    }

    // Prepares the flywheel and related mechanisms for shooting based on distance to target.
    public static FlyWheelSpinUpResult prepareForShooting(FlyWheel flyWheel, Kicker kicker, Flipper flipper, Intake intake, Double distanceInInchFromAprilTag, Telemetry telemetry) {
        Integer requiredFlyWheelVelocity = Util.getRequiredFlyWheelVelocity(distanceInInchFromAprilTag);
        FlyWheelSpinUpResult result = Util.waitForFlyWheelShootingVelocity(flyWheel, requiredFlyWheelVelocity, 3000, telemetry);

        telemetry.addData("Time to reach velocity (ms) - ", result.durationMs);
        telemetry.addData("Velocity achieved - ", "%.0f RPM (%.0f%% of target)", result.achievedVelocity, (result.achievedVelocity / result.targetVelocity) * 100);

        return result;
    }

    public static void shoot(FlyWheel flyWheel, Kicker kicker, Flipper flipper, Intake intake, Double robotDistanceFromAprilTag, DecodeAprilTag aprilTag, String aprilTagName, Telemetry telemetry) {

        // Configuration constants
        final int NUM_SHOTS = 3;                    // Shoot 4 times for reliability (only 3 required)
        final double INITIAL_FLIPPER_ANGLE = 120;   // Starting flipper angle in degrees
        final double ANGLE_INCREMENT = 30;          // Angle increase per shot (degrees)
        final int KICKER_OPEN_DELAY_MS = 300;       // Wait time for kicker to open
        final int BASE_FLIPPER_DELAY_MS = 150;      // Base wait time for flipper movement
        final int FLIPPER_DELAY_INCREMENT_MS = 50;  // Additional delay per shot
        final int FLIPPER_RESET_DELAY_MS = 200;     // Wait time for flipper to reset
        final int JAM_CLEAR_CYCLES = 3;             // Number of kicker cycles to clear jam
        final int MAX_JAM_CLEAR_ATTEMPTS = 2;       // Maximum attempts to clear jam per shot
        final double DISTANCE_FALLBACK = robotDistanceFromAprilTag;  // Initial distance as fallback

        // Initial preparation: close gate, reduce intake power, wait for positioning
        prepareFlyWheelToShoot(flyWheel, kicker, intake, robotDistanceFromAprilTag, telemetry);

        // Jam detection and clearing BEFORE shooting loop
        // Try to reach target velocity with jam detection
        FlyWheelSpinUpResult spinUpResult = null;
        int attemptNumber = 0;
        final int MAX_TOTAL_ATTEMPTS = MAX_JAM_CLEAR_ATTEMPTS + 1;  // +1 for initial attempt

        while (attemptNumber < MAX_TOTAL_ATTEMPTS) {
            attemptNumber++;

            spinUpResult = prepareForShooting(flyWheel, kicker, flipper, intake, robotDistanceFromAprilTag, telemetry);

            if (spinUpResult.success) {
                // Flywheel reached acceptable velocity (>75% of target)
                telemetry.addData("Flywheel Ready", "%.0f%% of target velocity", (spinUpResult.achievedVelocity / spinUpResult.targetVelocity) * 100);
                telemetry.update();
                break;
            } else {
                // Jam detected - flywheel didn't reach 75% of target
                if (attemptNumber < MAX_TOTAL_ATTEMPTS) {
                    // Still have attempts left - clear jam and retry
                    telemetry.addData("JAM DETECTED", "Attempt %d of %d", attemptNumber, MAX_TOTAL_ATTEMPTS);
                    telemetry.addData("Clearing", "Cycling kicker...");
                    telemetry.update();

                    // Attempt to clear jam by cycling kicker (flywheel will be stopped inside)
                    clearJam(flyWheel, kicker, JAM_CLEAR_CYCLES, telemetry);

                    // Wait a moment before retrying
                    threadSleep(300);
                } else {
                    // Max attempts reached, proceed anyway with reduced performance warning
                    telemetry.addData("WARNING", "Max jam clear attempts reached - proceeding anyway");
                    telemetry.addData("Velocity", "%.0f%% of target", (spinUpResult.achievedVelocity / spinUpResult.targetVelocity) * 100);
                    telemetry.update();
                }
            }
        }

        // Execute shooting sequence - flywheel is already at speed
        for (int shotNumber = 0; shotNumber < NUM_SHOTS; shotNumber++) {

            // Calculate flipper angle for this shot (120°, 150°, 180°, 210°)
            double currentFlipperAngle = INITIAL_FLIPPER_ANGLE + (shotNumber * ANGLE_INCREMENT);

            // Update distance from AprilTag before each shot (robot may have moved)
            Double currentDistance = DISTANCE_FALLBACK;  // Default to initial distance
            if (aprilTag != null && aprilTagName != null) {
                AprilTagPoseFtc currentPose = aprilTag.getCoordinate(aprilTagName);
                if (currentPose != null) {
                    currentDistance = currentPose.range;
                    telemetry.addData("Shot " + (shotNumber + 1) + " Distance", String.format("%.1f inches", currentDistance));
                } else {
                    telemetry.addData("Shot " + (shotNumber + 1) + " Distance", String.format("%.1f inches (fallback - tag lost)", currentDistance));
                }
            } else {
                telemetry.addData("Shot " + (shotNumber + 1) + " Distance", String.format("%.1f inches (fallback - no camera)", currentDistance));
            }
            telemetry.update();

            // Bring flywheel back up to speed (each shot slows it down)
            // No jam detection needed here - jam was already cleared before loop
            prepareForShooting(flyWheel, kicker, flipper, intake, currentDistance, telemetry);

            // Open kicker gate to release game piece into flywheel path
            if (shotNumber==0){ //only open for the 1st shot -- continous shoot
            kicker.setPosition(Kicker.gateShoot);
            threadSleep(KICKER_OPEN_DELAY_MS);}

            // Flip the game piece at the calculated angle
            flipper.turnFlipper(currentFlipperAngle);

            // Wait for flipper to complete movement (time increases with larger angles)
            int flipperWaitTime = BASE_FLIPPER_DELAY_MS + (shotNumber * FLIPPER_DELAY_INCREMENT_MS);
            threadSleep(flipperWaitTime);

            // Close kicker gate to prepare for next shot
      //      kicker.setGatePosition(Kicker.GATE_CLOSE);  //alway open for cont shoot

            // Reset flipper to starting position
            flipper.resetFlipper();
            threadSleep(FLIPPER_RESET_DELAY_MS);
        }

        // Log completion
        telemetry.addData("Shots completed", NUM_SHOTS);
        telemetry.update();

        // Return all mechanisms to intake mode
        prepareFlyWheelToIntake(flyWheel, kicker, intake, flipper, telemetry);
    }

    public enum MovementDirection {
        FORWARD,
        BACKWARD,
        STRAFE_LEFT,
        STRAFE_RIGHT,
        TURN_LEFT,
        TURN_RIGHT
    }


    public static void setSpeed(double min_speed, double max_speed) {
        MIN_SPEED = min_speed;
        MAX_SPEED = max_speed;
    }

    public static void resetToDefaultSpeed() {
        MAX_SPEED = 0.8;
        MIN_SPEED = 0.2;
    }


    static Double MAX_SPEED = 0.8;
    static Double MIN_SPEED = 0.2;

    public static void moveRobot(
            DcMotor leftFront, DcMotor leftBack,
            DcMotor rightFront, DcMotor rightBack,
            GoBildaPinpointDriver odo,
            IMU imu,
            MovementDirection direction,
            double distance,
            double targetAngle,
            Telemetry telemetry) {


        Double P_DRIVE_GAIN = 0.03;
        Double ERROR_RANGE__INCHES = 0.5;
        Double P_YAW_GAIN = 0.10;
        Double ERROR_RANGE_DEGREE = 2.0;
        Integer TIMEOUT_MS = 5000;

        // --- 1. Initialization ---
        odo.resetPosAndIMU();
        odo.setPosX(0.0, DistanceUnit.INCH);
        odo.setPosY(0.0, DistanceUnit.INCH);
        threadSleep(100);

        odo.update();
        Double startX = odo.getPosX(DistanceUnit.INCH);
        Double startY = odo.getPosY(DistanceUnit.INCH);
        Double startYaw = Math.toDegrees(AngleUnit.normalizeRadians(odo.getHeading(AngleUnit.RADIANS)));
        //Double currentAngleRAD = odo.getHeading(AngleUnit.RADIANS);

        ElapsedTime timer = new ElapsedTime();

        // --- We will use different variables depending on direction ---
        Double targetPosition;
        Double currentPosition;
        Double currentHeading;
        Double currentAngle;

        Double errorDistance;
        Double errorAngle;

        Double axialPower;
        Double lateralPower;
        Double yawPower;


        // --- 2. ISOLATED Control Loop ---
        // We run a DIFFERENT loop based on the direction.
        // This prevents the robot from trying to do two things at once.

        switch (direction) {
            case FORWARD:
                targetPosition = startY + distance;

                // --- TELEMETRY 1: BEFORE THE LOOP ---
                // (Checks starting values and calculation)
                // odo.updat();
//                Double odoPosX = odo.getPosX(DistanceUnit.INCH);
//                Double odoPosY = odo.getPosY(DistanceUnit.INCH);

                /*
                telemetry.addData("--- DEBUG: PRE-RUN ---", "");
                telemetry.addData("Direction", direction);
                telemetry.addData("Start X", "%.2f", startX);
                telemetry.addData("Target X", "%.2f", targetPosition);
                telemetry.addData("Distance", "%.2f", distance);
                telemetry.addData("currentPosition", "%.2f", currentPosition);
                telemetry.addData("odoPosX", "%.2f", odoPosX);
                telemetry.addData("odoPosY", "%.2f", odoPosY);
                telemetry.addData("Tgt Heading", "%.2f", targetHeadingRadians);
                telemetry.addData("Start Heading", "%.2f", currentHeading);
                telemetry.update();
                threadSleep(3000);
                */

                timer = new ElapsedTime();
                int loopCounter = 0;
                // Keep looping until we pass the target
                odo.update();
                while (odo.getPosY(DistanceUnit.INCH) < targetPosition && timer.milliseconds() < TIMEOUT_MS) {
                    loopCounter++;
                    odo.update();

                    // --- A. Drive (Axial) Calculation ---
                    currentPosition = odo.getPosY(DistanceUnit.INCH);
                    errorDistance = targetPosition - currentPosition;

                    // P-Controller for Axial
                    axialPower = Range.clip(errorDistance * P_DRIVE_GAIN, MIN_SPEED, MAX_SPEED);


                    // --- B. Heading (Yaw) Calculation ---
                    currentHeading = odo.getHeading(AngleUnit.RADIANS);
                    double headingError = targetAngle - currentHeading;

                    // Handle wraparound (e.g., target 1°, current 359°)
                    while (headingError > Math.PI) headingError -= 2 * Math.PI;
                    while (headingError < -Math.PI) headingError += 2 * Math.PI;

                    // P-Controller for Yaw
                    yawPower = Range.clip(headingError * P_YAW_GAIN, -MAX_SPEED, MAX_SPEED);

                    // Apply *only* axial (forward) power
                    setMotorPower(leftFront, leftBack, rightFront, rightBack, axialPower, 0, yawPower);


                    // --- TELEMETRY 2: INSIDE THE LOOP ---
                    // (This is existing, correct telemetry)
                    odo.update();

                }
                //Break after the movement.
                setMotorPower(leftFront, leftBack, rightFront, rightBack, 0, 0, 0);
                //threadSleep(5000);
                break;

            case BACKWARD:
                targetPosition = startY - distance;

                // Keep looping until we pass the target
                while (odo.getPosY(DistanceUnit.INCH) > targetPosition && timer.milliseconds() < TIMEOUT_MS) {
                    odo.update();
                    currentPosition = odo.getPosY(DistanceUnit.INCH);
                    errorDistance = targetPosition - currentPosition; // Error will be negative

                    // Simple P-Controller. Note: errorDistance * gain will be negative.
                    axialPower = Range.clip(errorDistance * P_DRIVE_GAIN, -MAX_SPEED, -MIN_SPEED);

                    // Apply *only* axial (backward) power
                    setMotorPower(leftFront, leftBack, rightFront, rightBack, axialPower, 0, 0);

                    telemetry.addData("Moving", direction);
                    telemetry.addData("Target", "%.2f", targetPosition);
                    telemetry.addData("Current", "%.2f", currentPosition);
                    telemetry.addData("Power", "%.2f", axialPower);
                    telemetry.update();
                    odo.update();
                }
                break;

            case STRAFE_RIGHT:
                targetPosition = startX + distance;

                // Keep looping until we pass the target
                while (odo.getPosX(DistanceUnit.INCH) < targetPosition && timer.milliseconds() < TIMEOUT_MS) {
                    odo.update();
                    currentPosition = odo.getPosX(DistanceUnit.INCH);
                    errorDistance = targetPosition - currentPosition;

                    // Simple P-Controller
                    lateralPower = Range.clip(errorDistance * P_DRIVE_GAIN, MIN_SPEED, MAX_SPEED);

                    // Apply *only* lateral (strafe) power
                    setMotorPower(leftFront, leftBack, rightFront, rightBack, 0, -lateralPower, 0);

                    telemetry.addData("Moving", direction);
                    telemetry.addData("Target", "%.2f", targetPosition);
                    telemetry.addData("Current", "%.2f", currentPosition);
                    telemetry.addData("Power", "%.2f", lateralPower);
                    telemetry.update();
                    odo.update();
                }
                break;

            case STRAFE_LEFT:
                targetPosition = startX - distance;

                // Keep looping until we pass the target
                while (odo.getPosX(DistanceUnit.INCH) > targetPosition && timer.milliseconds() < TIMEOUT_MS) {
                    odo.update();
                    currentPosition = odo.getPosX(DistanceUnit.INCH);
                    errorDistance = targetPosition - currentPosition; // Error will be negative

                    // Simple P-Controller
                    lateralPower = Range.clip(errorDistance * P_DRIVE_GAIN, -MAX_SPEED, -MIN_SPEED);

                    // Apply *only* lateral (strafe) power
                    setMotorPower(leftFront, leftBack, rightFront, rightBack, 0, -lateralPower, 0);

                    telemetry.addData("Moving", direction);
                    telemetry.addData("Target", "%.2f", targetPosition);
                    telemetry.addData("Current", "%.2f", currentPosition);
                    telemetry.addData("Power", "%.2f", lateralPower);
                    telemetry.update();

                    odo.update();
                }
                break;
            case TURN_LEFT:
                odo.update();
                currentAngle = Math.toDegrees(AngleUnit.normalizeRadians(odo.getHeading(AngleUnit.RADIANS)));
                errorAngle = AngleUnit.normalizeDegrees(targetAngle - currentAngle);

                telemetry.addData("Target Angle", "%.2f", targetAngle);
                telemetry.addData("Current Angle", "%.2f", currentAngle);
                telemetry.addData("Error Angle", "%.2f", errorAngle);
                telemetry.update();
                sleepThread(1000);

                //while(Math.abs(errorAngle) < ERROR_RANGE_DEGREE && timer.milliseconds() < TIMEOUT_MS) {

                while (currentAngle < targetAngle && timer.milliseconds() < TIMEOUT_MS) {

                    Double turnPower = ((Math.abs(errorAngle) / 180) * (MAX_SPEED - MIN_SPEED) + MIN_SPEED) * Math.signum(errorAngle) * -1;

                    telemetry.addData("Target Angle", "%.2f", targetAngle);
                    telemetry.addData("Current Angle", "%.2f", currentAngle);
                    telemetry.addData("Error Angle", "%.2f", errorAngle);
                    telemetry.addData("Turn Power", "%.2f", turnPower);
                    telemetry.update();

                    //Half turn speed
                    setMotorPower(leftFront, leftBack, rightFront, rightBack, 0, 0, -turnPower);

                    odo.update();
                    //currentAngleRAD = odo.getHeading(AngleUnit.RADIANS);
                    currentAngle = Math.toDegrees(AngleUnit.normalizeRadians(odo.getHeading(AngleUnit.RADIANS)));
                    errorAngle = AngleUnit.normalizeDegrees(targetAngle - currentAngle);

                    if (Math.abs(errorAngle) < ERROR_RANGE_DEGREE) {
                        Util.setMotorPower(leftFront, leftBack, rightFront, rightBack, 0, 0, 0);
                        break;
                    }
                }
                break;
            case TURN_RIGHT:
                odo.update();
                currentAngle = Math.toDegrees(AngleUnit.normalizeRadians(odo.getHeading(AngleUnit.RADIANS)));
                errorAngle = AngleUnit.normalizeDegrees(targetAngle - currentAngle);

                telemetry.addData("Target Angle", "%.2f", targetAngle);
                telemetry.addData("Current Angle", "%.2f", currentAngle);
                telemetry.addData("Error Angle", "%.2f", errorAngle);
                telemetry.update();
                sleepThread(1000);

                //while(Math.abs(errorAngle) < ERROR_RANGE_DEGREE && timer.milliseconds() < TIMEOUT_MS) {

                while (currentAngle > targetAngle && timer.milliseconds() < TIMEOUT_MS) {

                    Double turnPower = ((Math.abs(errorAngle) / 180) * (MAX_SPEED - MIN_SPEED) + MIN_SPEED) * Math.signum(errorAngle) * -1;

                    telemetry.addData("Target Angle", "%.2f", targetAngle);
                    telemetry.addData("Current Angle", "%.2f", currentAngle);
                    telemetry.addData("Error Angle", "%.2f", errorAngle);
                    telemetry.addData("Turn Power", "%.2f", turnPower);
                    telemetry.update();

                    //Half turn speed
                    setMotorPower(leftFront, leftBack, rightFront, rightBack, 0, 0, -turnPower);

                    odo.update();
                    //currentAngleRAD = odo.getHeading(AngleUnit.RADIANS);
                    currentAngle = Math.toDegrees(AngleUnit.normalizeRadians(odo.getHeading(AngleUnit.RADIANS)));
                    errorAngle = AngleUnit.normalizeDegrees(targetAngle - currentAngle);

                    if (Math.abs(errorAngle) < ERROR_RANGE_DEGREE) {
                        Util.setMotorPower(leftFront, leftBack, rightFront, rightBack, 0, 0, 0);
                        break;
                    }
                }
                break;

        }

        // --- 3. Stop Motors ---
        setMotorPower(leftFront, leftBack, rightFront, rightBack, 0, 0, 0);

//        telemetry.addData("Status", "Movement Complete");
//        telemetry.addData("Final X", "%.2f", odo.getPosX(DistanceUnit.INCH));
//        telemetry.addData("Final Y", "%.2f", odo.getPosY(DistanceUnit.INCH));
//        telemetry.update();
    }

    /**
     * Helper method to apply power to all four motors using Mecanum logic.
     * (This is your original 'drive' logic, simplified)
     *
     * @param axialPower   Forward/Backward power (+/-)
     * @param lateralPower Strafe Left/Right power (+/-)
     * @param yawPower     Rotation power (+/-)
     */
    public static void setMotorPower(
            DcMotor leftFront, DcMotor leftBack,
            DcMotor rightFront, DcMotor rightBack,
            double axialPower, double lateralPower, double yawPower) {

        double leftFrontPower = axialPower - lateralPower - yawPower;
        double rightFrontPower = axialPower + lateralPower + yawPower;
        double leftBackPower = axialPower + lateralPower - yawPower;
        double rightBackPower = axialPower - lateralPower + yawPower;

        // Normalize (same as before)
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }

    public static AlignmentResult autoAlignWithAprilTag(
            LinearOpMode chassis,
            DecodeAprilTag aprilTag,
            String aprilTagName,
            Chassis chassisInstance,
            Telemetry telemetry) {

        // --- Configuration Constants ---
        final double BEARING_TOLERANCE_DEG = 10.0;  // Acceptable bearing error
        final double YAW_TOLERANCE_DEG = 10.0;      // Acceptable yaw error
        final int DETECTION_TIMEOUT_MS = 3000;       // Total AprilTag detection timeout (3 seconds)
        final int BEARING_ALIGNMENT_TIMEOUT_MS = 800; // Bearing alignment timeout
        final int YAW_ALIGNMENT_TIMEOUT_MS = 800;   // Yaw alignment timeout
        final double TURN_POWER = 0.3;               // Power for turning (bearing correction)
        final double STRAFE_POWER = 0.15;             // Power for strafing (yaw correction)
        final double P_TURN_GAIN = 0.02;             // Proportional gain for bearing control (0.02 = 0.6 power at 30° error)
        final double P_STRAFE_GAIN = 0.015;          // Proportional gain for yaw control
        final double SCAN_TURN_POWER = 0.2;          // Power for scanning turn
        final double FORWARD_SCAN_POWER = 0.15;      // Slow forward movement during scan
        final int CONTROL_LOOP_MS = 50;              // Control loop period
        final int SETTLE_TIME_MS = 150;              // Time to let robot settle after stopping
        final int SCAN_SWEEP_MS = 1000;               // Time to sweep 90 degrees (approximate)

        ElapsedTime timer = new ElapsedTime();

        // --- Step 1: Robust AprilTag Detection with Scanning ---
        // Camera is rear-facing and AprilTag is mounted HIGH (on basket)
        // If robot is too close, camera can't see the high tag
        // Strategy: Scan ±90° while moving FORWARD to increase viewing distance
        // Sequences: 1) Check in place, 2) Scan right+forward, 3) Scan left+forward, 4) Center+forward
        telemetry.addData("Auto Align", "Searching for AprilTag...");
        telemetry.update();

        timer.reset();
        boolean tagFound = false;

        // Strategy 1: Check current position (200ms)
        double scanStartTime = timer.milliseconds();
        while (chassis.opModeIsActive() && (timer.milliseconds() - scanStartTime) < 200) {
            if (aprilTag.findAprilTag(aprilTagName)) {
                tagFound = true;
                telemetry.addData("Auto Align", "Tag found at current position");
                telemetry.update();
                break;
            }
            sleepThread(CONTROL_LOOP_MS);
        }

        if (!tagFound && timer.milliseconds() < DETECTION_TIMEOUT_MS) {
            // Strategy 2: Scan right (+90 degrees) while moving FORWARD
            // Camera is rear-facing and tag is HIGH, so moving FORWARD increases viewing distance
            // This moves robot AWAY from the high tag behind it, giving camera better upward angle
            telemetry.addData("Auto Align", "Scanning right and moving forward...");
            telemetry.update();

            scanStartTime = timer.milliseconds();
            setMotorPower(chassisInstance.frontLeftDrive, chassisInstance.backLeftDrive,
                    chassisInstance.frontRightDrive, chassisInstance.backRightDrive,
                    FORWARD_SCAN_POWER, 0, -SCAN_TURN_POWER); // Slow forward + turn right (negative yaw = turn right)

            while (chassis.opModeIsActive() && (timer.milliseconds() - scanStartTime) < SCAN_SWEEP_MS
                    && timer.milliseconds() < DETECTION_TIMEOUT_MS) {
                if (aprilTag.findAprilTag(aprilTagName)) {
                    tagFound = true;
                    chassisInstance.stopRobot();
                    telemetry.addData("Auto Align", "Tag found while scanning right");
                    telemetry.update();
                    break;
                }
                sleepThread(CONTROL_LOOP_MS);
            }
            chassisInstance.stopRobot();
            sleepThread(100); // Settle time
        }

        if (!tagFound && timer.milliseconds() < DETECTION_TIMEOUT_MS) {
            // Strategy 3: Scan left (-180 degrees) while moving FORWARD
            // Continue moving forward to increase distance from high tag
            telemetry.addData("Auto Align", "Scanning left and moving forward...");
            telemetry.update();

            scanStartTime = timer.milliseconds();
            setMotorPower(chassisInstance.frontLeftDrive, chassisInstance.backLeftDrive,
                    chassisInstance.frontRightDrive, chassisInstance.backRightDrive,
                    FORWARD_SCAN_POWER, 0, SCAN_TURN_POWER); // Slow forward + turn left (positive yaw = turn left)

            while (chassis.opModeIsActive() && (timer.milliseconds() - scanStartTime) < (SCAN_SWEEP_MS * 2)
                    && timer.milliseconds() < DETECTION_TIMEOUT_MS) {
                if (aprilTag.findAprilTag(aprilTagName)) {
                    tagFound = true;
                    chassisInstance.stopRobot();
                    telemetry.addData("Auto Align", "Tag found while scanning left");
                    telemetry.update();
                    break;
                }
                sleepThread(CONTROL_LOOP_MS);
            }
            chassisInstance.stopRobot();
            sleepThread(100); // Settle time
        }

        if (!tagFound && timer.milliseconds() < DETECTION_TIMEOUT_MS) {
            // Strategy 4: Return to approximate center while moving FORWARD
            // Final forward movement while returning to center orientation
            telemetry.addData("Auto Align", "Returning to center and moving forward...");
            telemetry.update();

            setMotorPower(chassisInstance.frontLeftDrive, chassisInstance.backLeftDrive,
                    chassisInstance.frontRightDrive, chassisInstance.backRightDrive,
                    FORWARD_SCAN_POWER, 0, -SCAN_TURN_POWER); // Turn right to return to center
            sleepThread(SCAN_SWEEP_MS);

            if (timer.milliseconds() < DETECTION_TIMEOUT_MS) {

                chassisInstance.stopRobot();
                sleepThread(100);

                // One more quick check at center position
                if (aprilTag.findAprilTag(aprilTagName)) {
                    tagFound = true;
                    telemetry.addData("Auto Align", "Tag found at center");
                    telemetry.update();
                }
            } else {
                chassisInstance.stopRobot(); // Stop immediately if timeout
            }
        }


            if (!tagFound) {
                telemetry.addData("Auto Align", "FAILED - AprilTag not detected after scanning");
                telemetry.update();
                return new AlignmentResult(false, 0, 0, 0);
            }

            // --- Step 2: Align Bearing (Turn robot to center tag in camera) ---
            telemetry.addData("Auto Align", "Aligning bearing...");
            telemetry.update();

            timer.reset();
            boolean bearingAligned = false;

            while (chassis.opModeIsActive() && timer.milliseconds() < BEARING_ALIGNMENT_TIMEOUT_MS) {
                AprilTagPoseFtc pose = aprilTag.getCoordinate(aprilTagName);

                if (pose == null) {
                    telemetry.addData("Auto Align", "Lost AprilTag during bearing alignment");
                    telemetry.update();
                    chassisInstance.stopRobot();
                    return new AlignmentResult(false, 0, 0, 0);
                }

                double bearing = pose.bearing;

                if (Math.abs(bearing) < BEARING_TOLERANCE_DEG) {
                    bearingAligned = true;
                    break;
                }

                // Turn to correct bearing with proportional control
                // CAMERA IS REAR-FACING, so bearing is relative to BACK of robot
                // Positive bearing = tag to the right of camera (left side of robot front)
                // In mecanum drive: POSITIVE yaw → RIGHT turn (clockwise), NEGATIVE yaw → LEFT turn (counter-clockwise)
                // Therefore: Positive bearing needs NEGATIVE yaw (turn left), Negative bearing needs POSITIVE yaw (turn right)
                // Use proportional control to avoid oscillation: turnPower = -bearing * gain
                double turnPower = -Range.clip(-bearing * P_TURN_GAIN, -TURN_POWER, TURN_POWER);//
                setMotorPower(chassisInstance.frontLeftDrive, chassisInstance.backLeftDrive,
                        chassisInstance.frontRightDrive, chassisInstance.backRightDrive,
                        0, 0, turnPower);

                telemetry.addData("Bearing", "%.1f°", bearing);
                telemetry.update();

                sleepThread(CONTROL_LOOP_MS);
            }

            chassisInstance.stopRobot();
            sleepThread(SETTLE_TIME_MS); // Allow robot to fully stop before checking success

            if (!bearingAligned) {
                telemetry.addData("Auto Align", "FAILED - Bearing alignment timeout");
                telemetry.update();
                return new AlignmentResult(false, 0, 0, 0);
            }

            // --- Step 3: Align Yaw (Strafe to be perpendicular to tag) ---
            telemetry.addData("Auto Align", "Aligning yaw...");
            telemetry.update();

            timer.reset();
            boolean yawAligned = false;

            while (chassis.opModeIsActive() && timer.milliseconds() < YAW_ALIGNMENT_TIMEOUT_MS) {
                AprilTagPoseFtc pose = aprilTag.getCoordinate(aprilTagName);

                if (pose == null) {
                    telemetry.addData("Auto Align", "Lost AprilTag during yaw alignment");
                    telemetry.update();
                    chassisInstance.stopRobot();
                    return new AlignmentResult(false, 0, 0, 0);
                }

                double yaw = pose.yaw;

                if (Math.abs(yaw) < YAW_TOLERANCE_DEG) {
                    yawAligned = true;
                    break;
                }

                // Strafe and turning to correct yaw with proportional control
                // CAMERA IS REAR-FACING, yaw indicates tag surface angle relative to camera
                // Positive yaw = tag surface angled, need to strafe LEFT (from robot front perspective)
                // In mecanum: negative y = strafe left, positive y = strafe right
                // Use proportional control to avoid oscillation: strafePower = -yaw * gain
                double strafePower = -Range.clip(-yaw * P_STRAFE_GAIN, -STRAFE_POWER, STRAFE_POWER);
                setMotorPower(chassisInstance.frontLeftDrive, chassisInstance.backLeftDrive,
                        chassisInstance.frontRightDrive, chassisInstance.backRightDrive,
                        0, strafePower, strafePower/2);

                telemetry.addData("Yaw", "%.1f°", yaw);
                telemetry.update();

                sleepThread(CONTROL_LOOP_MS);
            }

            chassisInstance.stopRobot();

            sleepThread(SETTLE_TIME_MS); // Allow robot to fully stop before checking success

            if (!yawAligned) {
                telemetry.addData("Auto Align", "FAILED - Yaw alignment timeout");
                telemetry.update();
                return new AlignmentResult(false, 0, 0, 0);
            }

            // --- Step 4: Get Final Distance and Report Success ---
            AprilTagPoseFtc finalPose = aprilTag.getCoordinate(aprilTagName);

            if (finalPose == null) {
                telemetry.addData("Auto Align", "Lost AprilTag after alignment");
                telemetry.update();
                return new AlignmentResult(false, 0, 0, 0);
            }

            double distance = finalPose.range;
            double bearing = finalPose.bearing;
            double yaw = finalPose.yaw;

            telemetry.addData("Auto Align", "SUCCESS - Ready to shoot!");
            telemetry.addData("Distance", "%.1f inches", distance);
            telemetry.addData("Bearing", "%.1f°", bearing);
            telemetry.addData("Yaw", "%.1f°", yaw);
            telemetry.update();

            return new AlignmentResult(true, distance, bearing, yaw);
        }




}
