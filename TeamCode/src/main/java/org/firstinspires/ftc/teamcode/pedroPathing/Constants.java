package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.ForwardVelocityTuner;

public class Constants {

    // Constants for the Follower system, which controls the robot's movement and heading
    public static FollowerConstants followerConstants = new FollowerConstants()
            // Robot's mass in kilograms, used for motion calculations
            .mass(9.5)
            // Acceleration when no power is applied in the forward direction
            .forwardZeroPowerAcceleration(-33.71561842790841)
            // Acceleration when no power is applied in the lateral direction
            .lateralZeroPowerAcceleration(-58.92532697943539)
            // PIDF coefficients for translational movement (primary control loop)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.15, 0, 0.02, 0.025))
            // PIDF coefficients for secondary translational movement (fine-tuning)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.025))
            // PIDF coefficients for secondary heading adjustments (fine-tuning heading control)
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.01, 0.02))
            // PIDF coefficients for secondary drive adjustments (fine-tuning drive control)
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.002, 0, 0.00002, 0.6, 0.1))
            // PIDF coefficients for heading control (primary control loop)
            .headingPIDFCoefficients(new PIDFCoefficients(3, 0, 0, 0.02))
            // PIDF coefficients for drive control (primary control loop)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.009, 0.0, 0.0001, 0.6, 0.02))
            // Scaling factor for centripetal force calculations during turns
            .centripetalScaling(0.0008)
            // Enable secondary translational PIDF control
            .useSecondaryTranslationalPIDF(true)
            // Enable secondary heading PIDF control
            .useSecondaryHeadingPIDF(false)
            // Enable secondary drive PIDF control
            .useSecondaryDrivePIDF(true);

    // Constants for the Mecanum drivetrain configuration
    public static MecanumConstants driveConstants = new MecanumConstants()
            // Maximum motor power allowed
            .maxPower(1)
            // Names of the motors in the hardware map
            .rightFrontMotorName("frontRightDrive")
            .rightRearMotorName("backRightDrive")
            .leftRearMotorName("backLeftDrive")
            .leftFrontMotorName("frontLeftDrive")
            // Motor directions for proper movement
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            // Maximum velocity in the X (forward) direction in inches per second
            .xVelocity(65.27665265901823)
            // Maximum velocity in the Y (strafe) direction in inches per second
            .yVelocity(52.91157784048967);

    // Constants for the odometry localization system
    public static PinpointConstants localizerConstants = new PinpointConstants()
            // Y-offset of the forward odometry pod from the robot's center
            //convert mm to inches
            .forwardPodY((12/25.4))
            // X-offset of the strafe odometry pod from the robot's center
            //convert mm to inches
            .strafePodX(-(24/25.4))
            // Unit of measurement for distances (inches)
            .distanceUnit(DistanceUnit.INCH)
            // Name of the odometry hardware in the hardware map
            .hardwareMapName("odo")
            // Encoder resolution for the odometry pods
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            // Direction of the forward encoder
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            // Direction of the strafe encoder
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    // Path constraints for the robot's movement
    public static PathConstraints pathConstraints = new PathConstraints(
            0.99, // Maximum velocity scaling factor (percentage of max velocity)
            100,  // Maximum acceleration in inches per second squared
            1.1,  // Maximum angular velocity in radians per second
            0.7// Maximum angular acceleration in radians per second squared
    );
    
    // Method to create and configure the Follower object
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints) // Apply path constraints
                .mecanumDrivetrain(driveConstants) // Configure drivetrain
                .pinpointLocalizer(localizerConstants) // Configure localization
                .build();
    }
}