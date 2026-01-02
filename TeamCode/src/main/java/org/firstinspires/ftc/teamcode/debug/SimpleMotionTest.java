package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.calibration.RobotConstants;
import org.firstinspires.ftc.teamcode.external.gobilida.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.motion.FieldPose;
import org.firstinspires.ftc.teamcode.motion.FieldPositions;
import org.firstinspires.ftc.teamcode.motion.MotionConfig;
import org.firstinspires.ftc.teamcode.motion.MotionExecutor;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.text.FieldPosition;


/**
 * SIMPLE ODOMETRY TEST
 * 
 * Tests raw Pinpoint odometry readings during basic motion:
 * 1. Record initial position
 * 2. Move forward 24 inches
 * 3. Rotate to 90 degrees
 * 4. Record final position
 * 5. Display initial vs final readings
 * 
 * NO calibration, NO validation - just raw Pinpoint data
 */
@Autonomous(name = "Simple Motion Test -2", group = "Debug")
public class SimpleMotionTest extends LinearOpMode {
    
    private MotionExecutor motionExecutor;
    private Pose2D initialPose;
    private Pose2D finalPose;
    
    @Override
    public void runOpMode() {
        
        // ========== INITIALIZATION ==========
        
        telemetry.addLine("Initializing Simple Odometry Test...");
        telemetry.update();
        
        // Initialize motors
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeftDrive");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRightDrive");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeftDrive");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRightDrive");
        
        // Initialize odometry
        GoBildaPinpointDriver odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        
        // Initialize motion executor with simplified odometry
        motionExecutor = new MotionExecutor(frontLeft, frontRight, backLeft, backRight, odometry);

        // Set control mode (default is HYBRID)
        // Options: PURE_FEEDFORWARD, PURE_FEEDBACK, HYBRID
        //motionExecutor.setControlMode(MotionExecutor.ControlMode.PURE_FEEDFORWARD);
        motionExecutor.setControlMode(MotionExecutor.ControlMode.PURE_FEEDBACK);

        // set your robot reference point start point on field
        //  FieldPositions.REFERENCE_FIELD_ORIGIN

     /*   motionExecutor.resetToFieldOrigin( new Pose2D(
                DistanceUnit.INCH,
                FieldPositions.FIELD_ORIGIN_X,
                FieldPositions.FIELD_ORIGIN_Y,
                AngleUnit.DEGREES,
                FieldPositions.FIELD_ORIGIN_HEADING
        ));

        */

      motionExecutor.resetToFieldOrigin(); // set to field origin


        
        telemetry.addLine("✅ Systems Ready!");
        telemetry.addLine("");
        telemetry.addLine("Test Plan:");
        telemetry.addLine("1. Record initial position");
        telemetry.addLine("2. Move forward 24 inches");
        telemetry.addLine("3. Rotate to 90 degrees");
        telemetry.addLine("4. Compare initial vs final");
        telemetry.addLine("");
        telemetry.addLine("Press START to begin test");
        telemetry.update();
        
        waitForStart();
        
        if (opModeIsActive()) {
            runTest();
        }
    }
    
    private void runTest() {
        
        // ========== STEP 1: RECORD INITIAL POSITION ==========
        
        telemetry.addLine("🔄 Step 1: Recording initial position...");
        telemetry.update();
        
        // Update odometry and get initial reading

        motionExecutor.updateState();
        initialPose = motionExecutor.getMotionState().getCurrentPose();
        
        telemetry.addLine("✅ Initial position recorded:");
        telemetry.addData("Initial X", "%.2f inches", initialPose.getX(DistanceUnit.INCH));
        telemetry.addData("Initial Y", "%.2f inches", initialPose.getY(DistanceUnit.INCH));
        telemetry.addData("Initial Heading", "%.1f degrees", initialPose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("intake x", "%.2f inches", RobotConstants.INTAKE_POINT.x);
        telemetry.addData("intake y", "%.2f inches", RobotConstants.INTAKE_POINT.y);
        telemetry.addData("intake odo x", "%.2f inches", RobotConstants.ODOMETRY_SENSOR.x);
        telemetry.addData("intake odo y", "%.2f inches", RobotConstants.ODOMETRY_SENSOR.y);
        telemetry.addData("ref x", "%.2f inches", FieldPositions.getActiveReferencePoint().x);
        telemetry.addData("ref y", "%.2f inches", FieldPositions.getActiveReferencePoint().y);



        telemetry.addLine("");
        telemetry.update();
        
        sleep(2000); // Pause to read
        
        // ========== STEP 2: MOVE FORWARD 24 INCHES & rotate ==========
        
        telemetry.addLine("🔄 Step 2: Moving forward 24 inches...");
        telemetry.update();
        
        // Calculate target position: move 24 inches forward from current position
        double targetX = initialPose.getX(DistanceUnit.INCH) + 72;
        double targetY = initialPose.getY(DistanceUnit.INCH)+43;
        double targetHeading = initialPose.getHeading(AngleUnit.DEGREES) + 90;
        
        // Execute motion
        motionExecutor.moveToPose(targetX, targetY, targetHeading, 20);
        
        telemetry.addLine("✅ Forward motion complete");
        telemetry.update();
        
        sleep(1000); // Pause for stability
        

        // ========== STEP 3: RECORD FINAL POSITION ==========
        
        telemetry.addLine("🔄 Step 4: Recording final position...");
        telemetry.update();

        // Update odometry and get final reading
        motionExecutor.updateState();
        finalPose = motionExecutor.getMotionState().getCurrentPose();
        
        // ========== DISPLAY RESULTS ==========
        
        displayResults();
        
        // Keep displaying results until stop is pressed
        while (opModeIsActive()) {
            displayResults();
            sleep(100);
        }
    }
    
    private void displayResults() {
        telemetry.clear();
        telemetry.addLine("🎯 ODOMETRY TEST RESULTS");
        telemetry.addLine("========================");
        telemetry.addLine("");
        
        telemetry.addLine("📍 INITIAL POSITION:");
        telemetry.addData("  X", "%.2f inches", initialPose.getX(DistanceUnit.INCH));
        telemetry.addData("  Y", "%.2f inches", initialPose.getY(DistanceUnit.INCH));
        telemetry.addData("  Heading", "%.1f degrees", initialPose.getHeading(AngleUnit.DEGREES));
        telemetry.addLine("");
        
        telemetry.addLine("📍 FINAL POSITION:");
        telemetry.addData("  X", "%.2f inches", finalPose.getX(DistanceUnit.INCH));
        telemetry.addData("  Y", "%.2f inches", finalPose.getY(DistanceUnit.INCH));
        telemetry.addData("  Heading", "%.1f degrees", finalPose.getHeading(AngleUnit.DEGREES));
        telemetry.addLine("");
        
        telemetry.addLine("📊 CALCULATED CHANGES:");
        double deltaX = finalPose.getX(DistanceUnit.INCH) - initialPose.getX(DistanceUnit.INCH);
        double deltaY = finalPose.getY(DistanceUnit.INCH) - initialPose.getY(DistanceUnit.INCH);
        double deltaHeading = finalPose.getHeading(AngleUnit.DEGREES) - initialPose.getHeading(AngleUnit.DEGREES);
        
        // Handle heading wraparound
        while (deltaHeading > 180) deltaHeading -= 360;
        while (deltaHeading <= -180) deltaHeading += 360;
        
        telemetry.addData("  ΔX", "%.2f inches (expected: ~24.0)", deltaX);
        telemetry.addData("  ΔY", "%.2f inches (expected: ~0.0)", deltaY);
        telemetry.addData("  ΔHeading", "%.1f degrees (expected: ~90.0)", deltaHeading);
        telemetry.addLine("");
        
        telemetry.addLine("📏 ERROR ANALYSIS:");
        double xError = Math.abs(deltaX - 24.0);
        double yError = Math.abs(deltaY - 0.0);
        double headingError = Math.abs(deltaHeading - 90.0);
        
        telemetry.addData("  X Error", "%.2f inches", xError);
        telemetry.addData("  Y Error", "%.2f inches", yError);
        telemetry.addData("  Heading Error", "%.1f degrees", headingError);
        
        telemetry.update();
    }
}
