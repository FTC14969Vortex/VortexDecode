package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.motion.FieldPositions;
import org.firstinspires.ftc.teamcode.motion.FieldPose;
import org.firstinspires.ftc.teamcode.subsystems.BaseMotion;
import org.firstinspires.ftc.teamcode.calibration.RobotConstants;

import java.text.FieldPosition;

/**
 * Test OpMode to validate FieldPositions refactor and BaseMotion functionality
 */
@Autonomous(name = "BaseMotion Test", group = "Debug")
public class BaseMotionTest extends LinearOpMode {
    
    private BaseMotion baseMotion;
    
    @Override
    public void runOpMode() {
        
        // Initialize BaseMotion subsystem
        baseMotion = new BaseMotion();
        baseMotion.init(this);
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Press Play", "to test FieldPositions and BaseMotion");
        telemetry.update();
        
        waitForStart();
        
        if (opModeIsActive()) {
            testFieldPositions();
            testBaseMotionBasics();
            testReferencePointManagement();
            testMoveToPoseWithBluePositions();
        }
    }
    
    /**
     * Test FieldPositions refactor
     */
    private void testFieldPositions() {
        telemetry.addData("=== TESTING FIELD POSITIONS ===", "");
        
        // Test BLUE positions (should work without prefix)
        FieldPose blueStartNear = FieldPositions.START_NEAR;
        FieldPose blueShootingClose = FieldPositions.SHOOTING_NEAR;
        
        telemetry.addData("BLUE START_NEAR", blueStartNear.toCompactString());
        telemetry.addData("BLUE SHOOTING_CLOSE", blueShootingClose.toCompactString());
        
        // Test RED position generation
        FieldPose redStartNear = FieldPositions.getRedPosition(blueStartNear);
        FieldPose redShootingClose = FieldPositions.getRedPosition(blueShootingClose);
        
        telemetry.addData("RED START_NEAR (mirrored)", redStartNear.toCompactString());
        telemetry.addData("RED SHOOTING_CLOSE (mirrored)", redShootingClose.toCompactString());
        
        // Verify mirroring works correctly
        telemetry.addData("Mirror Test", "BLUE Y=" + blueStartNear.y + " -> RED Y=" + redStartNear.y);
        telemetry.addData("Mirror Test", "BLUE H=" + blueStartNear.heading + " -> RED H=" + redStartNear.heading);
        
        telemetry.update();
        sleep(3000);
    }
    
    /**
     * Test BaseMotion basic functionality
     */
    private void testBaseMotionBasics() {
        telemetry.clear();
        telemetry.addData("=== TESTING BASE MOTION ===", "");
        
        // Test drive mode switching
        telemetry.addData("Initial Drive Mode", baseMotion.getDriveMode());
        
        baseMotion.toggleDriveMode();
        telemetry.addData("After Toggle", baseMotion.getDriveMode());
        
        baseMotion.setDriveMode(BaseMotion.DriveMode.ROBOT_CENTRIC);
        telemetry.addData("Set to Robot Centric", baseMotion.getDriveMode());
        
        // Test current pose
        telemetry.addData("Current Pose", baseMotion.getCurrentPose().toString());
        
        // Test timeMotion (very short duration for safety)
        telemetry.addData("Testing", "timeMotion FORWARD for 0.5 seconds");
        telemetry.update();
        
        baseMotion.timeMotion(BaseMotion.Direction.FORWARD, 5.0, 0.5);
        
        telemetry.addData("timeMotion", "Completed");
        telemetry.update();
        sleep(3000);
    }
    
    /**
     * Test reference point management
     */
    private void testReferencePointManagement() {
        telemetry.clear();
        telemetry.addData("=== TESTING REFERENCE POINTS ===", "");
        
        // Test getting current reference point
        telemetry.addData("Current Reference Point", FieldPositions.getActiveReferencePoint());
        telemetry.addData("Current Initial Position", FieldPositions.getReferencePointInitialPosition().toCompactString());
        
        // Test setting reference point
        baseMotion.setReferencePoint(RobotConstants.INTAKE_POINT);
        telemetry.addData("After Setting INTAKE_POINT", FieldPositions.getActiveReferencePoint());
        
        // Test setting reference point to field origin
        baseMotion.setReferencePointToFieldOrigin();
        telemetry.addData("After Setting to Origin", FieldPositions.getReferencePointInitialPosition().toCompactString());
        
        // Test setting reference point to a position
        baseMotion.setReferencePointToPosition(FieldPositions.START_NEAR);
        telemetry.addData("After Setting to START_NEAR", FieldPositions.getReferencePointInitialPosition().toCompactString());
        
        telemetry.addData("Test", "Completed Successfully!");
        telemetry.update();
        sleep(5000);
    }
    
    /**
     * Test moveToPose with Blue positions using getBluePosition method
     */
    private void testMoveToPoseWithBluePositions() {
        telemetry.clear();
        telemetry.addData("=== TESTING MOVE TO POSE ===", "");
        
        // Get Blue positions using getBluePosition method
        FieldPose blueStartNear = FieldPositions.getBluePosition("START_NEAR");
        FieldPose blueShootingNear = FieldPositions.getBluePosition("SHOOTING_CLOSE");
        
        if (blueStartNear == null || blueShootingNear == null) {
            telemetry.addData("ERROR", "Could not get Blue positions");
            telemetry.update();
            sleep(2000);
            return;
        }
        
        telemetry.addData("Initial Position", blueStartNear.toCompactString());
        telemetry.addData("Target Position", blueShootingNear.toCompactString());
        telemetry.update();
        sleep(2000);
        
        // Set initial position to Blue Start Near
        telemetry.addData("Setting Initial Position", "Blue Start Near");
        telemetry.update();
        baseMotion.setReferencePointToPosition(blueStartNear);
        sleep(3000);
        
        // Move to Blue Shooting Near position
        telemetry.addData("Moving to", "Blue Shooting Near");
        telemetry.addData("From", blueStartNear.toCompactString());
        telemetry.addData("To", blueShootingNear.toCompactString());
        telemetry.update();
        
        baseMotion.moveToPose(blueShootingNear, 12.0);

        
        telemetry.addData("Movement", "Completed!");
        telemetry.addData("Final Position", baseMotion.getCurrentPose().toCompactString());
        telemetry.update();
        sleep(3000);
    }
}
