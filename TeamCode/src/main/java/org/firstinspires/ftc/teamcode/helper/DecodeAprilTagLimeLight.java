package org.firstinspires.ftc.teamcode.helper;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.ArrayList;

public class DecodeAprilTagLimeLight {

    public static final String BLUE_APRIL_TAG = "bluetarget"; // e.g., Tag ID 20
    public static final String RED_APRIL_TAG = "redtarget";  // e.g., Tag ID 24

    private final int BLUE_TARGET_ID = 20;
    private final int RED_TARGET_ID = 24;

    // Name of the Limelight device configured in the Hardware Map
    private static final String LIMELIGHT_NAME = "limelight";

    private Limelight3A limelight;
    private LinearOpMode opChassis;
    private ArrayList<String> telemetryMsgList = new ArrayList<>();

    // Constructor
    public void DecodeAprilTagLimelight(LinearOpMode chassis) {
        opChassis = chassis;
    }

    /**
     * Initializes the Limelight3A device and starts polling for data.
     */
    public void initCamera() {
        try {
            // Get the Limelight device from the Hardware Map
            limelight = opChassis.hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);

            // Set the active pipeline (Assuming pipeline 0 is your AprilTag pipeline)
            limelight.pipelineSwitch(0);

            // Start polling for data
            limelight.start();
            telemetryMsgList.add("Limelight Initialized and Polling Started (Pipeline 0)");

        } catch (Exception e) {
            telemetryMsgList.add("ERROR: Could not find or initialize Limelight: " + e.getMessage());
        }
        outTelemetry();
    }

    /**
     * Stops the Limelight data polling.
     */
    public void stopCamera() {
        if (limelight != null) {
            limelight.stop();
            telemetryMsgList.add("Limelight Polling Stopped");
        }
        outTelemetry();
    }

    public boolean findAprilTag(int aprilTagID) {

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return false;

        // Check the fiducial results list
        for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
            if (fr.getFiducialId() == aprilTagID) {
                return true;
            }
        }

        return false;
    }

    public AprilTagPoseFtc getCoordinate(int aprilTagID) {

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        // Step through the fiducial detections
        for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
            if (fr.getFiducialId() == aprilTagID) {

                // Limelight SDK method getTargetPoseCameraSpace() returns an FTC Pose3D object.
                Pose3D pose = fr.getTargetPoseCameraSpace();

                // --- Access Position (X, Y, Z) ---
                // Limelight provides this in meters.
                Position position = pose.getPosition();
                double x_m = position.x;
                double y_m = position.y;
                double z_m = position.z;

                double x_in = DistanceUnit.INCH.fromMeters(x_m);
                double y_in = DistanceUnit.INCH.fromMeters(y_m);
                double z_in = DistanceUnit.INCH.fromMeters(z_m);

                // Calculate range
                double range_in = Math.sqrt(x_in * x_in + y_in * y_in + z_in * z_in);

                // --- Access Orientation (Roll, Pitch, Yaw) ---
                // Limelight provides these in degrees.
                YawPitchRollAngles orientation = pose.getOrientation();
                double roll_deg = orientation.getRoll(AngleUnit.DEGREES);
                double pitch_deg = orientation.getPitch(AngleUnit.DEGREES);
                double yaw_deg = orientation.getYaw(AngleUnit.DEGREES);

                // Create the FTC AprilTagPoseFtc object
                AprilTagPoseFtc retPose = new AprilTagPoseFtc(
                        range_in,               // Range
                        yaw_deg,                // Bearing (often mapped to Yaw/Heading)
                        pitch_deg,              // Elevation (often mapped to Pitch)
                        x_in,                   // X (Translate X in inches)
                        y_in,                   // Y (Translate Y in inches)
                        z_in,                   // Z (Translate Z in inches)
                        roll_deg,               // Roll (Rotate X in degrees)
                        pitch_deg,              // Pitch (Rotate Y in degrees)
                        yaw_deg                 // Yaw (Rotate Z in degrees)
                );

                //telemetryMsgList.add(String.format("\n==== (ID %d) %s", fr.getFiducialId(), name));
                telemetryMsgList.add(String.format("Range, %5.1f inches", retPose.range));
                telemetryMsgList.add(String.format("Bearing (Yaw),%3.0f degrees", retPose.bearing));
                telemetryMsgList.add(String.format("Pitch, %3.0f degrees", retPose.elevation));
                outTelemetry();

                return retPose;
            }
        }

        return null;
    }

    /**
     * Private helper to map the friendly name to the AprilTag ID.
     */
    private int getTargetId(String name) {
        if (name.compareToIgnoreCase(BLUE_APRIL_TAG) == 0) {
            return BLUE_TARGET_ID;
        } else if (name.compareToIgnoreCase(RED_APRIL_TAG) == 0) {
            return RED_TARGET_ID;
        }
        return -1;
    }

    /**
     * Private helper to output telemetry messages.
     */
    private void outTelemetry() {
        for (String msg : telemetryMsgList) {
            opChassis.telemetry.addLine(msg);
        }
        opChassis.telemetry.update();
        telemetryMsgList.clear();
    }

}
