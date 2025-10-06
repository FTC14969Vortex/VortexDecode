//imports
package org.firstinspires.ftc.teamcode.Opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helper.Chassis;
import org.firstinspires.ftc.teamcode.Helper.Chassis2;
import org.firstinspires.ftc.teamcode.Helper.DecodeAprilTag;
import org.firstinspires.ftc.teamcode.Helper.FlyWheel;
import org.firstinspires.ftc.teamcode.Helper.Gate;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@TeleOp(name = "DecodeTeleopV2.4", group = "TeleOp")

public class Teleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Chassis2 chassis = new Chassis2(this);
        chassis.init(true);
//        chassis.setDriveMode(Chassis.DriveMode.ROBOT_CENTRIC);

        FlyWheel flyWheel = new FlyWheel();
        flyWheel.init(this);

        Gate gate = new Gate();
        gate.init(this);

//        DecodeAprilTag aprilTag = new DecodeAprilTag(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        chassis.odo.resetPosAndIMU();
        waitForStart();
        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            chassis.odo.update();
            telemetry.addData("Odo x", chassis.odo.getEncoderX());
            telemetry.addData("Odo y", chassis.odo.getEncoderY());
            telemetry.update();
//            if (gamepad1.a) {
//                chassis.resetHeading();
//
//            }
//            // If gamepad x is pressed then switch to field centric
//            // If gamepad y is pressed then switch to robot centric
//            if (gamepad1.x) {
//                chassis.resetIMU();
//                chassis.setDriveMode(Chassis.DriveMode.FIELD_CENTRIC);
//            } else if (gamepad1.y) {
//                chassis.setDriveMode(Chassis.DriveMode.ROBOT_CENTRIC);
//            }
            float axial = -gamepad1.left_stick_y;
            float lateral = gamepad1.left_stick_x;
            float yaw = gamepad1.right_stick_x;
            chassis.moveRobot(axial,lateral,yaw);

            if (gamepad1.right_bumper) {
                flyWheel.start(1);
                sleep(1500);
                gate.release(1);

            } else if (gamepad1.left_bumper) {
                flyWheel.stop();
                gate.stop();


        }
    }

}
    }