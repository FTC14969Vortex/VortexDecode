//imports
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "TeleOp-Peter-v2", group = "TeleOp")

public class TeleOpDrive extends LinearOpMode {

    @Override
    public void runOpMode() {

        Chassis chassis = new Chassis(this);
        chassis.setRobotCentric();
        chassis.init();

        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Use gamepad Y button to toggle between robot and field centric control
            if (gamepad1.y) {
                chassis.toggleDriveMode();
            }

            if (gamepad1.a) {
                chassis.resetIMU();
            }

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            // Note: pushing stick forward gives negative value
            // stick Y direction: up is -, down is +
            // stick X direction: left is -, right is +
            chassis.drive(
                    -gamepad1.left_stick_y, // axial - Forward/Backward (inverted joystick for push forward = positive)
                    gamepad1.left_stick_x, // lateral - Strafe left/right (positive is right, negation is left)
                    gamepad1.right_stick_x // yaw -  Turn left/right (positive is clockwise, negative is counter-clockwise)
            );

        }
    }
}