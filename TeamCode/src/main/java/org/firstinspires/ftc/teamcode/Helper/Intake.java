package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {

    private DcMotor intake;
    private OpMode opMode;

    public void init(OpMode opMode) {

        this.opMode = opMode;

        intake = opMode.hardwareMap.get(DcMotor.class, "intake");
    }

    public void intake(double motorPower) {


        intake.setPower(motorPower);
    }

    public void stopIntake(){
        intake.setPower(0);
    }
}
