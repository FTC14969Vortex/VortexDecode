package org.firstinspires.ftc.teamcode.helper;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {

    private DcMotor intake;
    private OpMode opMode;

    public void init(OpMode opMode) {

        this.opMode = opMode;

        intake = opMode.hardwareMap.get(DcMotor.class, "intake");
    }

    public void setIntakePower(double motorPower) {
        intake.setPower(motorPower);
    }


    public void startIntake(){
        intake.setPower(1);
    }
    public void stopIntake(){
        intake.setPower(0);
    }
}
