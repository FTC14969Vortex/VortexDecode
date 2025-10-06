package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Gate {

    private CRServo leftGate;
    private CRServo rightGate;
    private OpMode opMode;

    public void init(OpMode opMode) {

        this.opMode = opMode;

        leftGate = opMode.hardwareMap.get(CRServo.class, "leftGate");
        rightGate = opMode.hardwareMap.get(CRServo.class, "rightGate");
        rightGate.setDirection(CRServo.Direction.REVERSE);;
    }
    public void release(double power) {

        leftGate.setPower(power);
        rightGate.setPower(power);
    }
    public void stop(){

        leftGate.setPower(0);
        rightGate.setPower(0);
    }
    }

