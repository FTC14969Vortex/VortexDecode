package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

public class Kicker {

    private Servo kickerPos;
    private OpMode opMode;
    private DistanceSensor channelSersor;
    
    public void init(HardwareMap hwMap) {
        kickerPos = hwMap.get(Servo.class, "kicker");
        this.opMode = opMode;

    }

    public void setKickerPos(double position){
        kickerPos.setPosition(position);
    }
}
