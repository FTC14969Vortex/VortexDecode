package org.firstinspires.ftc.teamcode.Helper;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

    private boolean readyToLoad() {
        boolean ret = false;

        // Get distance reading from 2M sensor
        channelSersor = hardwareMap.get(DistanceSensor.class, "channelSensor");
        double dDistance = channelSersor.getDistance(DistanceUnit.CM);
        telemetry.addData("channel distance sensor", dDistance);
        if (dDistance > 10 ) ret = true;

        return ret;
    }
}
