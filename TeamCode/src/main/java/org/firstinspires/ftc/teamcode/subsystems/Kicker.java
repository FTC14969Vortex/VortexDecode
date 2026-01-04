package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Kicker {

    private Servo kickerPos;
    private OpMode opMode;

    public static final double gateClose = 0.37;
    public static final double gateShoot = 0.21;
    public static final double gateIntake = 0.48; // was 0.6
                                                // 0.5 was good for intake without 2nd ball run against the passive gate
                                                // 0.45 good for intake 3 without touching flywheel

    public static final String GATE_CLOSE = "GATE_CLOSE";
    public static final String GATE_SHOOT = "GATE_SHOOT";
    public static final String GATE_INTAKE = "GATE_INTAKE";

    private String gatePosition;

    public void init(HardwareMap hwMap) {
        kickerPos = hwMap.get(Servo.class, "kicker");
        this.opMode = opMode;
        setGatePosition(GATE_INTAKE);
    }
    public void setGatePosition(String gatePosition){
        this.gatePosition = gatePosition;

        if(gatePosition.equals(GATE_CLOSE)){
            kickerPos.setPosition(gateClose);
        } else if(gatePosition.equals(GATE_SHOOT)){
            kickerPos.setPosition(gateShoot);
        }  else if(gatePosition.equals(GATE_INTAKE)){
            kickerPos.setPosition(gateIntake);
        } else {
            return;
        }

    }

    public String getGatePosition(){
        return gatePosition;
    }

    public void setPosition(double position){
        kickerPos.setPosition(position);
    }

    public double getPosition(){
        return kickerPos.getPosition();
    }

}
