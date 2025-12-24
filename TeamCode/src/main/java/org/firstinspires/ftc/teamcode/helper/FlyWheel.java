package org.firstinspires.ftc.teamcode.helper;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FlyWheel {

    
    private DcMotorEx flyWheel;
    private OpMode opMode;
    public static int FLYWHEEL_SHOOTING_VELOCITY = 1200;
    public double FLYWHEEL_SHOOTING_POWER = (0.65*FLYWHEEL_SHOOTING_VELOCITY)/1500;


    public void init (OpMode opMode) {

        this.opMode = opMode;

        flyWheel = opMode.hardwareMap.get(DcMotorEx.class, "flyWheel");
        flyWheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void start() {
        flyWheel.setPower(FLYWHEEL_SHOOTING_POWER);
    }
    public void start(Double flyWheelShootingPower) {
        flyWheel.setPower(flyWheelShootingPower);
    }
    public void stop() {
        flyWheel.setPower(0.0);
    }
    public void setPower(double power) {

        flyWheel.setPower(power);
    }

    public double getPower(){
        return flyWheel.getPower();

    }

    public double getVelocity(){
        return flyWheel.getVelocity();
    }

    }