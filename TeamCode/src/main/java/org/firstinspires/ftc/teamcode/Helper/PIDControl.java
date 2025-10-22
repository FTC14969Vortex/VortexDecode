package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDControl  {

    /*

     * Proportional Integral Derivative Controller

     */


    //Tune these
    double Kp = 1;
    double Ki = 0;
    double Kd = 0;

    double integralSum = 0;
    double lastError = 0;


    ElapsedTime timer = new ElapsedTime();

    boolean inPosition;

    public double getOutput(double target, double current) {

        double error = target - current;

        integralSum += error * timer.seconds();

        double derivative = (error - lastError) / timer.seconds();

        lastError = error;

        return (Kp * error) + (Kd * derivative) + (Ki * integralSum);

    }

    public boolean isInPosition() {
        return inPosition;
    }
//    @Override
//    public void runOpMode() throws InterruptedException {
//        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "motor");
//        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }
