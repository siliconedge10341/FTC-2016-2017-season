package org.firstinspires.ftc.teamcode.classes;

/**
 * Created by bandi on 1/24/2017.
 */

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;


public class FlyWheelShooter extends LinearOpMode{
    // Uses PID controlled shooting, so accuracy will not get affected by lower battery voltage

    protected DcMotor motorShootL;
    protected DcMotor motorShootR;
//Left Motor
    private double kP = 9000000.0;
    private double kI = 0.0;
    //private double kD = 380000;
    private double kD = 10000.0;

    private double integral = 0.0;
    private double derivative = 0.0;

    private double motorOut = 0.0;
    private double fTarget = 7.5e-7;
    private double fVelocity = 0.0;
    private double fError = 0.0;
    private double fLastError = 0.0;

    private int fEncoder = 0;
    private int fLastEncoder = 0;

    private long fVelocityTime = 0;
    private long fLastVelocityTime = 0;

    private double place = 0.1;

    private boolean firstCross;

    private double tolerance = 0.5e-7;

    private double targetVoltage = 12.5;
    private double voltage;
//Right Motor
    private double kPRight = 9000000.0;
    private double kIRight = 0.0;
    //private double kD = 380000;
    private double kDRight = 10000.0;

    private double integralRight = 0.0;
    private double derivativeRight = 0.0;

    private double motorOutRight = 0.0;
    private double fTargetRight = 7.5e-7;
    private double fVelocityRight = 0.0;
    private double fErrorRight = 0.0;
    private double fLastErrorRight = 0.0;
    private double tbh = 0.0;

    private int fEncoderRight = 0;
    private int fLastEncoderRight = 0;

    private long fVelocityTimeRight = 0;
    private long fLastVelocityTimeRight = 0;


    public void calculateLeftPID(){
        fVelocityTime = System.nanoTime();
        fEncoder = motorShootL.getCurrentPosition();
        fVelocity = (double)(fEncoder - fLastEncoder) / (fVelocityTime - fLastVelocityTime);
        fError = fTarget - fVelocity;

        integral += fError;
        if(fError == 0)
        {
            integral = 0;
        }

        if(Math.abs(fError) > 50)
        {
            integral = 0;
        }

        derivative = fError - fLastError;

        fLastError = fError;
        fLastEncoder = fEncoder;
        fLastVelocityTime = fVelocityTime;

        motorOut = (kP * fError) + (kI * integral) + (kD * derivative);

        motorOut = Range.clip(motorOut, 0.0, 1.0);

        telemetry.addData("1", "LEFTkP " + (kP * fError));
        telemetry.addData("2", "LEFTError " + fError);
        telemetry.addData("3", "LEFTTime " + fVelocityTime);
        telemetry.addData("4", "LEFTEncoder " + fEncoder);
        telemetry.addData("5", "LEFTLast Encoder " + fLastEncoder);
        telemetry.addData("6", "LEFTEncoder Change " + (fEncoder - fLastEncoder));
        telemetry.addData("7", "LEFTTime Change " + (fVelocityTime - fLastVelocityTime));
        telemetry.addData("8", "LEFTVelocity " + fVelocity);
        telemetry.addData("9", "LEFTResult " + motorOut);
        telemetry.update();
        motorShootL.setPower(motorOut);
    }

    public void calculateRightPID(){
        fVelocityTimeRight = System.nanoTime();
        fEncoderRight = motorShootL.getCurrentPosition();
        fVelocityRight = (double)(fEncoder - fLastEncoder) / (fVelocityTime - fLastVelocityTime);
        fErrorRight = fTargetRight - fVelocityRight;

        integralRight += fErrorRight;
        if(fErrorRight == 0)
        {
            integralRight = 0;
        }

        if(Math.abs(fError) > 50)
        {
            integralRight = 0;
        }

        derivative = fError - fLastError;

        fLastError = fError;
        fLastEncoder = fEncoder;
        fLastVelocityTime = fVelocityTime;

        motorOut = (kP * fError) + (kI * integral) + (kD * derivative);

        motorOut = Range.clip(motorOut, 0.0, 1.0);

        telemetry.addData("1", "RIGHTkP " + (kP * fError));
        telemetry.addData("2", "RIGHTError " + fError);
        telemetry.addData("3", "RIGHTTime " + fVelocityTime);
        telemetry.addData("4", "RIGHTEncoder " + fEncoder);
        telemetry.addData("5", "RIGHTLast Encoder " + fLastEncoder);
        telemetry.addData("6", "RIGHTEncoder Change " + (fEncoder - fLastEncoder));
        telemetry.addData("7", "RIGHTTime Change " + (fVelocityTime - fLastVelocityTime));
        telemetry.addData("8", "RIGHTVelocity " + fVelocity);
        telemetry.addData("9", "RIGHTResult " + motorOut);
        telemetry.update();
        motorShootL.setPower(motorOut);
    }

    public void runOpMode() throws InterruptedException{
        motorShootL = hardwareMap.dcMotor.get("shooter_left");
        motorShootR = hardwareMap.dcMotor.get("shooter_right");

    }

    public double getBatteryVoltage(){
        double voltage = this.hardwareMap.voltageSensor.iterator().next().getVoltage();
        telemetry.addData("Voltage",voltage);
        return voltage;
    }
}
