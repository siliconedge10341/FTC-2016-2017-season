package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by the lord and savior Arko Chaterjee on 9/7/2016.
 */
public class Mecanum{

    private float FRpower;
    private float FLpower;
    private float BRpower;
    private float BLpower;
//Private variables

    public Mecanum() {
        FRpower = 0;
        FLpower = 0;
        BRpower = 0;
        BLpower = 0;
    }
    public Mecanum(float motorFR, float motorFL, float motorBR, float motorBL){
        FRpower = motorFR;
        FLpower = motorFL;
        BRpower = motorBR;
        BLpower = motorBL;
    }

    public void runmotor(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL){

        motorFR.setPower(FRpower);
        motorBR.setPower(BRpower);
        motorFL.setPower((FLpower));
        motorBL.setPower(BLpower);
    }

    public void set_Power(float rjoystick_x,float ljoystick_y,float ljoystick_x){
        float ch1= rjoystick_x;
        float ch3 = ljoystick_y;
        float ch4 = ljoystick_x;

        //Get joystick inputs
        FLpower = ch3 + ch1 + ch4;
        BLpower = ch3 + ch1 - ch4;
        FRpower = ch3 - ch1 - ch4;
        BRpower = ch3 - ch1 + ch4;

        FLpower = Range.clip(FLpower,-1,1);
        BLpower = Range.clip(BLpower,-1,1);
        FRpower = Range.clip(FRpower,-1,1);
        BRpower = Range.clip(BRpower,-1,1);

    }
    //initialization routine



}