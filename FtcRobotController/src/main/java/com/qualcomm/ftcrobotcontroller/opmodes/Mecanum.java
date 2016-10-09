package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Dival Banerjee on 9/7/2016.
 */
public class Mecanum{

    float FRpower;
    float FLpower;
    float BRpower;
    float BLpower;


    public Mecanum() {


    }
    public Mecanum(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL){

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

    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

}