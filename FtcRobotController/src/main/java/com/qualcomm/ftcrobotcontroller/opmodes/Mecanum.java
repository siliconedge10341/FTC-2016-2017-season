package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Dival Banerjee on 9/7/2016.
 */
public class Mecanum  extends OpMode{

    float FRpower;
    float FLpower;
    float BRpower;
    float BLpower;


    public Mecanum() {


    }

    public void runmotor(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL){
        motorFL = hardwareMap.dcMotor.get("bmotor_l");
        motorFR = hardwareMap.dcMotor.get("bmotor_r");
        motorBL = hardwareMap.dcMotor.get("fmotor_l");
        motorBR = hardwareMap.dcMotor.get("fmotor_r");

        motorFR.setPower(FRpower);
        motorBR.setPower(BRpower);
        motorFL.setPower((FLpower));
        motorBL.setPower(BLpower);
    }

    public void set_Power(float rjoystick_x,float ljoystick_y,float ljoystick_x){
        float ch1= rjoystick_x;
        float ch3 = ljoystick_y;
        float ch4 = ljoystick_x;

        FRpower = ch3 + ch1 + ch4;
        BLpower = ch3 + ch1 - ch4;
        FRpower = ch3 - ch1 - ch4;
        BRpower = ch3 - ch1 + ch4;



    }
    //initialization routine


    public void init() {



        //lmotor_1 ----> LeftMotor
        //rmotor_1 ----> RightMotor

        //Turn the collector


        //TODO design a power-on self test
        //Power ON Self Test

    }

    //main function body
    @Override
    public void loop() {


    }


    @Override
    public void stop() {

    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
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