package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.ftcrobotcontroller.classes.Mecanum;

import com.qualcomm.robotcore.util.Range;

/**
 * Created by Dival Banerjee on 9/7/2016.
 */
public class MecanumOP extends OpMode{


    //to avoid the overuse of magic numbers

    //drive motors
    DcMotor motorFR;
    DcMotor motorFL;
    DcMotor motorBR;
    DcMotor motorBL;

    Mecanum yo;

    public MecanumOP() {
        // default constructor
        motorFL.setPower(0);
        motorFL.setPower(0);
        motorBR.setPower(0);
        motorBL.setPower(0);
        yo.set_Power(0,0,0);
    }

    //initialization routine
    @Override
    public void init() {
        motorFL = hardwareMap.dcMotor.get("bmotor_l");
        motorFR = hardwareMap.dcMotor.get("bmotor_r");
        motorBL = hardwareMap.dcMotor.get("fmotor_l");
        motorBR = hardwareMap.dcMotor.get("fmotor_r");

    }

    //main function body
    @Override
    public void loop() {
        yo.set_Power(gamepad2.right_stick_x, gamepad1.left_stick_y, gamepad1.left_stick_x);
        yo.run_motor( motorFR, motorFL, motorBR, motorBL);

    }

    @Override
    public void stop() {
        yo.set_Power(0, 0, 0);
        // set to zero so the power doesn't influnce any motion or rotation in the robot
        yo.run_motor( motorFR, motorFL, motorBR, motorBL);

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