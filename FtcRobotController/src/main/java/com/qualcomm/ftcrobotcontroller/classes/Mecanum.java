package com.qualcomm.ftcrobotcontroller.classes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.ftcrobotcontroller.classes.Distance;

/**
 * Created by the lord and savior Arko Chaterjee on 9/7/2016.
 */
public class Mecanum{

    private final double pi = 3.1415926;
    private final double wheel_diameter = 4 * (pi);
    private Distance dis_sensor = new Distance();

    private double FRpower = 0;
    private double FLpower = 0;
    private double BRpower = 0;
    private double BLpower = 0;
    private double BasePower = .5;
//Private variables

    public Mecanum() {
        FRpower = 0;
        FLpower = 0;
        BRpower = 0;
        BLpower = 0;
    }
    public Mecanum(double motorFR, double motorFL, double motorBR, double motorBL){
        FRpower = motorFR;
        FLpower = motorFL;
        BRpower = motorBR;
        BLpower = motorBL;
    }

    public void run_motor(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL){

        motorFR.setPower(FRpower);
        motorBR.setPower(BRpower);
        motorFL.setPower((FLpower));
        motorBL.setPower(BLpower);
    }

    public void set_Power(float rjoystick_x,float ljoystick_y,float ljoystick_x){
        float ch1 = rjoystick_x;
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

    public void run_left(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL) {
        FLpower = BasePower;
        BLpower = -BasePower;
        FRpower = -BasePower;
        BRpower = BasePower;
        // The positive and negative inputs only mean direction, not speed.


        motorFR.setPower(FRpower);
        motorBR.setPower(BRpower);
        motorFL.setPower((FLpower));
        motorBL.setPower(BLpower);
        // This sets the motors that go into DemoAutonomous.

    }

    public void run_right(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL) {
        FLpower = -BasePower;
        BLpower = BasePower;
        FRpower = BasePower;
        BRpower = -BasePower;
        // The positive and negative inputs only mean direction, not speed.

        motorFR.setPower(FRpower);
        motorBR.setPower(BRpower);
        motorFL.setPower(FLpower);
        motorBL.setPower(BLpower);
        // This sets the motors that go into DemoAutonomous.

    }

    public void run_forward(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL) {
        FLpower = BasePower;
        BLpower = BasePower;
        FRpower = BasePower;
        BRpower = BasePower;
        // The positive and negative inputs only mean direction, not speed.

        motorFR.setPower(FRpower);
        motorBR.setPower(BRpower);
        motorFL.setPower(FLpower);
        motorBL.setPower(BLpower);
        // This sets the motors that go into DemoAutonomous.

    }

    public void run_backward(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL) {
        FLpower = -BasePower;
        BLpower = -BasePower;
        FRpower = -BasePower;
        BRpower = -BasePower;
        // The positive and negative inputs only mean direction, not speed.

        motorFR.setPower(FRpower);
        motorBR.setPower(BRpower);
        motorFL.setPower(FLpower);
        motorBL.setPower(BLpower);
        // This sets the motors that go into DemoAutonomous.

    }

    public void run_diagonal_right_up(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL) {
        FLpower = BasePower;
        BLpower = 0;
        FRpower = 0;
        BRpower = BasePower;
        // The positive and negative inputs only mean direction, not speed.


        motorFR.setPower(FRpower);
        motorBR.setPower(BRpower);
        motorFL.setPower((FLpower));
        motorBL.setPower(BLpower);
        // This sets the motors that go into DemoAutonomous.

    }

    public void run_diagonal_left_up(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL) {
        FLpower = 0;
        BLpower = BasePower;
        FRpower = BasePower;
        BRpower = 0;
        // The positive and negative inputs only mean direction, not speed.


        motorFR.setPower(FRpower);
        motorBR.setPower(BRpower);
        motorFL.setPower((FLpower));
        motorBL.setPower(BLpower);
        // This sets the motors that go into DemoAutonomous.

    }

    public void run_diagonal_left_down(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL) {
        FLpower = -BasePower;
        BLpower = 0;
        FRpower = 0;
        BRpower = -BasePower;
        // The positive and negative inputs only mean direction, not speed.

        motorFR.setPower(FRpower);
        motorBR.setPower(BRpower);
        motorFL.setPower((FLpower));
        motorBL.setPower(BLpower);
        // This sets the motors that go into DemoAutonomous.

    }

    public void run_diagonal_right_down(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL) {
        FLpower = 0;
        BLpower = -BasePower;
        FRpower = -BasePower;
        BRpower = 0;
        // The positive and negative inputs only mean direction, not speed.


        motorFR.setPower(FRpower);
        motorBR.setPower(BRpower);
        motorFL.setPower((FLpower));
        motorBL.setPower(BLpower);
        // This sets the motors that go into DemoAutonomous.

    }

    public void turn_right(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL) {
        FLpower = BasePower;
        BLpower = BasePower;
        FRpower = -BasePower;
        BRpower = -BasePower;
        // The positive and negative inputs only mean direction, not speed.

        motorFR.setPower(FRpower);
        motorBR.setPower(BRpower);
        motorFL.setPower((FLpower));
        motorBL.setPower(BLpower);
        // This sets the motors that go into DemoAutonomous.

    }

    public void turn_left(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL) {
        FLpower = -BasePower;
        BLpower = -BasePower;
        FRpower = BasePower;
        BRpower = BasePower;
        // The positive and negative inputs only mean direction, not speed.


        motorFR.setPower(FRpower);
        motorBR.setPower(BRpower);
        motorFL.setPower((FLpower));
        motorBL.setPower(BLpower);
        // This sets the motors that go into DemoAutonomous.

    }

    public void brake(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL) {
        FLpower = 0;
        BLpower = 0;
        FRpower = 0;
        BRpower = 0;
        // The positive and negative inputs only mean direction, not spee

        motorFR.setPower(FRpower);
        motorBR.setPower(BRpower);
        motorFL.setPower((FLpower));
        motorBL.setPower(BLpower);
        // This sets the motors that go into DemoAutonomous.

    }
    public double get_wheel_rotations() {
        double distance = 0;
        double wheel_rotation = 0;

        distance = dis_sensor.getWheelDistance();
        wheel_rotation = distance/wheel_diameter;

        return wheel_rotation;
    }
    //initialization routine



}