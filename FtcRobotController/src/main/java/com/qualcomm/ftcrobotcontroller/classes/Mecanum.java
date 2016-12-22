package com.qualcomm.ftcrobotcontroller.classes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
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
    private double MaxPower = 1;
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

    public void set_Power(float rjoystick_x,float ljoystick_y,float ljoystick_x, int upbutton){
        float ch1 = rjoystick_x;
        float ch3 = ljoystick_x;
        float ch4 = ljoystick_y;

        //Get joystick inputs
        FLpower = -(ch3 + ch1 + ch4);
        BLpower = -(ch3 + ch1 - ch4);
        FRpower = ch3 - ch1 - ch4;
        BRpower = ch3 - ch1 + ch4;
        if (upbutton == 1){
            FLpower = FLpower/2;
            FRpower = FRpower/2;
            BLpower = BLpower/2;
            BRpower = BRpower/2;
        } else {
            // Power doesn't change
        }

    }

    public void run_left(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL) {
        FLpower = -BasePower;
        BLpower = BasePower;
        FRpower = BasePower;
        BRpower = -BasePower;
        // The positive and negative inputs only mean direction, not speed.


        motorFR.setPower(FRpower);
        motorBR.setPower(BRpower);
        motorFL.setPower((FLpower));
        motorBL.setPower(BLpower);
        // This sets the motors that go into DemoAutonomous.

    }

    public void run_right(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL) {
        FLpower = BasePower;
        BLpower = -BasePower;
        FRpower = -BasePower;
        BRpower = BasePower;
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

    public void turn_right(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL, int degrees) {
        FLpower = BasePower;
        BLpower = BasePower;
        FRpower = -BasePower;
        BRpower = -BasePower;
        // The positive and negative inputs only mean direction, not speed.
        int Wheel_Rotate = degrees / 90;

        while (this.get_wheel_rotations() <= Wheel_Rotate) {
            motorFR.setPower(FRpower);
            motorBR.setPower(BRpower);
            motorFL.setPower((FLpower));
            motorBL.setPower(BLpower);
        }
        // This sets the motors that go into DemoAutonomous.

    }

    public void turn_left(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL, int degrees) {
        FLpower = -BasePower;
        BLpower = -BasePower;
        FRpower = BasePower;
        BRpower = BasePower;
        // The positive and negative inputs only mean direction, not speed.
        int Wheel_Rotate = degrees / 90;

        while (this.get_wheel_rotations() <= Wheel_Rotate) {
            motorFR.setPower(FRpower);
            motorBR.setPower(BRpower);
            motorFL.setPower((FLpower));
            motorBL.setPower(BLpower);
        }
        // This sets the motors that go into DemoAutonomous.

    }

    public void brake(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL) {
        FLpower = 0;
        BLpower = 0;
        FRpower = 0;
        BRpower = 0;
        // The positive and negative inputs only mean direction, not speed.

        motorFR.setPower(FRpower);
        motorBR.setPower(BRpower);
        motorFL.setPower((FLpower));
        motorBL.setPower(BLpower);
        // This sets the motors that go into DemoAutonomous.

    }
    public void reset_encoders(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL) {
        // The positive and negative inputs only mean direction, not speed.
        motorFR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorBR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorFL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorBL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        // This sets the motors that go into DemoAutonomous.
    }

    public void run_using_encoders(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL) {
        // The positive and negative inputs only mean direction, not speed.
        motorFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        // This sets the motors that go into DemoAutonomous.
    }
    public void setPosition(int encoderval,DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL){
        motorFR.setTargetPosition(encoderval);
        motorFL.setTargetPosition(encoderval);
        motorBR.setTargetPosition(encoderval);
        motorBL.setTargetPosition(encoderval);
    }
    public void setPower(double power){
        BasePower = power;
    }
    public int testDistance(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL){
        if (motorFL.getCurrentPosition()>= motorFL.getTargetPosition()){
            return 1;
        }else{
            return 0;
        }
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