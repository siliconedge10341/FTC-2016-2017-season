package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorController;
//import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
//import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by the lord and savior Arko Chaterjee on 9/7/2016.
 */
public class Mecanum{
    // instance variables
    // private variables
    // constants
    private final double pi = 3.1415926;
    private final double wheel_diameter = 4 * (pi);
    private Distance dis_sensor = new Distance();

    // powers
    private double FRpower = 0;
    private double FLpower = 0;
    private double BRpower = 0;
    private double BLpower = 0;
    private double BasePower = 1;
    //private double MaxPower = 1;

    // constructors
    public Mecanum() {
        // default constructors
        FRpower = 0;
        FLpower = 0;
        BRpower = 0;
        BLpower = 0;
    }
    public Mecanum(double motorFR, double motorFL, double motorBR, double motorBL) {
        // fill constructor
        // this sets the motors to the specific train withing demo and drive
        FRpower = motorFR;
        FLpower = motorFL;
        BRpower = motorBR;
        BLpower = motorBL;
    }

    // sets
    public void run_motor(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL) {
        motorFR.setPower(FRpower);
        motorBR.setPower(BRpower);
        motorFL.setPower((FLpower));
        motorBL.setPower(BLpower);
    }
    public void set_Power(float rjoystick_x,float ljoystick_y,float ljoystick_x, int upbutton){
        float ch1 = rjoystick_x;
        float ch3 = ljoystick_x;
        float ch4 = ljoystick_y;

        double X2 = 0, Y1 = 0, X1 = 0, threshold = .15;

        if(Math.abs(ch3) > threshold)
            Y1 = ch3;
        else
            Y1 = 0;
//Create "deadzone" for X1/Ch4
        if(Math.abs(ch4) > threshold)
            X1 = ch4;
        else
            X1 = 0;
//Create "deadzone" for X2/Ch1
        if(Math.abs(ch1) > threshold)
            X2 = ch1;
        else
            X2 = 0;

        if(Y1<.3 && Y1>-.3){
            Y1 = 0;
        }

            FLpower = (Y1 + X2 + X1);
            BLpower = (Y1 + X2 - X1);
            FRpower = -(Y1 - X2 - X1);
            BRpower = -(Y1 - X2 + X1);


    }
    public void setPosition(int fr,int fl,int br, int bl, DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL){
        motorFR.setTargetPosition(fr);
        motorFL.setTargetPosition(fl);
        motorBR.setTargetPosition(br);
        motorBL.setTargetPosition(bl);
    }
    public void setPower(double power) {
        BasePower = power;
    }

    // gets
    public double get_wheel_rotations() {
        double distance = 0;
        double wheel_rotation = 0;

        distance = dis_sensor.getWheelDistance();
        wheel_rotation = distance/wheel_diameter;

        return wheel_rotation;
    }

    // methods
    // These are the functions for the specific direction
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

    // resets
    public void reset_encoders(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL) {
        // The positive and negative inputs only mean direction, not speed.
        motorFR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFL.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBL.setMode(DcMotor.RunMode.RESET_ENCODERS);

        // This sets the motors that go into DemoAutonomous.
    }
    public void run_to_position(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL) {
        // The positive and negative inputs only mean direction, not speed.
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // This sets the motors that go into DemoAutonomous.
    }
    public void run_using_encoders(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL) {
        // The positive and negative inputs only mean direction, not speed.
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

        // This sets the motors that go into DemoAutonomous.
    }

    // testers
    public int testDistance(DcMotor A_Motor) {
        // This takes any motor and tests the distance that that motor has traveled
        // We only do one because all have traveled the same amount. Knowing Turns
        // Do not count.

        if (A_Motor.getCurrentPosition()>= A_Motor.getTargetPosition()){
            return 1;
        }else{
            return 0;
        }
    }
    //initialization routine



}