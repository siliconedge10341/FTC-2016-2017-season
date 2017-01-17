package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by the lord and savior Arko Chaterjee, Richard the genius, and Krimps on 9/7/2016.
 */
public class Mecanum{
    // instance variables
    // private variables
        // constants
        private final double pi = 3.1415926;
        private final double wheel_diameter = 4 * (pi);

        // powers
        private double FRpower = 0;
        private double FLpower = 0;
        private double BRpower = 0;
        private double BLpower = 0;
        private double BasePower = 0;

    // constructors
    public Mecanum() {
        // default constructors
        FRpower = 0;
        FLpower = 0;
        BRpower = 0;
        BLpower = 0;
        BasePower = .5;
    }

    // sets
    public void set_Power(float rjoystick_x,float ljoystick_y,float ljoystick_x, boolean percision_flag){
        float ch1 = rjoystick_x;
        float ch3 = ljoystick_x;
        float ch4 = ljoystick_y;
        double midfl, midbl, midfr, midbr;
        boolean FL = ch4 > ch3;
        boolean FR = ch4 > -ch3;
        if (ch1 < -0.4 || ch1 > 0.4) {
            midfl = ch1;
            midbl = ch1;
            midfr = ch1;
            midbr = ch1;
        } else if (FL && FR) {
            midfl = ch4;
            midbl = ch4;
            midfr = -ch4;
            midbr = -ch4;
        } else if (FL || FR) {
            if(FL) {
                //
                // Will be adjusted
                //
                midfl = ch3; //0.7
                midbl = -ch3;  //0.95
                midfr = ch3; //0.78
                midbr = -ch3;  //1
            } else {
                midfl = ch3; //.78
                midbl = -ch3;  //1
                midfr = ch3; //0.7
                midbr = -ch3;  //0.95
            }
        } else {
            midfl = ch4;
            midbl = ch4;
            midfr = -ch4;
            midbr = -ch4;
        }
        //
        // Power has to be over a certain amount. We found this to work because the amount of
        // power that is below 1 together doesn't move the robot anyways. So, in order to protect
        // the motors, we did this in case.
        //
        if (Math.abs(midfl) + Math.abs(midbl) + Math.abs(midfr) + Math.abs(midbr) < 1)
        {
            midfl = 0;
            midbl = 0;
            midfr = 0;
            midbr = 0;
        }

        FLpower= midfl;
        BLpower= midbl;
        FRpower= midfr;
        BRpower= midbr;


        if (Math.abs(FLpower) + Math.abs(BLpower) + Math.abs(FRpower) + Math.abs(BRpower) < 0.5)
        {
            FLpower = 0;
            BLpower = 0;
            FRpower = 0;
            BRpower = 0;
        }

        /*
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

        if (percision_flag = true) {
            FLpower = -(Y1 + X2 + X1);
            BLpower = (Y1 + X2 - X1);
            FRpower = -(Y1 - X2 - X1);
            BRpower = (Y1 - X2 + X1);
        } else {
            FLpower = -(Y1 + X2 + X1) / 2;
            BLpower = (Y1 + X2 - X1) / 2;
            FRpower = -(Y1 - X2 - X1) / 2;
            BRpower = (Y1 - X2 + X1) / 2;
        }
        */

    }
    public void setPosition(int fr,int fl,int br, int bl, DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL){
        //
        // Sets the motor position to something and the runs it.
        motorFR.setTargetPosition(fr);
        motorFL.setTargetPosition(fl);
        motorBR.setTargetPosition(br);
        motorBL.setTargetPosition(bl);

    }
    public void setPowerD(double power) {
        //
        // Sets the power of the motor
        //
        BasePower = power;

    }

    // gets
    public double get_wheel_rotations() {
        double distance = 0;
        double wheel_rotation = 0;

        //distance = dis_sensor.getWheelDistance();
        wheel_rotation = distance/wheel_diameter;

        return wheel_rotation;
    }

    // methods
    public void run_motor(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL) {
        motorFR.setPower(FRpower);
        motorBR.setPower(BRpower);
        motorFL.setPower(FLpower);
        motorBL.setPower(BLpower);
    }

        // testers
        public int testDistance(DcMotor A_Motor) {
            // This takes any motor and tests the distance that that motor has traveled
            // We only do one because all have traveled the same amount. Knowing Turns
            // Do not count.

            if (Math.abs(A_Motor.getCurrentPosition()) >= Math.abs(A_Motor.getTargetPosition())){
                return 1;
            }else{
                return 0;
            }
        }

    // directions for the robot to run
    public void run_left_using_alignment(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL, double distanceFrontR, double distanceBackR) {
        FLpower = -BasePower;
        BLpower = BasePower;
        FRpower = -BasePower;
        BRpower = BasePower;
        double distanceDifference = distanceFrontR - distanceBackR;
        if(distanceDifference > 2) {
            //
            // turn right
            //
            BLpower += distanceDifference/60;
            BRpower += distanceDifference/60;
        } else if(distanceDifference < -2) {
            distanceDifference = -1 * distanceDifference;
            //
            // turn left
            //
            FLpower -= distanceDifference/60;
            FRpower -= distanceDifference/60;
        }
        motorFR.setPower(FRpower);
        motorBR.setPower(BRpower);
        motorFL.setPower(FLpower);
        motorBL.setPower(BLpower);
    }

    public void run_right_using_alignment(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL, double distanceFrontR, double distanceBackR) {
        FLpower = BasePower;
        BLpower = -BasePower;
        FRpower = BasePower;
        BRpower = -BasePower;
        double distanceDifference = distanceFrontR - distanceBackR;
        if(distanceDifference > 2) {
            //
            // turn right
            //
            FLpower += distanceDifference/60;
            FRpower += distanceDifference/60;
        } else if(distanceDifference < -2) {
            distanceDifference = -1 * distanceDifference;
            //
            // turn left
            //
            BLpower -= distanceDifference/60;
            BRpower -= distanceDifference/60;
        }
        motorFR.setPower(FRpower);
        motorBR.setPower(BRpower);
        motorFL.setPower(FLpower);
        motorBL.setPower(BLpower);

    }

    // methods
    //
    // These are the functions for the specific direction
    //
    public void run_left(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL) {
        FLpower = -BasePower;
        BLpower = BasePower;
        FRpower = -BasePower;
        BRpower = BasePower;
        //
        // The positive and negative inputs only mean direction, not speed.
        //
        motorFR.setPower(FRpower);
        motorBR.setPower(BRpower);
        motorFL.setPower(FLpower);
        motorBL.setPower(BLpower);
        //
        // This sets the motors that go into DemoAutonomous.
        //
    }
    public void run_right(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR , DcMotor motorBL) {
        FLpower = BasePower;
        BLpower = -BasePower;
        FRpower = BasePower;
        BRpower = -BasePower;
        //
        // The positive and negative inputs only mean direction, not speed.
        //
        motorFR.setPower(FRpower);
        motorBR.setPower(BRpower);
        motorFL.setPower(FLpower);
        motorBL.setPower(BLpower);
        //
        // This sets the motors that go into DemoAutonomous.
        //
    }
    public void run_forward(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL) {
        FLpower = BasePower;
        BLpower = BasePower;
        FRpower = -BasePower;
        BRpower = -BasePower;
        //
        // The positive and negative inputs only mean direction, not speed.
        //
        motorFR.setPower(FRpower);
        motorBR.setPower(BRpower);
        motorFL.setPower(FLpower);
        motorBL.setPower(BLpower);
        //
        // This sets the motors that go into DemoAutonomous.
        //
    }
    public void run_backward(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL) {
        FLpower = -BasePower;
        BLpower = -BasePower;
        FRpower = BasePower;
        BRpower = BasePower;
        //
        // The positive and negative inputs only mean direction, not speed.
        //
        motorFR.setPower(FRpower);
        motorBR.setPower(BRpower);
        motorFL.setPower(FLpower);
        motorBL.setPower(BLpower);
        //
        // This sets the motors that go into DemoAutonomous.
        //
    }
    public void run_diagonal_right_up(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL) {
        FLpower = BasePower;
        BLpower = 0;
        FRpower = 0;
        BRpower = BasePower;
        //
        // The positive and negative inputs only mean direction, not speed.
        //
        motorFR.setPower(-FRpower);
        motorBR.setPower(-BRpower);
        motorFL.setPower((FLpower));
        motorBL.setPower(BLpower);
        //
        // This sets the motors that go into DemoAutonomous.
        //
    }
    public void run_diagonal_left_up(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL) {
        FLpower = BasePower;
        BLpower = 0;
        FRpower = 0;
        BRpower = -BasePower;
        //
        // The positive and negative inputs only mean direction, not speed.
        //
        motorFR.setPower(FRpower);
        motorBR.setPower(BRpower);
        motorFL.setPower(FLpower);
        motorBL.setPower(BLpower);
        //
        // This sets the motors that go into DemoAutonomous.
        //
    }
    public void run_diagonal_left_down(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL) {
        FLpower = -BasePower;
        BLpower = 0;
        FRpower = 0;
        BRpower = -BasePower;
        //
        // The positive and negative inputs only mean direction, not speed.
        //
        motorFR.setPower(-FRpower);
        motorBR.setPower(-BRpower);
        motorFL.setPower((FLpower));
        motorBL.setPower(BLpower);
        //
        // This sets the motors that go into DemoAutonomous.
        //
    }
    public void run_diagonal_right_down(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL) {
        FLpower = 0;
        BLpower = -BasePower;
        FRpower = -BasePower;
        BRpower = 0;
        //
        // The positive and negative inputs only mean direction, not speed.
        //
        motorFR.setPower(-FRpower);
        motorBR.setPower(-BRpower);
        motorFL.setPower((FLpower));
        motorBL.setPower(BLpower);
        //
        // This sets the motors that go into DemoAutonomous.
        //
    }
    public void turn_right(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL) {
        FLpower = BasePower;
        BLpower = BasePower;
        FRpower = BasePower;
        BRpower = BasePower;
        // The positive and negative inputs only mean direction, not speed.
        // This sets the motors that go into DemoAutonomous.
        motorFR.setPower(FRpower);
        motorBR.setPower(BRpower);
        motorFL.setPower(FLpower);
        motorBL.setPower(BLpower);
    }
    public void turn_left(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL) {
        FLpower = -BasePower;
        BLpower = -BasePower;
        FRpower = -BasePower;
        BRpower = -BasePower;
        //
        // The positive and negative inputs only mean direction, not speed.
        // This sets the motors that go into DemoAutonomous.
        //
        motorFR.setPower(FRpower);
        motorBR.setPower(BRpower);
        motorFL.setPower(FLpower);
        motorBL.setPower(BLpower);
    }
    public void brake(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL) {
        FLpower = 0;
        BLpower = 0;
        FRpower = 0;
        BRpower = 0;
        //
        // The positive and negative inputs only mean direction, not speed.
        //
        motorFR.setPower(FRpower);
        motorBR.setPower(BRpower);
        motorFL.setPower(FLpower);
        motorBL.setPower(BLpower);
        //
        // This sets the motors that go into DemoAutonomous.
        //
    }

    // resets
    public void reset_encoders(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL) {
        //
        // The positive and negative inputs only mean direction, not speed.
        //
        motorFR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFL.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBL.setMode(DcMotor.RunMode.RESET_ENCODERS);
        //
        // This sets the motors that go into DemoAutonomous.
        //
    }
    public void run_to_position(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL) {
        //
        // The positive and negative inputs only mean direction, not speed.
        //
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        // This sets the motors that go into DemoAutonomous.
        //
    }
    public void run_using_encoders(DcMotor motorFR, DcMotor motorFL, DcMotor motorBR, DcMotor motorBL) {
        //
        // The positive and negative inputs only mean direction, not speed.
        //
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        //
        // This sets the motors that go into DemoAutonomous.
        //
    }

}