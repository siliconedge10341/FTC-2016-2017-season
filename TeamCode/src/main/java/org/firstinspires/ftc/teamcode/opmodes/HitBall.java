package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.classes.LineFollow;
import org.firstinspires.ftc.teamcode.classes.Range;
import org.firstinspires.ftc.teamcode.classes.Mecanum;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.lasarobotics.vision.opmode.VisionOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Provide a basic autonomous operational mode that uses the left and right
 * drive motors and associated encoders implemented using a state machine for
 * the Push Bot.
 *
 * @author SSI Robotics
 * @version 2015-08-01-06-01
 */

@Autonomous(name = "Hitball", group = "Blue")
public class HitBall extends VisionOpMode {
    // instance variables
    // private variables
    // Motors
    DcMotor fr;
    DcMotor fl;
    DcMotor bl;
    DcMotor br;
    DcMotor motorShootL;
    DcMotor motorShootR;
    Servo releaseServo;


    // Range Sensor


    // Sensor Classes
    Mecanum Drive_Train = new Mecanum();
    //LineFollow ods = new LineFollow();
    ElapsedTime runtime = new ElapsedTime();


    // Reading for the initial color we take at the beginning of the match.
    // This helps us because when we test for the white line, we want to be
    // able to tell the difference from the color of the ground. Thus
    // knowing where the sensor is.
    double initialC = 0;

    // states variable for the loop
    int v_state = 0;

    public HitBall() {

    }

    @Override
    public void init() {
        // Sets every class at the beginning of the demoautonomous run class
        //Hardware Maps
        fr = hardwareMap.dcMotor.get("fr_motor");
        fl = hardwareMap.dcMotor.get("fl_motor");
        br = hardwareMap.dcMotor.get("br_motor");
        bl = hardwareMap.dcMotor.get("bl_motor");

        motorShootL = hardwareMap.dcMotor.get("shooter_left");
        motorShootR = hardwareMap.dcMotor.get("shooter_right");
        releaseServo = hardwareMap.servo.get("servo_ball");


        // Sets Position
        releaseServo.setPosition(0.3);

        //VISION:
        v_state = 0;
        // NOTE: This is for the RIGHT Side
    }

    @Override
    public void start() {
        // start
        super.start();

        // reset encoders to begin period of autonomous
        Drive_Train.reset_encoders(fr, fl, br, bl);
    }

    @Override
    public void loop() {
        //----------------------------------------------------------------------
        //
        // State: Initialize (i.e. state_0).
        //
        // Sets v_state
        v_state = 0;


        super.loop();
        switch (v_state) {
            //
            // Synchronize the state machine and hardware.
            //
            case 0:

                //Shoots ball for 3 seconds

                releaseServo.setPosition(.9);

                motorShootL.setPower(1.0);
                motorShootR.setPower(-1.0);
                runtime.reset();
                while (runtime.seconds() < 3) {
                    telemetry.addData("seconds", runtime.seconds());
                }
                motorShootL.setPower(0);
                motorShootR.setPower(0);
                v_state++;

                break;
            //
            // Drive forward until the encoders exceed the specified values.
            //
            case 1:
                //Move and detect line
                Drive_Train.run_to_position(fr, fl, br, bl);
                Drive_Train.setPowerD(.3);
                Drive_Train.run_forward(fr,fl,br,bl);
                Drive_Train.setPosition(5600,5600,5600,5600, fr, fl, br, bl);


                Drive_Train.reset_encoders(fr, fl, br, bl);
                Drive_Train.run_using_encoders(fr, fl, br, bl);
                Drive_Train.brake(fr, fl, br, bl);
                v_state++;
                break;
            //
            default:
                break;
        }
        telemetry.addData("18", "State: " + v_state);
        // Now the loop repeats if not default.
    }
    @Override
    public void stop(){
        super.stop();
    }
}
