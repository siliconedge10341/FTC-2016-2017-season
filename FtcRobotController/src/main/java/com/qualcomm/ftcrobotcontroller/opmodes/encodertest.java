package com.qualcomm.ftcrobotcontroller.opmodes;

//------------------------------------------------------------------------------
//
// PushBotAuto
//

import com.qualcomm.ftcrobotcontroller.classes.Mecanum;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Provide a basic autonomous operational mode that uses the left and right
 * drive motors and associated encoders implemented using a state machine for
 * the Push Bot.
 *
 * @author SSI Robotics
 * @version 2015-08-01-06-01
 */
public class encodertest extends PushBotTelemetry

{
    Mecanum Drive_Train = new Mecanum();
    DcMotor fr;
    DcMotor fl;
    DcMotor bl;
    DcMotor br;

    private double pi = 3.1415;
    private double circumfrance = pi*4;

    public encodertest(){

    }

  @Override public void init(){
      fr = hardwareMap.dcMotor.get("fr_motor");
      fl = hardwareMap.dcMotor.get("fl_motor");
      br = hardwareMap.dcMotor.get("br_motor");
      bl = hardwareMap.dcMotor.get("bl_motor");
  }

    @Override public void start ()

    {
        super.start ();

        //
        // Reset the motor encoders on the drive wheels.
        //
        reset_drive_encoders ();

    } // start

    //--------------------------------------------------------------------------

    @Override public void loop ()

    {
        run_using_encoders();
        Drive_Train.run_forward(fr, fl, br, bl);

        if (have_drive_encoders_reached(circumfrance*12,circumfrance*12)){
            reset_drive_encoders();
            Drive_Train.brake(fr,fl,br,bl);
        }
    }
} // PushBotAuto
