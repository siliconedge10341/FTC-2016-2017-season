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
public class DemoAutonomous extends PushBotTelemetry

{
    Mecanum Drive_Train = new Mecanum();
    DcMotor fr;
    DcMotor fl;
    DcMotor bl;
    DcMotor br;

    public DemoAutonomous(){
        fr.setPower(0.0);
        fl.setPower(0.0);
        bl.setPower(0.0);
        br.setPower(0.0);
        Drive_Train.set_Power(0, 0, 0, 0);
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
        //----------------------------------------------------------------------
        //
        // State: Initialize (i.e. state_0).
        //
        switch (v_state) {
            //
            // Synchronize the state machine and hardware.
            //
            case 0:

                v_state++;

                break;
            //
            // Drive forward until the encoders exceed the specified values.
            //
            case 1:
                Drive_Train.run_using_encoders(fr, fl, br, bl);

                Drive_Train.run_forward(fr, fl, br, bl);

                if (Drive_Train.testDistance(fr, fl, br, bl) == 1) {
                    //
                    // Reset the encoders to ensure they are at a known good value.
                    //
                    Drive_Train.reset_encoders(fr, fl, br, bl);


                    Drive_Train.brake(fr, fl, br, bl);


                    v_state++;
                }
                break;
            //
            // Wait...
            //
            case 2:
                if (have_drive_encoders_reset()) {
                    v_state++;
                }
                break;

            case 3:
                Drive_Train.run_using_encoders(fr, fl, br, bl);
                Drive_Train.run_left(fr, fl, br, bl);
                Drive_Train.setPosition(2880,fr, fl, br, bl);
                if (Drive_Train.testDistance(fr, fl, br, bl) == 1) {
                    Drive_Train.reset_encoders(fr, fl, br, bl);
                    Drive_Train.brake(fr, fl, br, bl);
                    v_state++;
                }
                break;
            //
            // Wait...
            //
            case 4:
                if (have_drive_encoders_reset()) {
                    v_state++;
                }
                break;
            //
            // Turn right until the encoders exceed the specified values.
            //
            case 5:
                Drive_Train.run_using_encoders(fr, fl, br, bl);

                Drive_Train.run_forward(fr, fl, br, bl);
                Drive_Train.setPosition(4080,fr, fl, br, bl);
                if (Drive_Train.testDistance(fr, fl, br, bl) == 1) {
                    Drive_Train.reset_encoders(fr, fl, br, bl);
                    Drive_Train.brake(fr, fl, br, bl);
                    v_state++;
                }
                break;
            //
            // Wait...
            //
            case 6:
                if (have_drive_encoders_reset()) {
                    v_state++;
                }
                break;

            default:

                break;
        }
        update_telemetry (); // Update common telemetry
        telemetry.addData ("18", "State: " + v_state);

    } // loop

    //--------------------------------------------------------------------------
    //
    // v_state
    //
    /**
     * This class member remembers which state is currently active.  When the
     * start method is called, the state will be initialized (0).  When the loop
     * starts, the state will change from initialize to state_1.  When state_1
     * actions are complete, the state will change to state_2.  This implements
     * a state machine for the loop method.
     */
    private int v_state = 0;

} // PushBotAuto
