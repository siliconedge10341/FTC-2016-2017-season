package com.qualcomm.ftcrobotcontroller.opmodes;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.VisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;
import com.qualcomm.ftcrobotcontroller.classes.LineFollow;

import com.qualcomm.ftcrobotcontroller.classes.Mecanum;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Provide a basic autonomous operational mode that uses the left and right
 * drive motors and associated encoders implemented using a state machine for
 * the Push Bot.
 *
 * @author SSI Robotics
 * @version 2015-08-01-06-01
 */
public class DemoAutonomous extends VisionOpMode

{
    Mecanum Drive_Train = new Mecanum();
    DcMotor fr;
    DcMotor fl;
    DcMotor bl;
    DcMotor br;
    LineFollow ods =new LineFollow();
    TouchSensor ts;

    double initialC;



    public DemoAutonomous(){
        // NOTE: This is for the RIGHT Side

    }

  @Override public void init(){
      fr = hardwareMap.dcMotor.get("fr_motor");
      fl = hardwareMap.dcMotor.get("fl_motor");
      br = hardwareMap.dcMotor.get("br_motor");
      bl = hardwareMap.dcMotor.get("bl_motor");

      //VISION:
      super.init();
      this.setCamera(Cameras.PRIMARY);

      this.setFrameSize(new Size(900, 900));

      enableExtension(VisionOpMode.Extensions.BEACON);         //Beacon detection
      enableExtension(VisionOpMode.Extensions.ROTATION);       //Automatic screen rotation correction
      enableExtension(VisionOpMode.Extensions.CAMERA_CONTROL); //Manual camera control
      beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);

      beacon.setColorToleranceRed(0);
      beacon.setColorToleranceBlue(0);

      cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
      cameraControl.setAutoExposureCompensation();

      initialC = ods.getVal();
  }

    @Override public void start ()

    {
        super.start ();

        Drive_Train.reset_encoders(fr, fl, br, bl);
    } // start

    //--------------------------------------------------------------------------

    @Override public void loop ()

    {
        //----------------------------------------------------------------------
        //
        // State: Initialize (i.e. state_0).
        //

        super.loop();
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
            case 1:     //Move and detect line
                Drive_Train.run_using_encoders(fr, fl, br, bl);

                Drive_Train.setPower(.3);
                Drive_Train.run_diagonal_right_up(fr, fl, br, bl);
                Drive_Train.setPosition(3*1440,fr, fl, br, bl);

                if (Drive_Train.testDistance(fr, fl, br, bl) == 1 || (ods.getVal() > initialC + .1)) {
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
                //Strafe right

                Drive_Train.run_using_encoders(fr, fl, br, bl);
                Drive_Train.run_right(fr, fl, br, bl);
                Drive_Train.setPosition(2*1440,fr, fl, br, bl);
                if (Drive_Train.testDistance(fr, fl, br, bl) == 1 ||  ts.isPressed() == true) {

                    Drive_Train.reset_encoders(fr, fl, br, bl);
                    Drive_Train.brake(fr, fl, br, bl);
                    v_state++;
                }
                break;
            //
            // Wait...
            //
            case 3:
                //Detect beacon
                if(beacon.getAnalysis().isLeftBlue() == true){
                    //go forward if the left side of the beacon is blue
                    Drive_Train.setPosition(720,fr, fl, br, bl);
                    Drive_Train.run_using_encoders(fr, fl, br, bl);
                    Drive_Train.run_forward(fr, fl, br, bl);
                    if (Drive_Train.testDistance(fr, fl, br, bl) == 1) {

                        Drive_Train.reset_encoders(fr, fl, br, bl);
                        Drive_Train.brake(fr, fl, br, bl);

                    }

                }else{
                    Drive_Train.setPosition(720,fr, fl, br, bl);
                    Drive_Train.run_using_encoders(fr, fl, br, bl);
                    Drive_Train.run_backward(fr, fl, br, bl);
                    if (Drive_Train.testDistance(fr, fl, br, bl) == 1) {

                        Drive_Train.reset_encoders(fr, fl, br, bl);
                        Drive_Train.brake(fr, fl, br, bl);

                    }
                }
                //beacon code

                v_state++;
                break;

            case 4:
                //hit the button
                Drive_Train.setPosition(200,fr, fl, br, bl);
                Drive_Train.run_using_encoders(fr, fl, br, bl);
                Drive_Train.run_right(fr, fl, br, bl);
                if (Drive_Train.testDistance(fr, fl, br, bl) == 1) {
                    //if reached then stop
                    Drive_Train.reset_encoders(fr, fl, br, bl);
                    Drive_Train.brake(fr, fl, br, bl);

                    v_state++;
                }
                break;

            case 5:
                //go back a little bit
                Drive_Train.setPosition(200,fr, fl, br, bl);
                Drive_Train.run_using_encoders(fr, fl, br, bl);
                Drive_Train.run_left(fr, fl, br, bl);
                if (Drive_Train.testDistance(fr, fl, br, bl) == 1) {
                    //if reached then stop
                    Drive_Train.reset_encoders(fr, fl, br, bl);
                    Drive_Train.brake(fr, fl, br, bl);

                    v_state++;
                }
                break;
//Second part
            case 6:
                // run forward again to second line
                Drive_Train.run_using_encoders(fr, fl, br, bl);

                Drive_Train.run_forward(fr, fl, br, bl);
                Drive_Train.setPosition(4*1440,fr, fl, br, bl);

                if (Drive_Train.testDistance(fr, fl, br, bl) == 1 ||  (ods.getVal() > initialC + .1)) {
                    //if reached then stop
                    Drive_Train.reset_encoders(fr, fl, br, bl);
                    Drive_Train.brake(fr, fl, br, bl);
                    v_state++;
                }
                break;
            //
            // Wait...
            //
            case 7:
                //Strafe right

                Drive_Train.run_using_encoders(fr, fl, br, bl);
                Drive_Train.run_right(fr, fl, br, bl);
                Drive_Train.setPosition(2*1440,fr, fl, br, bl);
                if (Drive_Train.testDistance(fr, fl, br, bl) == 1 ||  ts.isPressed() == true) {

                    Drive_Train.reset_encoders(fr, fl, br, bl);
                    Drive_Train.brake(fr, fl, br, bl);
                    v_state++;
                }
                break;
            //
            // Wait...
            //
            case 8:
                //Detect beacon
                if(beacon.getAnalysis().isLeftBlue() == true){
                    //go forward if the left side of the beacon is blue
                    Drive_Train.setPosition(720,fr, fl, br, bl);
                    Drive_Train.run_using_encoders(fr, fl, br, bl);
                    Drive_Train.run_forward(fr, fl, br, bl);
                    if (Drive_Train.testDistance(fr, fl, br, bl) == 1) {

                        Drive_Train.reset_encoders(fr, fl, br, bl);
                        Drive_Train.brake(fr, fl, br, bl);

                    }
                }else{
                    Drive_Train.setPosition(720,fr, fl, br, bl);
                    Drive_Train.run_using_encoders(fr, fl, br, bl);
                    Drive_Train.run_backward(fr, fl, br, bl);
                    if (Drive_Train.testDistance(fr, fl, br, bl) == 1) {

                        Drive_Train.reset_encoders(fr, fl, br, bl);
                        Drive_Train.brake(fr, fl, br, bl);
                    }
                }
                //beacon code

                v_state++;
                break;

            case 9:
                //hit the button
                Drive_Train.setPosition(200,fr, fl, br, bl);
                Drive_Train.run_using_encoders(fr, fl, br, bl);
                Drive_Train.run_right(fr, fl, br, bl);
                if (Drive_Train.testDistance(fr, fl, br, bl) == 1) {
                    //if reached then stop
                    Drive_Train.reset_encoders(fr, fl, br, bl);
                    Drive_Train.brake(fr, fl, br, bl);

                    v_state++;
                }
                break;
            //
            // Wait...
            //

            default:

                break;
        }

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
