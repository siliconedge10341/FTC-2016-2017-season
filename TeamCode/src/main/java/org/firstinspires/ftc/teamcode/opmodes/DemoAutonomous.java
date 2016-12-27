package org.firstinspires.ftc.teamcode.opmodes;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.VisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;
import org.firstinspires.ftc.teamcode.classes.LineFollow;
import org.firstinspires.ftc.teamcode.classes.Range;
import org.firstinspires.ftc.teamcode.classes.Mecanum;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
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

@Autonomous(name = "Blue_Auto", group = "Blue")
public class DemoAutonomous extends VisionOpMode {
    // instance variables
    // private variables
    // Motors
    private DcMotor fr;
    private DcMotor fl;
    private DcMotor bl;
    private DcMotor br;
    private DcMotor motorShootL;
    private DcMotor motorShootR;
    private Servo releaseServo;

    // Range Sensor
    private ModernRoboticsI2cRangeSensor rangeSensor;

    // Sensor Classes
    private Mecanum Drive_Train = new Mecanum();
    private LineFollow ods = new LineFollow();
    private ElapsedTime runtime = new ElapsedTime();
    private Range RANGE = new Range();

    // Reading for the initial color we take at the beginning of the match.
    // This helps us because when we test for the white line, we want to be
    // able to tell the difference from the color of the ground. Thus
    // knowing where the sensor is.
    double initialC = 0;

    // states variable for the loop
    private int v_state = 0;

    public DemoAutonomous(){
        // NOTE: This is for the RIGHT Side
        initialC = 0;
        v_state = 0;
    }

  @Override public void init(){
      // Sets every class at the beginning of the demoautonomous run class
      fr = hardwareMap.dcMotor.get("fr_motor");
      fl = hardwareMap.dcMotor.get("fl_motor");
      br = hardwareMap.dcMotor.get("br_motor");
      bl = hardwareMap.dcMotor.get("bl_motor");

      motorShootL = hardwareMap.dcMotor.get("shooter_left");
      motorShootR = hardwareMap.dcMotor.get("shooter_right");

      rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
      RANGE.setRange(rangeSensor);

      //VISION:
      super.init();
      this.setCamera(Cameras.SECONDARY);
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

    @Override public void start () {
        // start
        super.start ();
        v_state = 0;
        // reset encoders to begin period of autonomous
        Drive_Train.reset_encoders(fr, fl, br, bl);
    }

    @Override public void loop () {
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
                runtime.reset();
                motorShootL.setPower(1.0);
                motorShootR.setPower(-1.0);
                if (runtime.seconds() > 3) {
                    motorShootL.setPower(0);
                    motorShootR.setPower(0);
                    v_state++;
                }
                break;
            //
            // Drive forward until the encoders exceed the specified values.
            //
            case 1:
                //Move and detect line
                Drive_Train.run_using_encoders(fr, fl, br, bl);

                Drive_Train.setPower(.3);
                Drive_Train.run_diagonal_right_up(fr, fl, br, bl);
                Drive_Train.setPosition(3*1440,fr, fl, br, bl);
                while(fl.isBusy()) {
                    if (Drive_Train.testDistance(fr) == 1 || (ods.getVal() > initialC + .1)) {
                        //
                        // Reset the encoders to ensure they are at a known good value.
                        //
                        Drive_Train.reset_encoders(fr, fl, br, bl);
                        Drive_Train.brake(fr, fl, br, bl);
                        v_state++;
                    }
                }
                break;
            //
            // Wait...
            //
            // First Button
            case 2:
                // Strafe right
                Drive_Train.run_using_encoders(fr, fl, br, bl);
                Drive_Train.run_right(fr, fl, br, bl);
                Drive_Train.setPosition(2*1440,fr, fl, br, bl);
                while(fl.isBusy()) {
                    if (Drive_Train.testDistance(fr) == 1 || RANGE.getData() <= 5) {

                        Drive_Train.reset_encoders(fr, fl, br, bl);
                        Drive_Train.brake(fr, fl, br, bl);
                        v_state++;
                    }
                }
                break;
            //
            // Wait...
            //
            case 3:
                // Detect beacon
                if(beacon.getAnalysis().isLeftBlue() == true){
                    //go forward if the left side of the beacon is blue
                    Drive_Train.setPosition(720,fr, fl, br, bl);
                    Drive_Train.run_using_encoders(fr, fl, br, bl);
                    Drive_Train.run_forward(fr, fl, br, bl);
                    if (Drive_Train.testDistance(fr) == 1) {
                        // motor to move button here
                        Drive_Train.reset_encoders(fr, fl, br, bl);
                        Drive_Train.brake(fr, fl, br, bl);

                    }
                } else {
                    Drive_Train.setPosition(720,fr, fl, br, bl);
                    Drive_Train.run_using_encoders(fr, fl, br, bl);
                    Drive_Train.run_backward(fr, fl, br, bl);
                    if (Drive_Train.testDistance(fr) == 1) {
                        // motor to move button here
                        Drive_Train.reset_encoders(fr, fl, br, bl);
                        Drive_Train.brake(fr, fl, br, bl);

                    }
                }
                // beacon code
                v_state++;
                break;
            //
            // Wait...
            //
            case 4:
                //hit the button
                Drive_Train.setPosition(200,fr, fl, br, bl);
                Drive_Train.run_using_encoders(fr, fl, br, bl);
                Drive_Train.run_right(fr, fl, br, bl);
                if (Drive_Train.testDistance(fr) == 1) {
                    //if reached then stop
                    Drive_Train.reset_encoders(fr, fl, br, bl);
                    Drive_Train.brake(fr, fl, br, bl);

                    v_state++;
                }
                break;
            //
            // Wait...
            //
            case 5:
                //go back a little bit
                Drive_Train.setPosition(200,fr, fl, br, bl);
                Drive_Train.run_using_encoders(fr, fl, br, bl);
                Drive_Train.run_left(fr, fl, br, bl);
                if (Drive_Train.testDistance(fr) == 1) {
                    //if reached then stop
                    Drive_Train.reset_encoders(fr, fl, br, bl);
                    Drive_Train.brake(fr, fl, br, bl);

                    v_state++;
                }
                break;
            //
            // Wait...
            //
            // Second Button
            case 6:
                // run forward again to second line
                Drive_Train.run_using_encoders(fr, fl, br, bl);

                Drive_Train.run_forward(fr, fl, br, bl);
                Drive_Train.setPosition(4*1440,fr, fl, br, bl);

                if (Drive_Train.testDistance(fr) == 1 ||  (ods.getVal() > initialC + .1)) {
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
                // Strafe right
                Drive_Train.run_using_encoders(fr, fl, br, bl);
                Drive_Train.run_right(fr, fl, br, bl);
                Drive_Train.setPosition(2*1440,fr, fl, br, bl);

                if (Drive_Train.testDistance(fr) == 1 || RANGE.getData() <= 5 ) {

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
                    if (Drive_Train.testDistance(fr) == 1) {

                        Drive_Train.reset_encoders(fr, fl, br, bl);
                        Drive_Train.brake(fr, fl, br, bl);
                    }
                }else{
                    Drive_Train.setPosition(720,fr, fl, br, bl);
                    Drive_Train.run_using_encoders(fr, fl, br, bl);
                    Drive_Train.run_backward(fr, fl, br, bl);
                    if (Drive_Train.testDistance(fr) == 1) {

                        Drive_Train.reset_encoders(fr, fl, br, bl);
                        Drive_Train.brake(fr, fl, br, bl);
                    }
                }
                //beacon code

                v_state++;
                break;
            //
            // Wait...
            //
            case 9:
                //hit the button
                Drive_Train.setPosition(200,fr, fl, br, bl);
                Drive_Train.run_using_encoders(fr, fl, br, bl);
                Drive_Train.run_right(fr, fl, br, bl);
                if (Drive_Train.testDistance(fr) == 1) {
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
                System.exit(0);
                break;
        }
        telemetry.addData ("18", "State: " + v_state);
        // Now the loop repeats if not default.
    }
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
}
