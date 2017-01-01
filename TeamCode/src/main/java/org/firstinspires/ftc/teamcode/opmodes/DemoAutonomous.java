package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.lasarobotics.vision.android.Cameras;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
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
import com.vuforia.ar.pl.DrawOverlayView;

/**
 * Provide a basic autonomous operational mode that uses the left and right
 * drive motors and associated encoders implemented using a state machine for
 * the Push Bot.
 *
 * @author Silicon Edge Robotics
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
    private ModernRoboticsI2cRangeSensor RANGE;

    // Light Sensor
    private OpticalDistanceSensor ods;

    // Sensor Classes
    private Mecanum Drive_Train = new Mecanum();
    //LineFollow ods = new LineFollow();
    private ElapsedTime runtime = new ElapsedTime();


    // Reading for the initial color we take at the beginning of the match.
    // This helps us because when we test for the white line, we want to be
    // able to tell the difference from the color of the ground. Thus
    // knowing where the sensor is.
    private double initialC = 0;
    private double initialD = 0;

    // states variable for the loop
    private int v_state = 0;
    private int ticks = 1440;
    private double WheelC = 3.14*4;

    public DemoAutonomous() {

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

        ods = hardwareMap.opticalDistanceSensor.get("ods_line");
        RANGE = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");

        // Sets Position
        releaseServo.setPosition(0.3);

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

        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);

        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();


        v_state = 0;
        // NOTE: This is for the RIGHT Side
    }

    @Override
    public void start() {
        // start
        super.start();

        // reset encoders to begin period of autonomous
    }

    @Override
    public void loop() {
        //----------------------------------------------------------------------
        //
        // State: Initialize (i.e. state_0).
        //
        // Sets v_state

        super.loop();
        switch (v_state) {
            //
            // Synchronize the state machine and hardware.
            //
            case 0:

                initialC = ods.getLightDetected();
                //Shoots ball for 3 seconds


                releaseServo.setPosition(.9);

                motorShootL.setPower(1.0);
                motorShootR.setPower(-1.0);
                runtime.reset();
                while (runtime.seconds() < 3) {
                    telemetry.addData("seconds", runtime.seconds());
                }
                releaseServo.setPosition(0);
                motorShootL.setPower(0);
                motorShootR.setPower(0);
                v_state++;

                break;
            //
            // Drive forward until the encoders exceed the specified values.
            //
            case 1:
                //Move and detect line
                //Drive_Train.run_to_position(fr, fl, br, bl);
                //Drive_Train.run_using_encoders(fr, fl, br, bl);
                // Drive_Train.setPosition(3* ticks,0,0,3 * ticks, fr, fl, br, bl);

                Drive_Train.setPowerD(1.0);
                Drive_Train.run_diagonal_right_up(fr, fl, br, bl);

                while (fl.isBusy() || ods.getLightDetected()< initialC +.15) {
                    telemetry.addData("Light ",ods.getLightDetected());
                }

                Drive_Train.reset_encoders(fr, fl, br, bl);
                Drive_Train.brake(fr, fl, br, bl);
                v_state++;
                break;
            //
            // Wait...
            //
            // First Button
            case 2:
                // Strafe right
                Drive_Train.setPowerD(1.0);
                Drive_Train.run_right(fr, fl, br, bl);

                while (RANGE.getDistance(DistanceUnit.CM) > 5) {

                    telemetry.addData("Distance",RANGE.getDistance(DistanceUnit.CM ));
                }
                Drive_Train.brake(fr,fl,br,bl);

                v_state++;
                break;
            //
            // Wait...
            //
            case 3:
                // Detect beacon
                runtime.reset();
                while(runtime.seconds()<2){
                    telemetry.addLine("Scanning");
                }
                if (beacon.getAnalysis().isLeftBlue() == true) {
                    //go forward if the left side of the beacon is blue
                    Drive_Train.run_to_position(fr, fl, br, bl);
                    Drive_Train.setPowerD(1.0);
                    Drive_Train.run_forward(fr,fl,br,bl);
                    Drive_Train.setPosition(430,430,430,430, fr, fl, br, bl);

                    Drive_Train.reset_encoders(fr, fl, br, bl);
                    Drive_Train.brake(fr, fl, br, bl);

                    v_state++;

                } else {
                    //go to other side
                    Drive_Train.run_to_position(fr, fl, br, bl);
                    Drive_Train.setPowerD(1.0);
                    Drive_Train.run_backward(fr, fl, br, bl);
                    Drive_Train.setPosition(-430,-430,-430,-430, fr, fl, br, bl);


                    Drive_Train.reset_encoders(fr, fl, br, bl);
                    Drive_Train.brake(fr, fl, br, bl);

                    v_state++;
                }
                // beacon code

                break;
            //
            // Wait...
            //
            case 4:
                //hit the button
                //200 is 1.74 inches
                Drive_Train.run_to_position(fr, fl, br, bl);
                Drive_Train.setPowerD(1.0);
                Drive_Train.run_right(fr, fl, br, bl);
                Drive_Train.setPosition(200,-200,200,-200, fr, fl, br, bl);

                Drive_Train.reset_encoders(fr, fl, br, bl);
                Drive_Train.brake(fr, fl, br, bl);

                v_state++;

                break;
            //
            // Wait...
            //
            case 5:
                //go back a little bit

                Drive_Train.run_to_position(fr, fl, br, bl);
                Drive_Train.setPowerD(1.0);
                Drive_Train.run_left(fr, fl, br, bl);
                Drive_Train.setPosition(-200,200,-200,200, fr, fl, br, bl);

                Drive_Train.reset_encoders(fr, fl, br, bl);
                Drive_Train.brake(fr, fl, br, bl);

                v_state++;
                break;
            //
            // Wait...
            //
            // Second Button
            case 6:
                // run forward again to second line
                Drive_Train.run_using_encoders(fr, fl, br, bl);

                Drive_Train.setPowerD(1.0);
                Drive_Train.run_forward(fr, fl, br, bl);

                while (ods.getLightDetected()> initialC +.1) {

                    telemetry.addData("Light", ods.getLightDetected());
                }
                Drive_Train.reset_encoders(fr, fl, br, bl);
                Drive_Train.brake(fr, fl, br, bl);

                v_state++;
                break;
            //
            // Wait...
            //
            case 7:
                // Strafe right
                Drive_Train.run_using_encoders(fr, fl, br, bl);
                Drive_Train.setPowerD(1.0);
                Drive_Train.run_right(fr, fl, br, bl);

                //Drive_Train.setPosition(-2*ticks,2 * ticks,-2*ticks,2*ticks, fr, fl, br, bl);

                while(RANGE.getDistance(DistanceUnit.CM) >= 5) {
                    telemetry.addData("Distance ", RANGE.getDistance(DistanceUnit.CM)  );
                }

                Drive_Train.reset_encoders(fr, fl, br, bl);
                Drive_Train.brake(fr, fl, br, bl);

                v_state++;
                break;
            //
            // Wait...
            //
            case 8:

                runtime.reset();
                while(runtime.seconds()<2){
                    telemetry.addLine("Scanning");
                }
                if (beacon.getAnalysis().isLeftBlue() == true) {
                    //go forward if the left side of the beacon is blue
                    Drive_Train.run_to_position(fr, fl, br, bl);
                    Drive_Train.setPowerD(1.0);
                    Drive_Train.run_forward(fr,fl,br,bl);
                    Drive_Train.setPosition(430,430,430,430, fr, fl, br, bl);

                    Drive_Train.reset_encoders(fr, fl, br, bl);
                    Drive_Train.brake(fr, fl, br, bl);

                    v_state++;

                } else {
                    //go to other side
                    Drive_Train.run_to_position(fr, fl, br, bl);
                    Drive_Train.setPowerD(1.0);
                    Drive_Train.run_backward(fr, fl, br, bl);
                    Drive_Train.setPosition(-430,-430,-430,-430, fr, fl, br, bl);

                    Drive_Train.reset_encoders(fr, fl, br, bl);
                    Drive_Train.brake(fr, fl, br, bl);

                    v_state++;
                }
                // beacon code

                break;
            //
            // Wait...
            //
            case 9:
                //hit the button
                Drive_Train.run_to_position(fr, fl, br, bl);
                Drive_Train.run_right(fr, fl, br, bl);
                Drive_Train.setPosition(-200,200,200,-200, fr, fl, br, bl);

                Drive_Train.reset_encoders(fr, fl, br, bl);
                Drive_Train.brake(fr, fl, br, bl);

                v_state++;

                break;
            //
            // Wait...
            //
            default:
                stop();
                break;
        }
        telemetry.addData("18", "State: " + v_state);
        // Now the loop repeats if not default.
    }

    @Override
    public void stop() {
        super.stop();
    }
}
