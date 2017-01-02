package org.firstinspires.ftc.teamcode.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.lasarobotics.vision.android.Cameras;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
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

@Autonomous(name = "Red_Auto", group = "Blue")
public class DemoAuto_Linear extends LinearVisionOpMode {
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
    ModernRoboticsI2cRangeSensor RANGE;
    OpticalDistanceSensor ods;

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


    public void runOpMode() throws InterruptedException {
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

        waitForVisionStart();

        //VISION:
        this.setCamera(Cameras.PRIMARY);
        this.setFrameSize(new Size(900, 900));

        enableExtension(LinearVisionOpMode.Extensions.BEACON);         //Beacon detection
        enableExtension(LinearVisionOpMode.Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(LinearVisionOpMode.Extensions.CAMERA_CONTROL); //Manual camera control
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);

        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);

        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);

        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();

        waitForStart();


        initialC = ods.getLightDetected();
        //Shoots ball for 3 seconds
        releaseServo.setPosition(.05);

        Drive_Train.setPowerD(0.4);
        Drive_Train.setPosition(720,720,720,720,fr, fl, br, bl);
        Drive_Train.run_to_position(fr, fl, br, bl);

        while (runtime.seconds() < 3) {
            motorShootL.setPower(1.0);
            motorShootR.setPower(-1.0);
        }

        motorShootL.setPower(0);
        motorShootR.setPower(0);

        /////////////////////////////////////////////////////////
        //Turn:

        //Move and detect line

        Drive_Train.run_using_encoders(fr,fl,br,bl);
        Drive_Train.setPowerD(.15);

        Drive_Train.run_diagonal_right_up(fr, fl, br, bl);

        while (opModeIsActive() && ods.getLightDetected()< initialC +.1) {
            telemetry.addData("Light ",ods.getLightDetected());

        }
        //Drive_Train.reset_encoders(fr, fl, br, bl);
        Drive_Train.brake(fr, fl, br, bl);


        // Strafe right
        Drive_Train.setPowerD(.5);
        Drive_Train.run_right(fr, fl, br, bl);
        while (RANGE.getDistance(DistanceUnit.CM) > 5) {

            telemetry.addData("Distance",RANGE.getDistance(DistanceUnit.CM ));
        }
        Drive_Train.brake(fr,fl,br,bl);
        Drive_Train.reset_encoders(fr,fl,br,bl);

        // Detect beacon
        if (beacon.getAnalysis().isLeftBlue() == true) {
            //go forward if the left side of the beacon is blue
            Drive_Train.run_using_encoders(fr, fl, br, bl);
            Drive_Train.setPowerD(.5);
            Drive_Train.setPosition(720,720,720,720, fr, fl, br, bl);

           // Drive_Train.run_forward(fr, fl, br, bl);
            while (Drive_Train.testDistance(fl) != 1){
                telemetry.addData("Encoder ", fl.getCurrentPosition());
            }
                // motor to move button here
                Drive_Train.reset_encoders(fr, fl, br, bl);
                Drive_Train.brake(fr, fl, br, bl);

        } else {
            Drive_Train.setPosition(-720,-720,-720,-720, fr, fl, br, bl);
            Drive_Train.run_to_position(fr, fl, br, bl);
            Drive_Train.run_backward(fr, fl, br, bl);
            if (Drive_Train.testDistance(fl) == 1) {
                // motor to move button here
                Drive_Train.reset_encoders(fr, fl, br, bl);
                Drive_Train.brake(fr, fl, br, bl);

            }
        }
        // beacon code
        v_state++;


        //hit the button
        Drive_Train.setPosition(200,-200,200,-200, fr, fl, br, bl);
        Drive_Train.run_using_encoders(fr, fl, br, bl);
        Drive_Train.run_right(fr, fl, br, bl);
        if (Drive_Train.testDistance(fl) == 1) {
            //if reached then stop
            Drive_Train.reset_encoders(fr, fl, br, bl);
            Drive_Train.brake(fr, fl, br, bl);

            v_state++;
        }

        //go back a little bit
        Drive_Train.setPosition(-200,200,-200,200, fr, fl, br, bl);
        Drive_Train.run_to_position(fr, fl, br, bl);
        Drive_Train.run_left(fr, fl, br, bl);
        if (Drive_Train.testDistance(fr) == 1) {
            //if reached then stop
            Drive_Train.reset_encoders(fr, fl, br, bl);
            Drive_Train.brake(fr, fl, br, bl);

            v_state++;
        }


        // run forward again to second line
        Drive_Train.run_using_encoders(fr, fl, br, bl);

        Drive_Train.run_forward(fr, fl, br, bl);
        Drive_Train.setPosition(4 * 1440,4*1440,4*1440,4*1440, fr, fl, br, bl);
        while (fl.isBusy()) {
            if (Drive_Train.testDistance(fl) == 1 || (ods.getLightDetected() > initialC + .1)) {
                //if reached then stop
                Drive_Train.reset_encoders(fr, fl, br, bl);
                Drive_Train.brake(fr, fl, br, bl);
                v_state++;
            }
        }

        // Wait...
        //

        Drive_Train.run_using_encoders(fr, fl, br, bl);
        Drive_Train.run_right(fr, fl, br, bl);
        Drive_Train.setPosition(-2*1440,2 * 1440,-2*1440,2*1440, fr, fl, br, bl);

        if (Drive_Train.testDistance(fl) == 1 || RANGE.getDistance(DistanceUnit.CM) <= 5) {

            Drive_Train.reset_encoders(fr, fl, br, bl);
            Drive_Train.brake(fr, fl, br, bl);
            v_state++;
        }

        //
        // Wait...
        //
        //Detect beacon
        if (beacon.getAnalysis().isLeftBlue() == true) {
            //go forward if the left side of the beacon is blue
            Drive_Train.setPosition(720,720,720,720, fr, fl, br, bl);
            Drive_Train.run_to_position(fr, fl, br, bl);
            Drive_Train.run_forward(fr, fl, br, bl);
            if (Drive_Train.testDistance(fl) == 1) {

                Drive_Train.reset_encoders(fr, fl, br, bl);
                Drive_Train.brake(fr, fl, br, bl);
            }
        } else {
            Drive_Train.setPosition(-720,-720,-720,-720, fr, fl, br, bl);
            Drive_Train.run_to_position(fr, fl, br, bl);
            Drive_Train.run_backward(fr, fl, br, bl);
            if (Drive_Train.testDistance(fl) == 1) {

                Drive_Train.reset_encoders(fr, fl, br, bl);
                Drive_Train.brake(fr, fl, br, bl);
            }
        }
        //beacon code

        //
        // Wait...
        //

        //hit the button
        Drive_Train.setPosition(-200,200,200,-200, fr, fl, br, bl);
        Drive_Train.run_using_encoders(fr, fl, br, bl);
        Drive_Train.run_right(fr, fl, br, bl);
        if (Drive_Train.testDistance(fl) == 1) {
            //if reached then stop
            Drive_Train.reset_encoders(fr, fl, br, bl);
            Drive_Train.brake(fr, fl, br, bl);

            v_state++;
        }





    }




}
