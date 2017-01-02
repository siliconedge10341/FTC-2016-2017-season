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

@Autonomous(name = "Joe_Auto_Linear", group = "Blue")
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
    Servo beaconServo;


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
        beaconServo = hardwareMap.servo.get("servo_beacon");

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
        //Start OpMode


        initialC = ods.getLightDetected();

/*
        int v_state;
        boolean startatcenter = true;
        boolean firetwice = true;
        boolean knockcapball = true;
        boolean pressbeacons = true;
        boolean endincorner = true;
        boolean endincenter = false;

        double posx, posy; // from corner to robot corner;
        if(startatcenter)
        {
            posx = 54.89; //in
            posy = 0.0;
            v_state = 1;
        }
        else
        {
            posx = 90.0; //in
            posy = 0.0;
            v_state = 0;
        }

        switch(v_state)
        {
            case 0: //  case if we don't start at the center; wait for alliance partner to move
                //wait until optical distance sensor registers > 5 feet

                encoderDrive(1492, "forward",1.0);

                posx = 54.89;
                posy = 0.0;
                v_state++;
            case 1: //  case of firing once
                motorShootL.setPower(1.0);
                motorShootR.setPower(-1.0);
                releaseServo.setPosition(0.9);
                runtime.reset();
                while (runtime.seconds() < 2.0) {
                    telemetry.addData("seconds", runtime.seconds());
                    telemetry.update();
                }
                v_state++;
            case 2: //  case of running collector and firing again

            case 3: //  case of strafing to the center

            case 4: //  case of knocking the cap ball over and returning

            case 5: //  case of rotating and strafing to beacons

            case 6: //  case of moving forward until white line is detected

            case 7: //  case of determining beacon color

            case 8: //  case of opposite color is on the left, move backward

            case 9: //  case of pressing beacon button

            case 10://  case of moving to next white line

            case 11://  case of determining beacon color

            case 12://  case of opposite color is on the left, move backward

            case 13://  case of pressing beacon button

            case 14://  case of moving to corner vortex

            case 15://  case of moving to center vortex
        }*/
        //Shoots ball for 3 seconds
        releaseServo.setPosition(.05);

        while (runtime.seconds() < 3) {
            motorShootL.setPower(1.0);
            motorShootR.setPower(-1.0);
        }

        motorShootL.setPower(0);
        motorShootR.setPower(0);

        /////////////////////////////////////////////////////////
        //Turn:
        fr.setPower(1.0);
        fl.setPower(1.0);
        br.setPower(1.0);
        bl.setPower(1.0);
        runtime.reset();
        while (runtime.seconds() < 0.86) {
            telemetry.addData("seconds", runtime.seconds());
            telemetry.update();
        }

        Drive_Train.brake(fr,fl,br,bl);
        //Hit the ball:
        encoderDrive(520 , "backward" , .5);

        //Move and detect line

        Drive_Train.run_using_encoders(fr,fl,br,bl);
        Drive_Train.setPowerD(.15);

        Drive_Train.run_diagonal_right_up(fr, fl, br, bl);

        while (opModeIsActive() && ods.getLightDetected()< initialC +.1) {
            telemetry.addData("Light ",ods.getLightDetected());
            telemetry.update();
        }
        //Drive_Train.reset_encoders(fr, fl, br, bl);
        Drive_Train.brake(fr, fl, br, bl);


        // Strafe right to the wal
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

            encoderDrive(720,"forward",.5);

        } else {
            encoderDrive(720,"backward" , .5);
        }
        // beacon code



        //hit the button

        encoderDrive(200,"right" , .15);

        //go back a little bit
        encoderDrive(200, "left" , .15);

        //Run to line

        Drive_Train.run_forward(fr, fl, br, bl);
        //Drive_Train.setPosition(4 * 1440,4*1440,4*1440,4*1440, fr, fl, br, bl);
        while (opModeIsActive() && ods.getLightDetected() > initialC + .1) {
              telemetry.addData("Light" , ods.getLightDetected());
            telemetry.update();
        }
        Drive_Train.reset_encoders(fr, fl, br, bl);
        Drive_Train.brake(fr, fl, br, bl);
        // Wait...
        //
        //Stop at wall

        Drive_Train.run_using_encoders(fr, fl, br, bl);
        Drive_Train.run_right(fr, fl, br, bl);

        while ( opModeIsActive() && RANGE.getDistance(DistanceUnit.CM) > 5) {
           telemetry.addData("Distance ", RANGE.getDistance(DistanceUnit.CM));
           telemetry.update();
        }
        Drive_Train.reset_encoders(fr, fl, br, bl);
        Drive_Train.brake(fr, fl, br, bl);
        //
        // Wait...
        //
        //Detect beacon
        if (beacon.getAnalysis().isLeftBlue() == true) {
            //go forward if the left side of the beacon is blue
            encoderDrive(720,"forward" , .15);
        } else {
            encoderDrive(720,"backward" , .15);
        }
        //beacon code

        //
        // Wait...
        //

        //hit the button
        encoderDrive(200,"right",.3);


    }


    public void encoderDrive(int encoderval, String direction, double power){
        Drive_Train.run_using_encoders(fr, fl, br, bl);

        Drive_Train.setPosition(encoderval,encoderval,encoderval,encoderval,fr,fl,br,bl);

        Drive_Train.setPowerD(power);
        if (direction == "forward"){
            Drive_Train.run_forward(fr,fl,br,bl);
        }else if(direction == "backward"){
            Drive_Train.run_backward(fr,fl,br,bl);
        }else if (direction == "left"){
            Drive_Train.run_left(fr,fl,br,bl);
        }else if (direction == "right"){
            Drive_Train.run_right(fr,fl,br,bl);
        }else if (direction == "diagonal_left_up"){
            Drive_Train.run_diagonal_left_up(fr,fl,br,bl);
        }

        while(Drive_Train.testDistance(fl) != 1){
            telemetry.addData("Pos " , fl.getCurrentPosition());
            telemetry.update();
        }

        Drive_Train.brake(fr, fl, br, bl);
    }




}
