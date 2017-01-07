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

@Autonomous(name = "Pls help", group = "Blue")
public class ExperementalAuto extends LinearVisionOpMode {
    // instance variables
    // private variables
    // Motors
    private DcMotor fr;
    private DcMotor fl;
    private DcMotor bl;
    private DcMotor br;
    private DcMotor motorShootL;
    private DcMotor motorShootR;
    private DcMotor motorCollector;

    // Servos
    private Servo releaseServo;
    private Servo beaconServo;

    // sensors
    private ModernRoboticsI2cRangeSensor rangef;
    private ModernRoboticsI2cRangeSensor ranges;
    private OpticalDistanceSensor ods;

    // Classes
    private Mecanum Drive_Train = new Mecanum();
    private ElapsedTime runtime = new ElapsedTime();

    // Checks
    private boolean startatcenter = true;
    private boolean firetwice = true;
    private boolean knockcapball = true;
    private boolean pressbeacons = true;
    private boolean endincorner = true;
    private boolean endincenter = false;
    private static final Double ticks_per_inch = 510 / (3.1415 * 4);
    private static final int ENCODER_TICKS = 1440;
    private static final double Wheel_Circumfrence = 3.14*4;

    private static final int Max_Shooter_Speed = 1784;
    private static final int Safe_Shooter_Speed  = Max_Shooter_Speed - 100;


    // Reading for the initial color we take at the beginning of the match.
    // This helps us because when we test for the white line, we want to be
    // able to tell the difference from the color of the ground. Thus
    // knowing where the sensor is.
    double initialC = 0;
    int encoderval;

    // states variable for the loop
    int v_state = 0;
    private double inches;

    // public data

    // Constructors
    public ExperementalAuto() {
        // Default Constructor

    }

    // Run
    public void runOpMode() throws InterruptedException {
        // Sets every class at the beginning of the demoautonomous run class
        // hardware maps
        // motors
        fr = hardwareMap.dcMotor.get("fr_motor");
        fl = hardwareMap.dcMotor.get("fl_motor");
        br = hardwareMap.dcMotor.get("br_motor");
        bl = hardwareMap.dcMotor.get("bl_motor");
        motorShootL = hardwareMap.dcMotor.get("shooter_left");
        motorShootR = hardwareMap.dcMotor.get("shooter_right");
        motorCollector = hardwareMap.dcMotor.get("ball_collector");

        // servos
        releaseServo = hardwareMap.servo.get("servo_ball");
        beaconServo = hardwareMap.servo.get("servo_beacon");

        // sensors
        ods = hardwareMap.opticalDistanceSensor.get("ods_line");
        rangef = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_front");
        ranges = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_side");

        // Sets Position
        releaseServo.setPosition(0.3);
        beaconServo.setPosition(.5);

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

        encoderDrive(12.0,"forward",1.0);


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
        while (opModeIsActive()) {
            switch (v_state) {
                case 0: //  case if we don't start at the center; wait for alliance partner to move
                    //
                    // wait until range sensor registers > 5 feet
                    //
                    while (rangef.getDistance(DistanceUnit.CM) < 152) {
                    }

                    encoderDrive(35.0, "forward", 1.0);

                    posx = 54.89;
                    posy = 0.0;
                    v_state++;
                case 1: //  case of firing once
                    //Shoots ball for 2 seconds
                    double leftMotorCalculatedPower = 0;
                    double rightMotorCalculatedPower = 0;

                    double leftMotorRPM =  0;
                    double rightMotorRPM = 0;
                    motorShootL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorShootR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorShootL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorShootR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    do{
                        if(leftMotorCalculatedPower <= 1){
                            leftMotorCalculatedPower = leftMotorCalculatedPower + .05;
                            motorShootL.setPower(leftMotorCalculatedPower);
                            telemetry.addData("Left Shooter Power:",leftMotorCalculatedPower);
                            telemetry.update();
                        }
                        else{
                            telemetry.addData("Left Motor:"," warning: max power reached");
                            telemetry.update();
                         //   break;
                        }

                        this.Wait(10);
                        int initialEncoderValue = motorShootL.getCurrentPosition();
                        this.Wait(10);
                        int DistanceTraveled = motorShootL.getCurrentPosition() - initialEncoderValue;
                        double RotationsPerMillisecond = ENCODER_TICKS * (DistanceTraveled/10);
                        double RotationsPerMinute = RotationsPerMillisecond * 1000 * 60;
                        telemetry.addData("Left Shooter RPM:", RotationsPerMinute);
                        telemetry.update();
                    }while(leftMotorRPM < Safe_Shooter_Speed);

                    v_state++;
                case 2: //  case of running collector and firing again
                    releaseServo.setPosition(.3);
                    motorCollector.setPower(0.9);
                    runtime.reset();
                    while (runtime.seconds() < 4.0) {
                        telemetry.addData("seconds", runtime.seconds());
                        telemetry.update();
                        if (runtime.seconds() > 2.0) {
                            releaseServo.setPosition(.05);
                            motorShootL.setPower(1.0);
                            motorShootR.setPower(-1.0);
                        }
                    }
                    motorCollector.setPower(0);
                    releaseServo.setPosition(.3);
                    motorShootL.setPower(0.0);
                    motorShootR.setPower(0.0);

                    if (knockcapball) {
                        v_state += 2;
                    } else {
                        v_state++;
                    }
                case 3:
                    //
                    //  case of strafing to the center (27 in)
                    //
                    encoderDrive(1, "left", 0.9);
                    posx = 54.89;
                    posy = 27;
                    v_state = 5;

                case 4: //  case of knocking the cap ball over and returning (35 in and back 8 in)
                    encoderDrive(35.0, "left", 0.9);
                    posx = 54.89;
                    posy = 35.0;
                    PauseAuto(0.3);
                    encoderDrive(8.0, "right", 0.9);
                    posx = 54.89;
                    posy = 27;
                    v_state = 5;

                case 5: //  case of rotating and strafing to beacons
                    PauseAuto(0.3);
                    //rotation
                    fr.setPower(1.0);
                    fl.setPower(1.0);
                    br.setPower(1.0);
                    bl.setPower(1.0);
                    runtime.reset();
                    while (runtime.seconds() < 0.86) {
                        telemetry.addData("seconds", runtime.seconds());
                        telemetry.update();
                    }
                    Drive_Train.brake(fr, fl, br, bl);
                    PauseAuto(0.3);
                    //strafe 30 in right

                    encoderDrive(12.0, "right", 0.9);
                    posx = 24.89;
                    posy = 27;

                    PauseAuto(0.5);
                    //backward 12 in

                    encoderDrive(12.89, "backward", 0.9);
                    posx = 24.89;
                    posy = 39;

                    PauseAuto(0.3);
                    //strafe 12.89 in right

                    Drive_Train.setPowerD(0.6);
                    Drive_Train.run_right(fr, fl, br, bl);
                    while (opModeIsActive() && ranges.getDistance(DistanceUnit.CM) > 30) {

                        telemetry.addData("Distance ", ranges.getDistance(DistanceUnit.CM));
                    }
                    Drive_Train.brake(fr, fl, br, bl);

                    posx = 12;
                    posy = 39;
                    PauseAuto(0.3);

                case 6: //  case of moving forward until white line is detected
                    Drive_Train.setPowerD(0.6);
                    Drive_Train.run_backward(fr, fl, br, bl);

                    while (opModeIsActive() && ods.getLightDetected() < initialC + .1) {
                        telemetry.addData("Light ", ods.getLightDetected());
                        telemetry.update();
                    }
                    Drive_Train.brake(fr, fl, br, bl);

                    v_state++;
                case 7: //  case of pressing beacon button
                    if (beacon.getAnalysis().isLeftBlue()) {
                        encoderDrive(12.0, "right", 0.5);
                        PauseAuto(0.2);

                        Drive_Train.setPowerD(0.6);
                        Drive_Train.run_left(fr, fl, br, bl);
                        while (opModeIsActive() && ranges.getDistance(DistanceUnit.CM) > 30) {

                            telemetry.addData("Distance ", ranges.getDistance(DistanceUnit.CM));
                        }
                        Drive_Train.brake(fr, fl, br, bl);
                    } else if (beacon.getAnalysis().isRightRed()) {
                        //beacon was correctly pressed
                    } else if (beacon.getAnalysis().isLeftRed()){
                        //go forward if the left side of the beacon is blue

                        //beacon is 1/2 a foot, presser is on the right side so it is lined up with the line
                        encoderDrive(6.0, "forward", .5);

                        encoderDrive(12.0, "right", 0.5);
                        PauseAuto(0.2);

                        Drive_Train.setPowerD(0.6);
                        Drive_Train.run_left(fr, fl, br, bl);
                        while (opModeIsActive() && ranges.getDistance(DistanceUnit.CM) > 30) {

                            telemetry.addData("Distance ", ranges.getDistance(DistanceUnit.CM));
                        }
                        Drive_Train.brake(fr, fl, br, bl);

                    }
                    v_state++;
                case 8://  case of moving to next white line
                    encoderDrive(6.0,"backward",0.9);

                    double distancemoved = ranges.getDistance(DistanceUnit.CM) - 30.0;

                    double angleofrobot = (Math.atan(distancemoved/30.54))*180/Math.PI;
                    Drive_Train.turn_left(fr,fl,br,bl);

                    runtime.reset();
                    while(runtime.seconds() < angleofrobot/100)
                    {
                        telemetry.addData("seconds", runtime.seconds());
                        telemetry.update();
                    }

                    Drive_Train.brake(fr, fl, br,bl);
                    Drive_Train.setPowerD(0.6);
                    Drive_Train.run_left(fr, fl, br, bl);
                    while (opModeIsActive() && ranges.getDistance(DistanceUnit.CM) > 30) {

                        telemetry.addData("Distance ", ranges.getDistance(DistanceUnit.CM));
                    }
                    Drive_Train.brake(fr, fl, br, bl);

                    Drive_Train.setPowerD(0.6);
                    Drive_Train.run_backward(fr, fl, br, bl);

                    while (opModeIsActive() && ods.getLightDetected() < initialC + .1) {
                        telemetry.addData("Light ", ods.getLightDetected());
                        telemetry.update();
                    }
                    Drive_Train.brake(fr, fl, br, bl);

                    v_state++;

                case 9://  case of pressing beacon button
                    if (beacon.getAnalysis().isLeftRed()) {
                        encoderDrive(12.0, "right", 0.5);
                        PauseAuto(0.2);

                        Drive_Train.setPowerD(0.6);
                        Drive_Train.run_left(fr, fl, br, bl);
                        while (opModeIsActive() && ranges.getDistance(DistanceUnit.CM) > 30) {

                            telemetry.addData("Distance ", ranges.getDistance(DistanceUnit.CM));
                        }
                        Drive_Train.brake(fr, fl, br, bl);
                    } else if (beacon.getAnalysis().isRightBlue()) {
                        //beacon was correctly pressed
                    } else {
                        //go forward if the left side of the beacon is blue

                        //beacon is 1/2 a foot, presser is on the right side so it is lined up with the line
                        encoderDrive(6.0, "forward", .5);

                        encoderDrive(12.0, "right", 0.5);
                        PauseAuto(0.2);

                        Drive_Train.setPowerD(0.6);
                        Drive_Train.run_left(fr, fl, br, bl);
                        while (opModeIsActive() && ranges.getDistance(DistanceUnit.CM) > 30) {

                            telemetry.addData("Distance ", ranges.getDistance(DistanceUnit.CM));
                        }
                        Drive_Train.brake(fr, fl, br, bl);

                    }
                    v_state++;

                case 10://  case of moving to corner vortex

                case 11://  case of moving to center vortex
            }
        }
        //Run to line
/*
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

        while ( opModeIsActive() && ranges.getDistance(DistanceUnit.CM) > 19) {
           telemetry.addData("Distance ", ranges.getDistance(DistanceUnit.CM));
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
            encoderDrive(6.0,"backward" , .5);
        } else {
           // encoderDrive(720,"backward" , .15);
        }
        //beacon code

        //
        // Wait...
        //

        //hit the button
        encoderDrive(200,"right",.3);
        encoderDrive(6.0,"right",.3);

        */
    }


    // methods
    public void PauseAuto(double time /*Seconds*/) {
        //
        // for Waiting between driving periods.
        //
        runtime.reset();
        while(runtime.seconds() < time)
        {
            // do nothing
            telemetry.addData("Seconds", runtime.seconds());
        }

    }
    public void encoderDrive(double inches /*Inches*/, String direction /*Direction*/, double power /*Power between 0.0 and 1.0*/) {
        //
        // Sets the encoders
        //

        encoderval = ticks_per_inch.intValue() * ((int) inches);
        Drive_Train.run_using_encoders(fr, fl, br, bl);
        //
        // Uses the encoders and motors to set the specific position
        //
        Drive_Train.setPosition(encoderval,encoderval,encoderval,encoderval,fr,fl,br,bl);
        //
        // Sets the power and direction
        //
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
        //
        // while in the -TEST DISTANCE- loop below, it will keep running until the distance
        // from the encoders is achieved. When achieved, the program will proceed to the end
        // of the function.
        //
        while(Drive_Train.testDistance(fl) != 1){
            telemetry.addData("Pos " , fl.getCurrentPosition());
            telemetry.update();
        }
        //
        // Ends the Drive period.
        //
        Drive_Train.brake(fr, fl, br, bl);

    }


    public void Wait(int delayInMilliseconds){
        long myStartTime  = System.currentTimeMillis();
        long myCurrentTime  = System.currentTimeMillis();

        do {
            myCurrentTime = System.currentTimeMillis();
            telemetry.addData("Waiting",(myCurrentTime-myStartTime));
        }while((myCurrentTime-myStartTime) < delayInMilliseconds);
    }

}
