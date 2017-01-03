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

@Autonomous(name = "Blue_Autonomous", group = "Blue")

public class Autonomous_blue extends LinearVisionOpMode {
    // instance variables
    // private variables
    // Motors
    DcMotor fr;
    DcMotor fl;
    DcMotor bl;
    DcMotor br;
    DcMotor motorShootL;
    DcMotor motorShootR;

    DcMotor motorCollector;

    Servo releaseServo;
    Servo beaconServo;


    // Range Sensor
    ModernRoboticsI2cRangeSensor rangef;
    ModernRoboticsI2cRangeSensor ranges;
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
    private static final Double ticks_per_inch = 510 / (3.1415 * 4);

    boolean startatcenter = true;
    boolean firetwice = true;
    boolean knockcapball = true;
    boolean pressbeacons = true;
    boolean endincorner = true;
    boolean endincenter = false;


    public void runOpMode() throws InterruptedException {
        // Sets every class at the beginning of the demoautonomous run class
        //Hardware Maps
        fr = hardwareMap.dcMotor.get("fr_motor");
        fl = hardwareMap.dcMotor.get("fl_motor");
        br = hardwareMap.dcMotor.get("br_motor");
        bl = hardwareMap.dcMotor.get("bl_motor");

        motorShootL = hardwareMap.dcMotor.get("shooter_left");
        motorShootR = hardwareMap.dcMotor.get("shooter_right");
        motorCollector = hardwareMap.dcMotor.get("ball_collector");

        releaseServo = hardwareMap.servo.get("servo_ball");
        beaconServo = hardwareMap.servo.get("servo_beacon");

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

        encoderDrive(510,"forward",1.0);


        /////////////////////////////////////////////////////////
        //Shoot ball:
        releaseServo.setPosition(.05);
        motorCollector.setPower(0.9);
        motorShootL.setPower(1.0);
        motorShootR.setPower(-1.0);
        runtime.reset();
        while (runtime.seconds() < 4.0){
            telemetry.addData("seconds",runtime.seconds());
            telemetry.update();
        }
        motorCollector.setPower(0);
        releaseServo.setPosition(.3);
        motorShootL.setPower(0.0);
        motorShootR.setPower(0.0);


        //Hit the ball:
        encoderDrive(24.0 , "left" , .5);
        PauseAuto(.5);
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
        Drive_Train.brake(fr, fl, br, bl);
        PauseAuto(.5);
        //Move to wall
        Drive_Train.setPowerD(.5);
        Drive_Train.run_right(fr, fl, br, bl);
        while (opModeIsActive() && ranges.getDistance(DistanceUnit.CM) > 19) {

            telemetry.addData("Distance ",ranges.getDistance(DistanceUnit.CM ));
        }
        Drive_Train.brake(fr,fl,br,bl);
        Drive_Train.reset_encoders(fr,fl,br,bl);

        beaconServo.setPosition(1.0);

        //Move and detect line

        Drive_Train.run_using_encoders(fr,fl,br,bl);
        Drive_Train.setPowerD(.15);

        Drive_Train.run_backward(fr, fl, br, bl);

        while (opModeIsActive() && ods.getLightDetected()< initialC +.1) {
            telemetry.addData("Light ",ods.getLightDetected());
            telemetry.update();
        }
        //Drive_Train.reset_encoders(fr, fl, br, bl);
        Drive_Train.brake(fr, fl, br, bl);

        beaconServo.setPosition(0.0);
        // Detect beacon
        if (beacon.getAnalysis().isLeftBlue() == true) {
            //go forward if the left side of the beacon is blue

            //beacon is 1/2 a foot, presser is on the right side so it is lined up with the line
            encoderDrive(6.0,"forward" , .5);

        } else {
            //beacon is 1/2 a foot

        }
        // beacon code



        //hit the button

        encoderDrive(1.0,"right" , .15);

        //go back a little bit
        encoderDrive(100, "left" , .15);

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
            encoderDrive(250,"backward" , .5);
        } else {
            // encoderDrive(720,"backward" , .15);
        }
        //beacon code

        //
        // Wait...
        //

        //hit the button
        encoderDrive(200,"right",.3);


    }

    public void PauseAuto(double time)
    {
        runtime.reset();
        while(runtime.seconds() < time)
        {
            //do nothing
        }
    }
    public void encoderDrive(double inches /*Inches*/, String direction /*Direction*/, double power /*Power between 0.0 and 1.0*/) {
        int encoderval;
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



}
