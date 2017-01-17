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

        private DcMotor fr;
        private DcMotor fl;
        private DcMotor bl;
        private DcMotor br;
        private DcMotor motorShootL;
        private DcMotor motorShootR;
       // private DcMotor motorCollector;
        private DcMotor motorConveyor;

        // servo
       // private Servo releaseServo;
        //private Servo beaconServo;

        // Range Sensor
        private ModernRoboticsI2cRangeSensor rangef;
        private ModernRoboticsI2cRangeSensor rangesf;
        private ModernRoboticsI2cRangeSensor rangesb;
        private OpticalDistanceSensor ods;

        // Sensor Classes
        private Mecanum Drive_Train = new Mecanum();
        private ElapsedTime runtime = new ElapsedTime();

    // Reading for the initial color we take at the beginning of the match.
    // This helps us because when we test for the white line, we want to be
    // able to tell the difference from the color of the ground. Thus
    // knowing where the sensor is.
    private double initialC = 0;

    // states variable for the loop
    private int v_state = 0;
    private static final Double ticks_per_inch = 510 / (3.1415 * 4);

    private static final double distancetoBase = 46.7;
    private static final double distanceFromWall = 19;
    private static final double buttonWidth = 6.0;


    public void runOpMode() throws InterruptedException {
        // Sets every class at the beginning of the autonomous run class
        // Hardware Maps
            // Motors
            fr = hardwareMap.dcMotor.get("fr_motor");
            fl = hardwareMap.dcMotor.get("fl_motor");
            br = hardwareMap.dcMotor.get("br_motor");
            bl = hardwareMap.dcMotor.get("bl_motor");

            motorShootL = hardwareMap.dcMotor.get("shooter_left");
            motorShootR = hardwareMap.dcMotor.get("shooter_right");
            //motorCollector = hardwareMap.dcMotor.get("ball_collector");
            motorConveyor = hardwareMap.dcMotor.get("conveyor_motor");

            // Servos
            //releaseServo = hardwareMap.servo.get("servo_ball");
           // beaconServo = hardwareMap.servo.get("servo_beacon");

            // Classes
            ods = hardwareMap.opticalDistanceSensor.get("ods_line");
            rangef = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_front");
            rangesf = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_side_left");
            rangesb = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_side_right");

            // Sets Position
            //releaseServo.setPosition(0.3);
            //beaconServo.setPosition(.5);

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

        //motorCollector.setPower(0.9);



        PauseAuto(0.4);
        //
        // Turn
        //

        PauseAuto(0.4);


        Drive_Train.setPowerD(.15);
        Drive_Train.run_diagonal_right_up(fr, fl, br, bl);
        while (opModeIsActive() && ods.getLightDetected()< initialC +.1) {
            // Get Data
            telemetry.addData("Light ",ods.getLightDetected());
            telemetry.update();
        }
        //Drive_Train.reset_encoders(fr, fl, br, bl);
        Drive_Train.brake(fr, fl, br, bl);

        PauseAuto(.4);

        //beaconServo.setPosition(0.0);
        //Go closer to wall
        Drive_Train.setPowerD(.2);
        Drive_Train.run_right_using_alignment(fr,fl,br,bl,rangesb.getDistance(DistanceUnit.CM),rangesf.getDistance(DistanceUnit.CM));
        while(rangesf.getDistance(DistanceUnit.INCH) > 8){
            // Get data
            telemetry.addData("Distance ", rangesf.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
        Drive_Train.brake(fr,fl,br,bl);
        //
        //Shoot ball
        //
        motorConveyor.setPower(.7);
        motorShootL.setPower(1.0);
        motorShootR.setPower(-1.0);
        runtime.reset();
        while (runtime.seconds() < 4.0){
            telemetry.addData("seconds",runtime.seconds());
            telemetry.update();
        }
        motorConveyor.setPower(0.0);
        motorShootL.setPower(0.0);
        motorShootR.setPower(0.0);

        //
        // Detect beacon
        //
        while(opModeIsActive()&& !beacon.getAnalysis().isBeaconFound()) {
            telemetry.addData("Beacon ", beacon.getAnalysis());
            telemetry.update();
        }
        if (beacon.getAnalysis().isLeftBlue() == true) {
            //
            // go forward if the left side of the beacon is blue.
            // beacon is 1/2 a foot, presser is on the right side so it is lined up with the line
            //
            encoderDrive(buttonWidth,"forward" , .3);

        } else {

            encoderDrive(buttonWidth - 2,"backward" , .3);

        }
        //
        // beacon code
        //
        PauseAuto(1.0);
        //
        // hit the button
        //
        encoderDrive(8.0*2,"right" , .15);

        PauseAuto(1.0);
        //
        // go back a little bit
        //
        encoderDrive(10.0 * 2, "left" , .15);
        //
        // Run to line
        //
        Drive_Train.setPowerD(.2);
        Drive_Train.run_forward(fr, fl, br, bl);
        //Drive_Train.setPosition(4 * 1440,4*1440,4*1440,4*1440, fr, fl, br, bl);
        while (opModeIsActive() && ods.getLightDetected() > initialC + .1) {
            // Get data
            telemetry.addData("Light" , ods.getLightDetected());
            telemetry.update();
        }
        Drive_Train.reset_encoders(fr, fl, br, bl);
        Drive_Train.brake(fr, fl, br, bl);
        //
        // Stop at wall
        //
        PauseAuto(.4);
        //Move closer to the wall

        Drive_Train.run_using_encoders(fr, fl, br, bl);
        Drive_Train.run_right(fr, fl, br, bl);
        while ( opModeIsActive() && rangesf.getDistance(DistanceUnit.INCH) > 8) {
            // Get data
            telemetry.addData("Distance ", rangesf.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
        Drive_Train.reset_encoders(fr, fl, br, bl);
        Drive_Train.brake(fr, fl, br, bl);
        //
        // Wait...
        //
        // Detect beacon
        //
        while (opModeIsActive() && !beacon.getAnalysis().isBeaconFound()) {
            telemetry.addData("Beacon", beacon.getAnalysis());
            telemetry.update();
        }
        if (beacon.getAnalysis().isLeftBlue() == true) {

            encoderDrive(buttonWidth,"forward" , .3);

        } else {

            encoderDrive(buttonWidth-2,"backward" , .3);

        }

        PauseAuto(1.0);
        //
        // beacon code
        //

        encoderDrive(8.0 *2,"right",.3);

        PauseAuto(.4);
        encoderDrive(3.0 , "left" , .3);

        //beaconServo.setPosition(.5);

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
