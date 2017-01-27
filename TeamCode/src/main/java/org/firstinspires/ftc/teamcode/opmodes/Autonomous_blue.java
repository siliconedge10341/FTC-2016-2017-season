package org.firstinspires.ftc.teamcode.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.lasarobotics.vision.android.Cameras;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;


import org.firstinspires.ftc.teamcode.classes.Mecanum;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;



@Autonomous(name = "Blue_Autonomous_dude", group = "Blue")

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
       private Servo ballServo;
        //private Servo beaconServo;

        // Range Sensor
        //private ModernRoboticsI2cRangeSensor rangef;
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
    private double avgr = 0;


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
            ballServo = hardwareMap.servo.get("servo_ball");
        ballServo.setPosition(Servo.MAX_POSITION);
           // beaconServo = hardwareMap.servo.get("servo_beacon");

            // Classes
            ods = hardwareMap.opticalDistanceSensor.get("ods_line");
            rangesf = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_side_front");
            rangesf.setI2cAddress(I2cAddr.create8bit(0x20));
            rangesb = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_side_back");
        rangesb.setI2cAddress(I2cAddr.create8bit(0x28));



        waitForVisionStart();

        //VISION:
        this.setCamera(Cameras.SECONDARY);
        this.setFrameSize(new Size(900, 900));

        enableExtension(LinearVisionOpMode.Extensions.BEACON);         //Beacon detection
        enableExtension(LinearVisionOpMode.Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(LinearVisionOpMode.Extensions.CAMERA_CONTROL); //Manual camera control
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);

        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);

        rotation.setIsUsingSecondaryCamera(true);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.LANDSCAPE_REVERSE);

        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();

        waitForStart();
        //Start OpMode

        initialC = ods.getLightDetected();

        //motorCollector.setPower(0.9);



        shootBall();
        //
        //Run to line
        //


        PauseAuto(1.5);

        //Turn to wall
        runtToLine();

        PauseAuto(0.5);

        alignWall();
        //
        // Detect beacon
        //

        PauseAuto(1.0);

        while (opModeIsActive() && !beacon.getAnalysis().isBeaconFound()) {
            telemetry.addData("Beacon", beacon.getAnalysis());
            telemetry.update();
        }
        if (beacon.getAnalysis().isLeftBlue() == true) {

            encoderDrive(3.0,"forward" , .3);



        } else {
            encoderDrive(3.0,"backward" , .3);

        }
        //
        // beacon code
        //
        telemetry.addData("Beacon " , beacon.getAnalysis().getColorString());
        telemetry.update();
        PauseAuto(1.0);
        //
        // hit the button
        //

        avgr = avgRangeF();
        PauseAuto(1.0);
        encoderDrive(avgr * 2 +3,"rightalign",.8);

        telemetry.addData("Hit button " , " ");
        telemetry.update();

        PauseAuto(1.0);
        encoderDrive(10.0 * 2, "left" , .7);
        //
        // Run to line
        //
        PauseAuto(.5);
        telemetry.addData("Next " , " Line");
        telemetry.update();

        PauseAuto(1.0);


        encoderDrive(24.0 , "forward" , .3);

        PauseAuto(.5);
        //alignWall();
        PauseAuto(.5);
        //
        //Run to line
        //
        runtToLine();
        //
        // Stop at wall
        //
        PauseAuto(.4);

        //
        // Detect beacon
        //
        while (opModeIsActive() && !beacon.getAnalysis().isBeaconFound()) {
            telemetry.addData("Beacon", beacon.getAnalysis());
            telemetry.update();
        }
        if (beacon.getAnalysis().isLeftBlue() == true) {

            encoderDrive(buttonWidth - 3,"forward" , .3);

        } else {

            encoderDrive(buttonWidth + 3,"backward" , .3);

        }

        PauseAuto(1.0);
        //
        // beacon code
        //
        avgr = avgRangeF();
        encoderDrive(avgr * 2,"rightalign",.3);

        PauseAuto(.4);
        encoderDrive(10.0 * 2 , "left" , .3);


    }

    public void PauseAuto(double time)
    {
        runtime.reset();
        while(runtime.seconds() < time)
        {
            //do nothing
        }
    }
    public void encoderDrive(double inches , String direction, double power ) {
        int encoderval;
        // Sets the encoders
        //
        Drive_Train.reset_encoders(fr,fl,br,bl);
        encoderval = ticks_per_inch.intValue() * (int)inches;
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
        if(direction == "leftalign")
        {
            while(Drive_Train.testDistance(fl)!= 1)
            {
                Drive_Train.run_left_using_alignment(fr,fl,br,bl,rangesb.getDistance(DistanceUnit.CM),rangesf.getDistance(DistanceUnit.CM));
                telemetry.addData("Pos ", fl.getCurrentPosition());
                telemetry.update();
            }
        }else if(direction == "rightalign"){
            while(Drive_Train.testDistance(fl)!= 1)
            {
                Drive_Train.run_right_using_alignment(fr,fl,br,bl,rangesb.getDistance(DistanceUnit.CM),rangesf.getDistance(DistanceUnit.CM));
                telemetry.addData("Pos ", fl.getCurrentPosition());
                telemetry.update();
            }
        }
        else {
            while (Drive_Train.testDistance(fl) != 1) {
                telemetry.addData("Pos ", fl.getCurrentPosition());
                telemetry.update();
            }
        }

        //
        // Ends the Drive period.
        //
        Drive_Train.brake(fr, fl, br, bl);


    }
    public double avgRangeF(){
        double avg = 0;
        int i = 0;
        int rcount = 0;
        for (i =0; i<=5 ;i++){
            if (rangesf.getDistance(DistanceUnit.INCH) < 1 || rangesf.getDistance(DistanceUnit.INCH) == 255){

            }else{
                avg = avg + rangesf.getDistance(DistanceUnit.INCH);
                rcount ++;
            }


        }

        return (avg / rcount);
    }
    public double avgRangeB(){
        double avg = 0;
        int i = 0;
        int rcount = 0;
        while (i<10 && opModeIsActive()){
            if (rangesb.getDistance(DistanceUnit.INCH) <= 1 || rangesb.getDistance(DistanceUnit.INCH) == 255){

            }else{
                avg = avg + rangesb.getDistance(DistanceUnit.INCH);
                rcount ++;
            }
            i++;
        }

        return (avg / rcount);
    }

public void alignWall() throws InterruptedException{
    Drive_Train.run_without_encoders(fr,fl,br,bl);

    while (Math.abs(rangesb.getDistance(DistanceUnit.CM) - rangesf.getDistance(DistanceUnit.CM)) > 1 && opModeIsActive()) {
        double cmback = rangesb.getDistance(DistanceUnit.CM);
        double cmfront = rangesf.getDistance(DistanceUnit.CM);
        if (cmback == 255 || cmfront == 255 || cmfront == 0 || cmback == 0){
            Drive_Train.brake(fr,fl,br,bl);

        }
        else if (cmfront < cmback) {
            fl.setPower(-0.7);
            fr.setPower(-0.7);
            bl.setPower(-0.7);
            br.setPower(-0.7);

        } else if (cmback < cmfront) {
            fl.setPower(0.7);
            fr.setPower(0.7);
            bl.setPower(0.7);
            br.setPower(0.7);
        }

    }
        telemetry.addData("back" , rangesb.getDistance(DistanceUnit.CM));
        telemetry.addData("front" , rangesf.getDistance(DistanceUnit.CM));
        telemetry.update();
        Drive_Train.brake(fr,fl,br,bl);
}

    public void shootBall(){

        motorShootL.setPower(0.7);
        motorShootR.setPower(-0.5);
        runtime.reset();
        runtime.startTime();
        while (runtime.seconds() < 1.0) {
            //
            // Lets the spinners speed up
            //
        }
        ballServo.setPosition(0.85);
        runtime.reset();
        runtime.startTime();
        while (runtime.seconds() < 1.0) {
            //
            // Shoots ball
            //
        }
        motorShootL.setPower(0.0);
        motorShootR.setPower(0.0);
        ballServo.setPosition(ballServo.MAX_POSITION);
        runtime.reset();
        runtime.startTime();
        while (runtime.seconds() < 0.75) {
            //
            // Loads another ball
            //
        }
        motorShootL.setPower(0.7);
        motorShootR.setPower(-0.5);
        runtime.reset();
        runtime.startTime();
        while (runtime.seconds() < 1.0) {
            //
            // Lets the spinners speed up
            //
        }
        ballServo.setPosition(0.85);
        runtime.reset();
        runtime.startTime();
        while (runtime.seconds() < 1.0) {
            //
            // Shoots ball
            //
        }
        ballServo.setPosition(ballServo.MAX_POSITION);
        motorShootL.setPower(0);
        motorShootR.setPower(0);
    }
    public void runtToLine(){
        Drive_Train.run_without_encoders(fr,fl,br,bl);
        Drive_Train.setPowerD(0.35);

        Drive_Train.run_forward(fr, fl, br, bl);
        runtime.reset();
        while (opModeIsActive() && ods.getLightDetected()< initialC +.1 && runtime.seconds()<5) {
            // Get Data
            telemetry.addData("base power" , Drive_Train.getSpeed());
            telemetry.addData("Light ",ods.getLightDetected());
            telemetry.update();

        }
        //Drive_Train.reset_encoders(fr, fl, br, bl);
        Drive_Train.brake(fr, fl, br, bl);
        telemetry.addData("Line" ,  " Found!");
        telemetry.update();
    }
    public  void encoderTurn(double inches ){
        Drive_Train.run_using_encoders(fr,fl,br,bl);
        int encoderval;
        // Sets the encoders
        //
        Drive_Train.reset_encoders(fr,fl,br,bl);
        encoderval = ticks_per_inch.intValue() * (int)inches;
        Drive_Train.run_using_encoders(fr, fl, br, bl);
        //
        // Uses the encoders and motors to set the specific position
        //
        Drive_Train.setPosition(encoderval,encoderval,encoderval,encoderval,fr,fl,br,bl);
        Drive_Train.setPowerD(.5);
        Drive_Train.turn_left(fr,fl,br,bl);

        while (Drive_Train.testDistance(fl) != 1) {
            telemetry.addData("Pos ", fl.getCurrentPosition());
            telemetry.update();
        }
        Drive_Train.brake(fr,fl,br,bl);

    }
}
