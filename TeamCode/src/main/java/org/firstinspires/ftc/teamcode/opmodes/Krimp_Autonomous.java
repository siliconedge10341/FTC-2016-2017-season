// Package for -THIS- project
package org.firstinspires.ftc.teamcode.opmodes;

// Imports for variables and phone configuration
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.classes.Light;
import org.firstinspires.ftc.teamcode.classes.Mecanum;
import org.firstinspires.ftc.teamcode.classes.Range;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.VisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

/**
 * Created by Evan Krimpenfort on 12/31/2016.
 *
 * This goes out to my teammates and fellow friends of Silicon Edge.
 * Let's hope this one works along with our rest.
 *
 * "Shut Up Anirudh" - Juan, 2016.
 */

@Autonomous(name = "Final_Countdown", group = "Blue")
public class Krimp_Autonomous extends VisionOpMode {
    // instance variables
    // private data
    // Motors
        private DcMotor mtrFR;
        private DcMotor mtrFL;
        private DcMotor mtrBR;
        private DcMotor mtrBL;
        private DcMotor mtrShootR;
        private DcMotor mtrShootL;
        private DcMotor mtrCollect;

    // Servos
        private Servo srvRelease;
        private Servo srvBeacon;

    // Classes
        private Mecanum drive_train = new Mecanum();
        private Range range_sensor_beacon = new Range();
        private Range range_sensor_proj = new Range();
        private Light color_sensor = new Light();
        private ElapsedTime runtime = new ElapsedTime();
        private ElapsedTime total_time = new ElapsedTime();

    // Variables
        // Reading for the initial color we zz take at the beginning of the match.
        // This helps us because when we test for the white line, we want to be
        // able to tell the difference from the color of the ground. Thus
        // knowing where the sensor is.
        private double initialC = 0;
        private double initialD = 0;

        // states variable for the loop
        private static final Double ticks_per_inch = 510 / (3.1415 * 4);

    // public data

    // Constructors
    public Krimp_Autonomous() {
        // Default Constructor

    }

    // Initialization
    public void init() {
        //
        // Initializes every motor, servo, variable, and position.
        //
        initialC = 0;
        initialD = 0;

        // Motors
        mtrFR = hardwareMap.dcMotor.get("fr_motor");
        mtrFL = hardwareMap.dcMotor.get("fl_motor");
        mtrBR = hardwareMap.dcMotor.get("br_motor");
        mtrBL = hardwareMap.dcMotor.get("bl_motor");
        mtrShootR = hardwareMap.dcMotor.get("shooter_right");
        mtrShootL = hardwareMap.dcMotor.get("shooter_left");
        mtrCollect = hardwareMap.dcMotor.get("ball_collector");

        mtrShootL.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        mtrShootR.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

        // Servos
        srvRelease = hardwareMap.servo.get("servo_ball");
        srvRelease = hardwareMap.servo.get("servo_beacon");

        // Classes
        color_sensor.setV_sensor(hardwareMap.opticalDistanceSensor.get("ods_line"));
        range_sensor_beacon.setRange(hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_side"));
        range_sensor_proj.setRange(hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_front"));

        // Positions
        srvRelease.setPosition(0.25);
        srvBeacon.setPosition(0.5);

        super.init();
        this.setCamera(Cameras.PRIMARY);
        this.setFrameSize(new Size(900, 900));

        enableExtension(LinearVisionOpMode.Extensions.BEACON);         // Beacon detection
        enableExtension(LinearVisionOpMode.Extensions.ROTATION);       // Automatic screen rotation correction
        enableExtension(LinearVisionOpMode.Extensions.CAMERA_CONTROL); // Manual camera control
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);

        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);

        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);

        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();

    }

    // Start
    public void start() {
        //
        // Starts the -LOOP-.
        //
        super.start();

        // Resets encoders to begin -LOOP-.
        drive_train.reset_encoders(mtrFR, mtrFL, mtrBR, mtrBL);

    }

    // Loop
    public void loop() {
        //
        // Loop that controls the -AUTONOMOUS- period.
        //
        super.loop();
        //
        // SHOOTING STATE:
        // v_state == 0 is all about a few seconds where the robot moves the launching servo
        // to release to balls. Then, after 6 seconds. the robot will move into the next state.
        //
        total_time.reset();
        total_time.startTime();
        //
        // Time starts
        //
        initialC = color_sensor.getLightDetected();
        telemetry.addData("Light WaveLength", initialC);
        //
        // Shoots ball for 3 seconds
        // One ball by the releases servo and there is one ball in the collector so there
        // is a delay between the shots. This allows for smoother transitions.
        //
        srvRelease.setPosition(srvRelease.MAX_POSITION);
        mtrShootL.setPower(1.0);
        mtrShootR.setPower(-1.0);
        mtrCollect.setPower(1.0);
        runtime.reset();
        runtime.startTime();
        while (runtime.seconds() < 4) {
            telemetry.addData("Seconds", runtime.seconds());
        }
        mtrShootL.setPower(0);
        mtrShootR.setPower(0);
        mtrCollect.setPower(0);
        srvRelease.setPosition(0.25);

        //
        // Wait...
        //
        // TURNING STATE
        // v_state == 1 is all about turning 90 degrees to the left to make sure that the
        // touching servo, the range sensor, and the beacon are all facing the wall.
        //
        PauseAuto(0.2);
        //
        // Rotation
        //
        mtrFR.setPower(1.0);
        mtrFL.setPower(1.0);
        mtrBR.setPower(1.0);
        mtrBL.setPower(1.0);
        runtime.reset();
        runtime.startTime();
        while (runtime.seconds() < 0.86) {
            // Get data
            telemetry.addData("seconds", runtime.seconds());
            telemetry.update();
        }
        drive_train.brake(mtrFR, mtrFL, mtrBR, mtrBL);

        PauseAuto(0.4);

        //
        // Wait...
        //
        // DIAGONAL RIGHT UP
        // v_state == 2 is all about moving the robot to it's specific position at the white
        // line up by the coloured box. From there we go into the next state. Repositioning.
        //
        drive_train.setPowerD(0.6);
        drive_train.run_diagonal_right_up(mtrFR, mtrFL, mtrBR, mtrBL);
        runtime.reset();
        runtime.startTime();
        while (runtime.seconds() < 8 || color_sensor.getLightDetected() < initialC + .1) {
            telemetry.addData("Colour WaveLength", color_sensor.getLightDetected());
        }
        drive_train.brake(mtrFR, mtrFL, mtrBR, mtrBL);

        PauseAuto(0.4);

        //
        // Wait...
        //
        // SET UP POSITION
        // v_state == 3 is depending on the colour, we will move the robot with left or right,
        // depending on which side of the coloured box is -BLUE-.
        //
        // if right -BLUE- then we move BACKWARD.
        // if left -BLUE- then we move FORWARD.
        //
        // After repositioning, we will proceed with the next state, pushing.
        //
        telemetry.addData("Beacon" , beacon.getAnalysis().toString());
        telemetry.update();

        if (beacon.getAnalysis().isLeftRed()) {
            encoderDrive(12.0, "right", 0.6);

            PauseAuto(0.2);

            drive_train.setPowerD(0.6);
            drive_train.run_left(mtrFR, mtrFL, mtrBR, mtrBL);
            while (range_sensor_beacon.getData() > 30) {
                // Get data
                telemetry.addData("Distance ", range_sensor_beacon.getData());
            }
            drive_train.brake(mtrBL, mtrBL, mtrBL, mtrBL);

        } else if (beacon.getAnalysis().isRightBlue()) {
            //
            // Beacon was correctly pressed
            //
        } else if (beacon.getAnalysis().isLeftBlue()){
            //
            // Go forward if the left side of the beacon is blue.
            // Beacon is 1/2 a foot, presser is on the right side so it is lined up with the line
            //
            // Wait...
            //

            encoderDrive(6.0, "forward", 0.6);

            PauseAuto(0.2);
            //
            // STRAFE RIGHT
            // v_state == 4 is when the robot goes in for the points and presses the -FIRST BEACON-
            // After pressing the button, it will go into the next state, going back to original position.
            //

            encoderDrive(12.0, "right", 0.6);

            PauseAuto(0.2);
            //
            // Wait...
            //
            // STRAFE LEFT
            // v_state 5 is about returning to where you were before pressing the button.
            // After this, the next state will be moving to the next beacon.
            //

            encoderDrive(10.0, "left" , 0.6);

        }

        PauseAuto(0.4);

        //
        // Wait...
        //
        // RUN FORWARD
        // v_state 6 is all about going to the next colour pad and pressing the second
        // button. We will use the color sensor to read the second line. Then we will
        // repeat the same steps to press the button again.
        //
        drive_train.setPowerD(0.6);
        drive_train.run_forward(mtrFR, mtrFL, mtrBR, mtrBL);
        runtime.reset();
        runtime.startTime();
        while (runtime.seconds() < 6 || color_sensor.getLightDetected() < initialC + .1) {
            telemetry.addData("Colour", color_sensor.getLightDetected());
            telemetry.addData("Seconds", runtime.seconds());
        }
        drive_train.brake(mtrFR, mtrFL, mtrBR, mtrBL);

        PauseAuto(0.4);

        //
        // Wait...
        //
        // SET UP POSITION
        // v_state == 7 is depending on the colour, we will move the robot with left or right,
        // depending on which side of the coloured box is -BLUE-.
        //
        // if right -BLUE- then we move BACKWARD.
        // if left -BLUE- then we move FORWARD.
        //
        // After repositioning, we will proceed with the next state, pushing.
        //
        telemetry.addData("Beacon" , beacon.getAnalysis().toString());
        telemetry.update();

        if (beacon.getAnalysis().isLeftRed()) {
            encoderDrive(12.0, "right", 0.6);

            PauseAuto(0.2);

            drive_train.setPowerD(0.6);
            drive_train.run_left(mtrFR, mtrFL, mtrBR, mtrBL);
            while (range_sensor_beacon.getData() > 30) {
                // Get data
                telemetry.addData("Distance ", range_sensor_beacon.getData());
            }
            drive_train.brake(mtrBL, mtrBL, mtrBL, mtrBL);

        } else if (beacon.getAnalysis().isRightBlue()) {
            //
            // Beacon was correctly pressed
            //
        } else if (beacon.getAnalysis().isLeftBlue()){
            //
            // Go forward if the left side of the beacon is blue.
            // Beacon is 1/2 a foot, presser is on the right side so it is lined up with the line
            //
            // Wait...
            //

            encoderDrive(6.0, "forward", 0.6);

            PauseAuto(0.2);
            //
            // STRAFE RIGHT
            // v_state == 8 is when the robot goes in for the points and presses the -FIRST BEACON-
            // After pressing the button, it will go into the next state, going back to original position.
            //

            encoderDrive(12.0, "right", 0.6);

            PauseAuto(0.2);
            //
            // Wait...
            //
            // STRAFE LEFT
            // v_state 9 is about returning to where you were before pressing the button.
            // After this, the next state will be moving to the next beacon.
            //

            encoderDrive(10.0, "left" , 0.6);

        }

        PauseAuto(0.4);

        //
        // Wait...
        //
        if (total_time.seconds() < 26) {
            //
            // Wait...
            //
            // DOWN
            // v_state 10 is all about moving to the corner vortex to get on the
            // ramp and get extra points for the team. We have to do this if time
            // is less than 26 seconds however, all in order to get extra points.
            //
            encoderDrive(4.0, "left", 0.5);

            PauseAuto(0.2);
            //
            // Moving a little more towards the ramp edge so we get the points.
            //
            encoderDrive(96.0, "backward", 0.5);

            PauseAuto(0.2);
            //
            // Completed the autonomous period
            //

        } else if (total_time.seconds() >= 26) {
            //
            // Wait...
            //
            // DIAGONAL DOWN LEFT
            // v_state 10 is about hitting the cat ball and staying on the middle platform
            // to gain the final points for the autonomous period. After this and the position
            // it stays at. The v_state will increase into nothing. Therefore, it will stop and
            // end. This is for 90 points in the Velocity vortex.
            //
            encoderDrive(9.0, "left", 0.5);

            PauseAuto(0.2);
            //
            // Moving a little more towards the center line so we get the points.
            //
            encoderDrive(48.0, "diagonal_left_down", 0.5);

            PauseAuto(0.2);
            //
            // Completed the autonomous period
            //
        }

        //
        // Program Ends...
        //
        // Wait...
        //
        // Stops the program.
        //
        stop();

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
    public void encoderDrive(double inches /*Seconds*/, String direction /*movement type*/, double power /*Power from 1.0 to -1.0*/) {
        int encoderval;

        encoderval = ticks_per_inch.intValue() * ((int) inches);
        drive_train.run_using_encoders(mtrFR, mtrFL, mtrBR, mtrBL);
        //
        // Uses the encoders and motors to set the specific position
        //
        drive_train.setPosition(encoderval, encoderval, encoderval, encoderval, mtrFR, mtrFL, mtrBR, mtrBL);
        //
        // Sets the power and direction
        //
        drive_train.setPowerD(power);
        if (direction == "forward"){
            drive_train.run_forward(mtrFR, mtrFL, mtrBR, mtrBL);
        } else if(direction == "backward"){
            drive_train.run_backward(mtrFR, mtrFL, mtrBR, mtrBL);
        } else if (direction == "left"){
            drive_train.run_left(mtrFR, mtrFL, mtrBR, mtrBL);
        } else if (direction == "right"){
            drive_train.run_right(mtrFR, mtrFL, mtrBR, mtrBL);
        } else if (direction == "diagonal_left_up"){
            drive_train.run_diagonal_left_up(mtrFR, mtrFL, mtrBR, mtrBL);
        } else if (direction == "diagonal_left_down"){
            drive_train.run_diagonal_left_down(mtrFR, mtrFL, mtrBR, mtrBL);
        }
        //
        // while in the -TEST DISTANCE- loop below, it will keep running until the distance
        // from the encoders is achieved. When achieved, the program will proceed to the end
        // of the function.
        //
        while(drive_train.testDistance(mtrFR) != 1){
            telemetry.addData("Pos " , mtrFR.getCurrentPosition());
            telemetry.update();
        }
        //
        // Ends the Drive period.
        //
        drive_train.brake(mtrFR, mtrFL, mtrBR, mtrBL);
    }
    public void stop() {
        //
        // Stops -LOOP- and ends Program.
        //
       drive_train.brake(mtrFR, mtrFL, mtrBR, mtrBL);

    }

}
