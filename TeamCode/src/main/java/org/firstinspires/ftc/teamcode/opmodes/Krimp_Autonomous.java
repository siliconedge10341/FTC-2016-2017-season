// Package for -THIS- project
package org.firstinspires.ftc.teamcode.opmodes;

// Imports for variables and phone configuration
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
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

@Autonomous(name = "Final_Countdown_Blue", group = "Blue")
public class Krimp_Autonomous extends LinearVisionOpMode {
    // instance variables
    // private data
    // Motors
        private DcMotor mtrFR;
        private DcMotor mtrFL;
        private DcMotor mtrBR;
        private DcMotor mtrBL;
        private DcMotor mtrShootT;
        private DcMotor mtrShootB;

    // Servos
        private Servo srvRelease;

    // Classes
        private Mecanum drive_train = new Mecanum();
        private ModernRoboticsI2cRangeSensor rangeF;
        private ModernRoboticsI2cRangeSensor rangeSF;
        private ModernRoboticsI2cRangeSensor rangeSB;
        private OpticalDistanceSensor colourS;
        private ElapsedTime runtime = new ElapsedTime();
        private ElapsedTime total_time = new ElapsedTime();

    // Variables
        // Reading for the initial color we zz take at the beginning of the match.
        // This helps us because when we test for the white line, we want to be
        // able to tell the difference from the color of the ground. Thus
        // knowing where the sensor is.
        private double initialC = 0;
        private double initialD = 0;
        private int v_state = 0;

        // states variable for the loop
        private static final Double ticks_per_inch = 510 / (3.1415 * 4);

    // public data
        // None;

    // Constructors
    public Krimp_Autonomous() {
        // Default Constructor

    }

    // Initialization
    private void initialize() throws InterruptedException {
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
        mtrShootB = hardwareMap.dcMotor.get("shooter_right"); // Bottom
        mtrShootT = hardwareMap.dcMotor.get("shooter_left");  // Top

        mtrShootT.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        mtrShootB.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

        // Servos
        srvRelease = hardwareMap.servo.get("servo_ball");

        // Classes
        colourS = hardwareMap.opticalDistanceSensor.get("ods_line");
        rangeF = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_front");
        rangeSF = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_side_left");
        rangeSB = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_side_right");

        // Positions
        srvRelease.setPosition(srvRelease.MAX_POSITION);

        waitForVisionStart();

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
    private void startRobot() {
        //
        // Starts the -LOOP-.
        //
        // Resets encoders to begin -LOOP-.
        drive_train.reset_encoders(mtrFR, mtrFL, mtrBR, mtrBL);

    }

    // Loop
    public void runOpMode() throws InterruptedException {
        //
        // Loop that controls the -AUTONOMOUS- period.
        //
        // SHOOTING STATE:
        // v_state == 0 is all about a few seconds where the robot moves the launching servo
        // to release to balls. Then, after 6 seconds. the robot will move into the next state.
        //
        v_state = 0;
        telemetry.addData("Current State: ", v_state);
        telemetry.update();
        this.initialize();
        this.startRobot();
        total_time.reset();
        total_time.startTime();
        //
        // Time starts
        //
        initialC = colourS.getLightDetected();
        telemetry.addData("Light WaveLength", initialC);
        telemetry.update();
        //
        // Shoots ball for 3 seconds
        // One ball by the releases servo and there is one ball in the collector so there
        // is a delay between the shots. This allows for smoother transitions.
        //
        mtrShootT.setPower(0.7);
        mtrShootB.setPower(-0.5);
        runtime.reset();
        runtime.startTime();
        while (runtime.seconds() < 1.0) {
            //
            // Lets the spinners speed up
            //
        }
        srvRelease.setPosition(0.85);
        runtime.reset();
        runtime.startTime();
        while (runtime.seconds() < 1.0) {
            //
            // Shoots ball
            //
        }
        mtrShootT.setPower(0.0);
        mtrShootB.setPower(0.0);
        srvRelease.setPosition(srvRelease.MAX_POSITION);
        runtime.reset();
        runtime.startTime();
        while (runtime.seconds() < 0.75) {
            //
            // Loads another ball
            //
        }
        mtrShootT.setPower(0.7);
        mtrShootB.setPower(-0.5);
        runtime.reset();
        runtime.startTime();
        while (runtime.seconds() < 1.0) {
            //
            // Lets the spinners speed up
            //
        }
        srvRelease.setPosition(0.85);
        runtime.reset();
        runtime.startTime();
        while (runtime.seconds() < 1.0) {
            //
            // Shoots ball
            //
        }
        srvRelease.setPosition(srvRelease.MAX_POSITION);
        mtrShootT.setPower(0);
        mtrShootB.setPower(0);

        //
        // Wait...
        //
        // TURNING STATE
        // v_state == 1 is all about turning 90 degrees to the left to make sure that the
        // touching servo, the range sensor, and the beacon are all facing the wall.
        //
        v_state = 1;
        telemetry.addData("Current State: ", v_state);
        telemetry.update();
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
        v_state = 2;
        telemetry.addData("Current State: ", v_state);
        telemetry.update();
        drive_train.setPowerD(0.6);
        drive_train.run_diagonal_right_up(mtrFR, mtrFL, mtrBR, mtrBL);
        runtime.reset();
        runtime.startTime();
        while (runtime.seconds() < 8 || colourS.getLightDetected() < initialC + .1) {
            telemetry.addData("Colour WaveLength", colourS.getLightDetected());
            telemetry.update();
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
        v_state = 3;
        telemetry.addData("Current State: ", v_state);
        telemetry.update();
        PauseAuto(0.4);
        //
        // Pause to see colour
        //
        telemetry.addData("Beacon" , beacon.getAnalysis().toString());
        telemetry.update();

        if (beacon.getAnalysis().isLeftBlue() && beacon.getAnalysis().isRightBlue()) {
            //
            // They are both correct, do nothing.
            //

        } else if (beacon.getAnalysis().isRightBlue() && beacon.getAnalysis().isLeftRed()) {
            //
            // Go forward if the left side of the beacon is red.
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
            v_state = 4;
            telemetry.addData("Current State: ", v_state);
            telemetry.update();

            encoderDrive(12.0, "right", 0.6);

            PauseAuto(0.2);
            //
            // Wait...
            //
            // STRAFE LEFT
            // v_state 5 is about returning to where you were before pressing the button.
            // After this, the next state will be moving to the next beacon.
            //
            v_state = 5;
            telemetry.addData("Current State: ", v_state);
            telemetry.update();

            encoderDrive(10.0, "left" , 0.6);

        } else if (beacon.getAnalysis().isLeftBlue() && beacon.getAnalysis().isRightRed()) {
            //
            // Go backward if the right side of the beacon is red.
            // Beacon is 1/2 a foot, presser is on the right side so it is lined up with the line
            //
            // Wait...
            //

            encoderDrive(6.0, "backward", 0.6);

            PauseAuto(0.2);
            //
            // STRAFE RIGHT
            // v_state == 4 is when the robot goes in for the points and presses the -FIRST BEACON-
            // After pressing the button, it will go into the next state, going back to original position.
            //
            v_state = 4;
            telemetry.addData("Current State: ", v_state);
            telemetry.update();

            encoderDrive(12.0, "right", 0.6);

            PauseAuto(0.2);
            //
            // Wait...
            //
            // STRAFE LEFT
            // v_state 5 is about returning to where you were before pressing the button.
            // After this, the next state will be moving to the next beacon.
            //
            v_state = 5;
            telemetry.addData("Current State: ", v_state);
            telemetry.update();

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
        v_state = 6;
        telemetry.addData("Current State: ", v_state);
        telemetry.update();
        drive_train.setPowerD(0.6);
        drive_train.run_forward(mtrFR, mtrFL, mtrBR, mtrBL);
        runtime.reset();
        runtime.startTime();
        while (runtime.seconds() < 6 || colourS.getLightDetected() < initialC + .1) {
            telemetry.addData("Colour", colourS.getLightDetected());
            telemetry.addData("Seconds", runtime.seconds());
            telemetry.update();
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
        v_state = 7;
        telemetry.addData("Current State: ", v_state);
        telemetry.update();
        PauseAuto(0.4);
        //
        // Pause to see colour
        //
        telemetry.addData("Beacon" , beacon.getAnalysis().toString());
        telemetry.update();

        if (beacon.getAnalysis().isLeftBlue() && beacon.getAnalysis().isRightBlue()) {
            //
            // They are both correct, do nothing.
            //

        } else if (beacon.getAnalysis().isRightBlue() && beacon.getAnalysis().isLeftRed()) {
            //
            // Go forward if the left side of the beacon is red.
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
            v_state = 8;
            telemetry.addData("Current State: ", v_state);
            telemetry.update();

            encoderDrive(12.0, "right", 0.6);

            PauseAuto(0.2);
            //
            // Wait...
            //
            // STRAFE LEFT
            // v_state 9 is about returning to where you were before pressing the button.
            // After this, the next state will be moving to the next beacon.
            //
            v_state = 9;
            telemetry.addData("Current State: ", v_state);
            telemetry.update();

            encoderDrive(10.0, "left" , 0.6);

        } else if (beacon.getAnalysis().isLeftBlue() && beacon.getAnalysis().isRightRed()) {
            //
            // Go backward if the right side of the beacon is red.
            // Beacon is 1/2 a foot, presser is on the right side so it is lined up with the line
            //
            // Wait...
            //

            encoderDrive(6.0, "backward", 0.6);

            PauseAuto(0.2);
            //
            // STRAFE RIGHT
            // v_state == 8 is when the robot goes in for the points and presses the -FIRST BEACON-
            // After pressing the button, it will go into the next state, going back to original position.
            //
            v_state = 8;
            telemetry.addData("Current State: ", v_state);
            telemetry.update();

            encoderDrive(12.0, "right", 0.6);

            PauseAuto(0.2);
            //
            // Wait...
            //
            // STRAFE LEFT
            // v_state 9 is about returning to where you were before pressing the button.
            // After this, the next state will be moving to the next beacon.
            //
            v_state = 9;
            telemetry.addData("Current State: ", v_state);
            telemetry.update();

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
            v_state = 10;
            telemetry.addData("Current State: ", v_state);
            telemetry.update();
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
            v_state = 10;
            telemetry.addData("Current State: ", v_state);
            telemetry.update();
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
        this.stopRobot();

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
    private void stopRobot() {
        //
        // Stops -LOOP- and ends Program.
        //
       drive_train.brake(mtrFR, mtrFL, mtrBR, mtrBL);

    }

}
