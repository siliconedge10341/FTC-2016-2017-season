// Package for -THIS- project
package org.firstinspires.ftc.teamcode.opmodes;

// Imports for variables and phone configuration
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.classes.Light;
import org.firstinspires.ftc.teamcode.classes.Mecanum;
import org.firstinspires.ftc.teamcode.classes.Range;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
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

    // Servos
        private Servo srvRelease;

    // Classes
        private Mecanum drive_train = new Mecanum();
        private Range range_sensor_beacon = new Range();
        private Range range_sensor_proj = new Range();
        private Light color_sensor = new Light();
        private ElapsedTime runtime = new ElapsedTime();

    // Variables
        // Reading for the initial color we zz take at the beginning of the match.
        // This helps us because when we test for the white line, we want to be
        // able to tell the difference from the color of the ground. Thus
        // knowing where the sensor is.
        private double initialC = 0;
        private double initialD = 0;

        // states variable for the loop
        private static final int ticks = 1440;
        private static final double WheelC = 3.14*4;

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

        // Servos
        srvRelease = hardwareMap.servo.get("servo_ball");

        // Classes
        color_sensor.setV_sensor(hardwareMap.opticalDistanceSensor.get("ods_line"));
        range_sensor_beacon.setRange(hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_side"));
        range_sensor_proj.setRange(hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_front"));

        // Positions
        srvRelease.setPosition(0.3);

        super.init();
        this.setCamera(Cameras.PRIMARY);
        this.setFrameSize(new Size(900, 900));

        enableExtension(VisionOpMode.Extensions.BEACON);         // Beacon detection.
        enableExtension(VisionOpMode.Extensions.ROTATION);       // Automatic screen rotation correction.
        enableExtension(VisionOpMode.Extensions.CAMERA_CONTROL); // Manual camera control.
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
        initialC = color_sensor.getLightDetected();
        telemetry.addData("Light WaveLength", initialC);
        //Shoots ball for 3 seconds

        srvRelease.setPosition(srvRelease.MAX_POSITION);
        mtrShootL.setPower(1.0);
        mtrShootR.setPower(-1.0);
        runtime.reset();
        while (runtime.seconds() < 3) {
            telemetry.addData("Seconds", runtime.seconds());
        }
        mtrShootL.setPower(0);
        mtrShootR.setPower(0);

        //
        // Wait...
        //
        // TURNING STATE
        // v_state == 1 is all about turning 90 degrees to the left to make sure that the
        // touching servo, the range sensor, and the beacon are all facing the wall.
        //
        drive_train.setPowerD(1.0);
        drive_train.turn_left(mtrFR, mtrFL, mtrBR, mtrBL);
        runtime.reset();
        while (runtime.seconds() < 1) {
            telemetry.addData("Seconds", runtime.seconds());
        }
        drive_train.brake(mtrFR, mtrFL, mtrBR, mtrBL);
        drive_train.reset_encoders(mtrFR, mtrFL, mtrBR, mtrBL);

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
        while (runtime.seconds() < 5 || color_sensor.getLightDetected() > initialC + 0.1) {
            telemetry.addData("Colour WaveLength", color_sensor.getLightDetected());
        }
        drive_train.brake(mtrFR, mtrFL, mtrBR, mtrBL);
        drive_train.reset_encoders(mtrFR, mtrFL, mtrBR, mtrBL);

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
        if (beacon.getAnalysis().isLeftBlue() == true) {
            // go forward if the left side of the beacon is blue.
            drive_train.setPowerD(0.2);
            drive_train.run_forward(mtrFR, mtrFL, mtrBR, mtrBL);
            runtime.reset();
            while (runtime.seconds() < 1) {
                // motor to move button here
                telemetry.addData("Seconds", runtime.seconds());
            }
            drive_train.brake(mtrFR, mtrFL, mtrBR, mtrBL);
            drive_train.reset_encoders(mtrFR, mtrFL, mtrBR, mtrBL);

        } else if (beacon.getAnalysis().isRightBlue() == true) {
            // go backward if the right side of the beacon is blue.
            drive_train.setPowerD(0.2);
            drive_train.run_backward(mtrFR, mtrFL, mtrBR, mtrBL);
            runtime.reset();
            while (runtime.seconds() < 1) {
                // motor to move button here
                telemetry.addData("Seconds", runtime.seconds());
            }
            drive_train.brake(mtrFR, mtrFL, mtrBR, mtrBL);
            drive_train.reset_encoders(mtrFR, mtrFL, mtrBR, mtrBL);

        }

        //
        // Wait...
        //
        // STRAFE RIGHT
        // v_state == 4 is when the robot goes in for the points and presses the -FIRST BEACON-
        // After pressing the button, it will go into the next state, going back to original position.
        //
        initialD = range_sensor_beacon.getData();
        drive_train.setPowerD(0.3);
        drive_train.run_right(mtrFR, mtrFL, mtrBR, mtrBL);
        runtime.reset();
        if (range_sensor_beacon.getData() <= initialD - .40 || runtime.seconds() < 4) {
            // motor to move button here
            telemetry.addData("Seconds", runtime.seconds());
            telemetry.addData("Distance", range_sensor_beacon.getData());
        }
        drive_train.brake(mtrFR, mtrFL, mtrBR, mtrBL);
        drive_train.reset_encoders(mtrFR, mtrFL, mtrBR, mtrBL);

        //
        // Wait...
        //
        // STRAFE LEFT
        // v_state 5 is about returning to where you were before pressing the button.
        // After this, the next state will be moving to the next beacon.
        //
        drive_train.setPowerD(0.3);
        drive_train.run_left(mtrFR, mtrFL, mtrBR, mtrBL);
        runtime.reset();
        while (range_sensor_beacon.getData() + .05 >= initialD || runtime.seconds() < 4) {
            // motor to move away from here
            telemetry.addData("Seconds", runtime.seconds());
            telemetry.addData("Distance", range_sensor_beacon.getData());
        }
        drive_train.brake(mtrFR, mtrFL, mtrBR, mtrBL);
        drive_train.reset_encoders(mtrFR, mtrFL, mtrBR, mtrBL);

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
        while (runtime.seconds() < 4 || color_sensor.getLightDetected() > initialC + 0.1) {
            telemetry.addData("Colour", color_sensor.getLightDetected());
            telemetry.addData("Seconds", runtime.seconds());
        }
        drive_train.brake(mtrFR, mtrFL, mtrBR, mtrBL);
        drive_train.reset_encoders(mtrFR, mtrFL, mtrBR, mtrBL);

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
        if (beacon.getAnalysis().isLeftBlue() == true) {
            // go forward if the left side of the beacon is blue.
            drive_train.setPowerD(0.2);
            drive_train.run_forward(mtrFR, mtrFL, mtrBR, mtrBL);
            while (runtime.seconds() < 1) {
                // motor to move button here
                telemetry.addData("Seconds", runtime.seconds());
            }
            drive_train.reset_encoders(mtrFR, mtrFL, mtrBR, mtrBL);
            drive_train.brake(mtrFR, mtrFL, mtrBR, mtrBL);

        } else if (beacon.getAnalysis().isRightBlue() == true) {
            // go backward if the right side of the beacon is blue.
            drive_train.setPowerD(0.2);
            drive_train.run_backward(mtrFR, mtrFL, mtrBR, mtrBL);
            while (runtime.seconds() < 1) {
                // motor to move button here
                telemetry.addData("Seconds", runtime.seconds());
            }
            drive_train.reset_encoders(mtrFR, mtrFL, mtrBR, mtrBL);
            drive_train.brake(mtrFR, mtrFL, mtrBR, mtrBL);

        }

        //
        // Wait...
        //
        // STRAFE RIGHT
        // v_state == 8 is when the robot goes in for the points and presses the -FIRST BEACON-
        // After pressing the button, it will go into the next state, going back to original position.
        //
        initialD = range_sensor_beacon.getData();
        drive_train.setPowerD(0.3);
        drive_train.run_right(mtrFR, mtrFL, mtrBR, mtrBL);
        runtime.reset();
        if (range_sensor_beacon.getData() <= initialD - .40 || runtime.seconds() < 4) {
            // motor to move button here
            telemetry.addData("Seconds", runtime.seconds());
            telemetry.addData("Distance", range_sensor_beacon.getData());
        }
        drive_train.brake(mtrFR, mtrFL, mtrBR, mtrBL);
        drive_train.reset_encoders(mtrFR, mtrFL, mtrBR, mtrBL);

        //
        // Wait...
        //
        // STRAFE LEFT
        // v_state 9 is about returning to where you were before pressing the button.
        // After this, the next state will be moving to the next beacon.
        //
        drive_train.setPowerD(0.3);
        drive_train.run_left(mtrFR, mtrFL, mtrBR, mtrBL);
        runtime.reset();
        while (range_sensor_beacon.getData() + .05 >= initialD || runtime.seconds() < 4) {
            // motor to move away from here
            telemetry.addData("Seconds", runtime.seconds());
            telemetry.addData("Distance", range_sensor_beacon.getData());
        }
        drive_train.brake(mtrFR, mtrFL, mtrBR, mtrBL);
        drive_train.reset_encoders(mtrFR, mtrFL, mtrBR, mtrBL);

        //
        // Wait...
        //
        // DIAGONAL DOWN LEFT
        // v_state 10 is about hitting the cat ball and staying on the middle platform
        // to gain the final points for the autonomous period. After this and the position
        // it stays at. The v_state will increase into nothing. Therefore, it will stop and
        // end. This is for 90 points in the Velocity vortex.
        //
        drive_train.setPowerD(0.6);
        drive_train.run_diagonal_left_down(mtrFR, mtrFL, mtrBR, mtrBL);
        runtime.reset();
        while (runtime.seconds() < 5) {
            // motor to move away from here
            telemetry.addData("Seconds", runtime.seconds());
        }
        drive_train.brake(mtrFR, mtrFL, mtrBR, mtrBL);
        drive_train.reset_encoders(mtrFR, mtrFL, mtrBR, mtrBL);
        //
        // Program Ends...
        //
        // Wait...
        //
        // Stops the program.
        //
        stop();

    }

    // Stop
    public void stop() {
        //
        // Stops -LOOP- and ends Program.
        //
        super.stop();

    }
}
