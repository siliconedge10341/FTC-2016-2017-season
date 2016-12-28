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
 * @author SSI Robotics
 * @version 2015-08-01-06-01
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
        Drive_Train.reset_encoders(fr, fl, br, bl);
    }

    @Override
    public void loop() {
        //
        // Proceed with the loop
        //
        super.loop();
        if (v_state == 0) {
            //
            // SHOOTING STATE:
            // v_state == 0 is all about a few seconds where the robot moves the launching servo
            // to release to balls. Then, after 6 seconds. the robot will move into the next state.
            //
            initialC = ods.getLightDetected();
            telemetry.addData("Light WaveLength", initialC);
            //Shoots ball for 6 seconds

            releaseServo.setPosition(.9);
            motorShootL.setPower(1.0);
            motorShootR.setPower(-1.0);
            runtime.reset();
            while (runtime.seconds() < 6) {
                telemetry.addData("Seconds", runtime.seconds());
            }
            motorShootL.setPower(0);
            motorShootR.setPower(0);
            v_state++;
            //
            // Wait...
            //
        } else if (v_state == 1) {
            //
            // TURNING STATE
            // v_state == 1 is all about turning 90 degrees to the left to make sure that the
            // touching servo, the range sensor, and the beacon are all facing the wall.
            //
            Drive_Train.setPowerD(1.0);
            Drive_Train.setPosition((5/3) * 1440,-(5/3) * 1440,(5/3) * 1440,-(5/3) * 1440, fr, fl, br, bl);
            Drive_Train.run_to_position(fr, fl, br, bl);
            Drive_Train.turn_left(fr, fl, br, bl, 180);
            while (Drive_Train.testDistance(fr) == 1) {
                telemetry.addData("Position", fr.getCurrentPosition());
            }
            Drive_Train.brake(fr, fl, br, bl);
            Drive_Train.reset_encoders(fr, fl, br, bl);
            v_state++;
            //
            // Wait...
            //
        } else if (v_state == 2) {
            //
            // DIAGONAL RIGHT UP
            // v_state == 2 is all about moving the robot to it's specific position at the white
            // line up by the coloured box. From there we go into the next state. Repositioning.
            //
            Drive_Train.setPowerD(1.0);
            Drive_Train.setPosition(6 * 1440,0,0,6 * 1440, fr, fl, br, bl);
            Drive_Train.run_to_position(fr, fl, br, bl);
            Drive_Train.run_diagonal_right_up(fr, fl, br, bl);
            while (Drive_Train.testDistance(fr) == 1 || ods.getLightDetected() >= initialC + .1 || ods.getLightDetected() <= initialC - .1) {
                telemetry.addData("Colour WaveLength", ods.getLightDetected());
            }
            Drive_Train.brake(fr, fl, br, bl);
            Drive_Train.reset_encoders(fr, fl, br, bl);
            v_state++;
            //
            // Wait...
            //
        } else if (v_state == 3) {
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
                Drive_Train.setPosition((1/4) * 1440,(1/4) * 1440,(1/4) * 1440,(1/4) * 1440, fr, fl, br, bl);
                Drive_Train.run_to_position(fr, fl, br, bl);
                Drive_Train.run_forward(fr, fl, br, bl);
                while (Drive_Train.testDistance(fr) == 1) {
                    // motor to move button here
                    telemetry.addData("Position", fr.getCurrentPosition());
                }
                Drive_Train.reset_encoders(fr, fl, br, bl);
                Drive_Train.brake(fr, fl, br, bl);
                v_state++;
                //
                // Wait...
                //
            } else if (beacon.getAnalysis().isRightBlue() == true) {
                // go backward if the righgt side of the beacon is blue.
                Drive_Train.setPosition(-(1/4) * 1440,-(1/4) * 1440,-(1/4) * 1440,-(1/4) * 1440, fr, fl, br, bl);
                Drive_Train.run_to_position(fr, fl, br, bl);
                Drive_Train.run_backward(fr, fl, br, bl);
                while (Drive_Train.testDistance(fr) == 1) {
                    // motor to move button here
                    telemetry.addData("Position", fr.getCurrentPosition());
                }
                Drive_Train.reset_encoders(fr, fl, br, bl);
                Drive_Train.brake(fr, fl, br, bl);
                v_state++;
                //
                // Wait...
                //
            }
        } else if (v_state == 4) {
            //
            // STRAFE RIGHT
            // v_state == 4 is when the robot goes in for the points and presses the -FIRST BEACON-
            // After pressing the button, it will go into the next state, going back to original position.
            //
            initialD = RANGE.getDistance(DistanceUnit.METER);
            Drive_Train.setPosition(2 * 1440,-2 * 1440,2 * 1440,-2 * 1440, fr, fl, br, bl);
            Drive_Train.run_to_position(fr, fl, br, bl);
            Drive_Train.run_right(fr, fl, br, bl);
            if (RANGE.getDistance(DistanceUnit.METER) <= initialD - .45) {
                // motor to move button here
                telemetry.addData("Distance", RANGE.getDistance(DistanceUnit.METER));
            }
            Drive_Train.reset_encoders(fr, fl, br, bl);
            Drive_Train.brake(fr, fl, br, bl);
            v_state++;
            //
            // Wait...
            //
        } else if (v_state == 5) {
            //
            // STRAFE LEFT
            // v_state 5 is about returning to where you were before pressing the button.
            // After this, the next state will be moving to the next beacon.
            //
            Drive_Train.setPosition(-2 * 1440,2 * 1440,-2 * 1440,2 * 1440, fr, fl, br, bl);
            Drive_Train.run_to_position(fr, fl, br, bl);
            Drive_Train.run_left(fr, fl, br, bl);
            while (RANGE.getDistance(DistanceUnit.METER) >= initialD) {
                telemetry.addData("Distance", RANGE.getDistance(DistanceUnit.METER));
                // motor to move button here
            }
            Drive_Train.reset_encoders(fr, fl, br, bl);
            Drive_Train.brake(fr, fl, br, bl);
            v_state++;
            //
            // Wait...
            //
        } else if (v_state == 6) {
            //
            // RUN FORWARD
            // v_state 6 is all about going to the next colour pad and pressing the second
            // button. We will use the color sensor to read the second line. Then we will
            // repeat the same steps to press the button again.
            //
            initialD = 0;
            Drive_Train.setPosition(10 * 1440,10 * 1440,10 * 1440,10 * 1440, fr, fl, br, bl);
            Drive_Train.run_to_position(fr, fl, br, bl);
            Drive_Train.run_forward(fr, fl, br, bl);
            while (Drive_Train.testDistance(fr) == 1 || ods.getLightDetected() >= initialC + .1 || ods.getLightDetected() <= initialC - .1) {
                // motor to move button here
                telemetry.addData("Colour WaveLength", ods.getLightDetected());
            }
            Drive_Train.reset_encoders(fr, fl, br, bl);
            Drive_Train.brake(fr, fl, br, bl);
            v_state++;
            //
            // Wait...
            //
        } else if (v_state == 7) {
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
                Drive_Train.setPosition((1/4) * 1440,(1/4) * 1440,(1/4) * 1440,(1/4) * 1440, fr, fl, br, bl);
                Drive_Train.run_to_position(fr, fl, br, bl);
                Drive_Train.run_forward(fr, fl, br, bl);
                while (Drive_Train.testDistance(fr) == 1) {
                    // motor to move button here
                    telemetry.addData("Position", fr.getCurrentPosition());
                }
                Drive_Train.reset_encoders(fr, fl, br, bl);
                Drive_Train.brake(fr, fl, br, bl);
                v_state++;
                //
                // Wait...
                //
            } else if (beacon.getAnalysis().isRightBlue() == true) {
                // go backward if the righgt side of the beacon is blue.
                Drive_Train.setPosition(-(1/4) * 1440,-(1/4) * 1440,-(1/4) * 1440,-(1/4) * 1440, fr, fl, br, bl);
                Drive_Train.run_to_position(fr, fl, br, bl);
                Drive_Train.run_backward(fr, fl, br, bl);
                while (Drive_Train.testDistance(fr) == 1) {
                    // motor to move button here
                    telemetry.addData("Position", fr.getCurrentPosition());
                }
                Drive_Train.reset_encoders(fr, fl, br, bl);
                Drive_Train.brake(fr, fl, br, bl);
                v_state++;
                //
                // Wait...
                //
            }
        } else if (v_state == 8) {
            //
            // STRAFE RIGHT
            // v_state == 4 is when the robot goes in for the points and presses the -FIRST BEACON-
            // After pressing the button, it will go into the next state, going back to original position.
            //
            initialD = RANGE.getDistance(DistanceUnit.METER);
            Drive_Train.setPosition(2 * 1440,-2 * 1440,2 * 1440,-2 * 1440, fr, fl, br, bl);
            Drive_Train.run_to_position(fr, fl, br, bl);
            Drive_Train.run_right(fr, fl, br, bl);
            if (RANGE.getDistance(DistanceUnit.METER) <= initialD - .45) {
                // motor to move button here
                telemetry.addData("Distance", RANGE.getDistance(DistanceUnit.METER));
            }
            Drive_Train.reset_encoders(fr, fl, br, bl);
            Drive_Train.brake(fr, fl, br, bl);
            v_state++;
            //
            // Wait...
            //
        } else if (v_state == 9) {
            //
            // STRAFE LEFT
            // v_state 9 is about returning to where you were before pressing the button.
            // After this, the next state will be moving to the next beacon.
            //
            Drive_Train.setPosition(-2 * 1440,2 * 1440,-2 * 1440,2 * 1440, fr, fl, br, bl);
            Drive_Train.run_to_position(fr, fl, br, bl);
            Drive_Train.run_left(fr, fl, br, bl);
            while (RANGE.getDistance(DistanceUnit.METER) >= initialD) {
                telemetry.addData("Distance", RANGE.getDistance(DistanceUnit.METER));
                // motor to move button here
            }
            Drive_Train.reset_encoders(fr, fl, br, bl);
            Drive_Train.brake(fr, fl, br, bl);
            v_state++;
            //
            // Wait...
            //
        } else if (v_state == 10) {
            //
            // DIAGONAL DOWN LEFT
            // v_state 10 is about hitting the cat ball and staying on the middle platform
            // to gain the final points for the autonomous period. After this and the position
            // it stays at. The v_state will increase into nothing. Therefore, it will stop and
            // end. This is for 90 points in the Velocity vortex.
            //
            Drive_Train.setPosition(-6 * 1440,0,0,-6 * 1440, fr, fl, br, bl);
            Drive_Train.run_to_position(fr, fl, br, bl);
            Drive_Train.run_diagonal_left_down(fr, fl, br, bl);
            //
            // Wait...
            //
            // Program Ends
            //
        } else {
            //
            // Program ends
            //
            stop();
        }
        telemetry.addData("v_state", v_state);
    }

    @Override
    public void stop() {
        super.stop();
    }
}
