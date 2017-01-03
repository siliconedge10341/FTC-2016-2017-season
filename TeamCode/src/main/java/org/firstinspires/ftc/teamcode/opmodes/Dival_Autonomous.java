package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.DeviceConfiguration;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.classes.Mecanum;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.VisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;


/**
 * Created by Dival Banerjee on 12/28/2016.
 * Hopefully this will work better
 * "Shut up anirudh" - Juan
 */

@Autonomous(name = "Dival's_Autonomous", group = "Blue")

public class Dival_Autonomous extends VisionOpMode{

    //variable declarations:
    //Drivetrain motors
    private DcMotor mtrFrontRight;
    private DcMotor mtrFrontLeft;
    private DcMotor mtrBackRight;
    private DcMotor mtrBackLeft;

    //Shooter Motors
    private DcMotor mtrShooterRight;
    private DcMotor mtrShooterLeft;

    //Servos
    private Servo svoBallRelaease;

    //Sensors
    private ModernRoboticsI2cRangeSensor rangeSide;
    private ModernRoboticsI2cRangeSensor rangeFront;
    private OpticalDistanceSensor ods;

    //Drivetrain
    private Mecanum Drive_Train = new Mecanum();

    //Loop runTime
    private ElapsedTime runtime = new ElapsedTime();

    //Colors for beacons
    //This is the initial color on the floor for comparison
    private double initialColor = 0;
    //This is the compared color that is used to determine whether the line is detected or not
    private double comparedColor = 0;
    //Used to keep track of the different stages completed during autonomous
    private int v_state;

    private static final int ENCODER_TICKS = 1440;
    private static final double Wheel_Circumfrence = 3.14*4;

    private static final int Max_Shooter_Speed = 1784;
    private static final int Safe_Shooter_Speed  = Max_Shooter_Speed - 100;

    public Dival_Autonomous(){
        // Default constructor
    }

    @Override
    public void init(){

        //Hardware mapping
        this.mtrFrontRight =  hardwareMap.dcMotor.get("fr_motor");
        this.mtrFrontLeft = hardwareMap.dcMotor.get("fl_motor");
        this.mtrBackRight = hardwareMap.dcMotor.get("br_motor");
        this.mtrBackLeft = hardwareMap.dcMotor.get("bl_motor");

        this.mtrShooterLeft = hardwareMap.dcMotor.get("shooter_left");
        this.mtrShooterRight= hardwareMap.dcMotor.get("shooter_right");
        this.svoBallRelaease = hardwareMap.servo.get("servo_ball");

        this.ods = hardwareMap.opticalDistanceSensor.get("ods_line");

        //TODO:WE need to rename our current range sensor on the side in the config - ask pavan to do this
        //TODO: Also install a second range sensor for forward distance
        this.rangeSide = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_side");
        this.rangeFront = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "senor_range_front");

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

        //setDriveMode(DcMotorController.RunMode.Run_Without_Encoder)

    }

    @Override
    public void start() {
        // start
        super.start();

        // reset encoders to begin period of autonomous
      //  Drive_Train.reset_encoders(fr, fl, br, bl);
    }


    public void loop(){
        super.loop();
        //Different cases for autonomous, step by step


        switch(v_state){

            case 1:
                //Configure robot for autonomous
                svoBallRelaease.setPosition(.5); // Move beacon pusher out of the way
                initialColor = this.ods.getLightDetected();
                telemetry.addData("Stage 1:","Confuration complete");
                break;
            case 2:
                //shoot balls
                double leftMotorCalculatedPower = 0;
                double rightMotorCalculatedPower = 0;

                double leftMotorRPM =  0;
                double rightMotorRPM = 0;


                while(leftMotorRPM < Safe_Shooter_Speed){
                    leftMotorCalculatedPower = leftMotorCalculatedPower + .05;
                    mtrShooterLeft.setPower(leftMotorCalculatedPower);
                    telemetry.addData("Left Shooter Power:",leftMotorCalculatedPower);
                    this.Wait(10);
                    int initialEncoderValue = mtrShooterLeft.getCurrentPosition();
                    this.Wait(10);
                    int DistanceTraveled = mtrShooterLeft.getCurrentPosition() - initialEncoderValue;
                    double RotationsPerMillisecond = ENCODER_TICKS * (DistanceTraveled/10);
                    double RotationsPerMinute = RotationsPerMillisecond * 1000 * 60;
                    telemetry.addData("Left Shooter RPM:", RotationsPerMinute);
                }



                break;
            case 3:
                //Move forward

                break;
            case 4:
                //Move toward beacon
                break;
            case 5:
                //Beacon 1
                break;
            case 6:
                //Move to beacon 2
                break;
            case 7:
                //Beacon 2
                break;
            case 8:
                //Knock cap ball off
                break;
            default:
                stop();
                break;
        }

    }
    @Override
    public void stop() {
        super.stop();
    }


    public void Wait(int delayInMilliseconds){
        long myStartTime  = System.currentTimeMillis();
        long myCurrentTime  = System.currentTimeMillis();

        do {
        }while(myCurrentTime-myStartTime < delayInMilliseconds);
    }



}
