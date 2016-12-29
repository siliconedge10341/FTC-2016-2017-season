package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.classes.Mecanum;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.VisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

/**
 * Created by Jose Martinez on 12/28/2016.
 */

@Autonomous(name = "Shut_Up_Anirudh", group = "Blue")
public class Jose_Autonomous extends VisionOpMode {

    // instance variables
    // private variables
    // Motors
    private DcMotor motorFR;
    private DcMotor motorFL;
    private DcMotor motorBL;
    private DcMotor motorBR;
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

    public Jose_Autonomous() {} // this is a default constructor, igrnore for now -Dival

    @Override
    public void init() {
        // Sets every class at the beginning of the demoautonomous run class
        //Hardware Maps
        motorFR = hardwareMap.dcMotor.get("fr_motor");
        motorFL = hardwareMap.dcMotor.get("fl_motor");
        motorBR = hardwareMap.dcMotor.get("br_motor");
        motorBL = hardwareMap.dcMotor.get("bl_motor");

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
        //Drive_Train.reset_encoders(motorFR, motorFL, motorBR, motorBL);
    }


    public void loop(){
        super.loop();
        switch (v_state) {
            case 0:
                Drive_Train.setPowerD(0.5);
                Drive_Train.run_forward(motorFR,motorFL,motorBR,motorBL);

                runtime.reset();
                while (runtime.seconds() < 1) {
                    telemetry.addData("seconds", runtime.seconds());
                    telemetry.update();
                }
                Drive_Train.brake(motorFR,motorFL,motorBR,motorBL);

                Drive_Train.setPowerD(0.5);
                Drive_Train.run_left(motorFR,motorFL,motorBR,motorBL);

                runtime.reset();
                while (runtime.seconds() < 1) {
                    telemetry.addData("seconds", runtime.seconds());
                    telemetry.update();
                }
                Drive_Train.brake(motorFR,motorFL,motorBR,motorBL);

                Drive_Train.setPowerD(0.5);
                Drive_Train.run_backward(motorFR,motorFL,motorBR,motorBL);

                runtime.reset();
                while (runtime.seconds() < 1) {
                    telemetry.addData("seconds", runtime.seconds());
                    telemetry.update();
                }
                Drive_Train.brake(motorFR,motorFL,motorBR,motorBL);

                Drive_Train.setPowerD(0.5);
                Drive_Train.run_right(motorFR,motorFL,motorBR,motorBL);

                runtime.reset();
                while (runtime.seconds() < 1) {
                    telemetry.addData("seconds", runtime.seconds());
                    telemetry.update();
                }
                Drive_Train.brake(motorFR,motorFL,motorBR,motorBL);

                break;
        }
    }
    @Override
    public void stop() {
        super.stop();
    }
}
