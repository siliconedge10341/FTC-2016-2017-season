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
    private int v_state = 0; // Used for the loop as a counter per states.

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
        // Reading for the initial color we take at the beginning of the match.
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
        v_state = 0;
    }

    // Initialization
    public void init() {
        //
        // Initializes every motor, servo, variable, and position.
        //
        v_state = 0;
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
        range_sensor_beacon.setRange(hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_beacon"));
        range_sensor_proj.setRange(hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_proj"));

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

    }

    // Loop
    public void loop() {
        //
        // Loop that controls the -AUTONOMOUS- period.
        //
        super.loop();

        switch (v_state) {
            case 0:

                break;
            //
            // Wait...
            //
            case 1:

                break;
            //
            // Wait...
            //
            case 2:

                break;
            //
            // Wait...
            //
            case 3:

                break;
            //
            // Wait...
            //
            case 4:

                break;
            //
            // Wait...
            //
            case 5:

                break;
            //
            // Wait...
            //
            case 6:

                break;
            //
            // Wait...
            //
            case 7:

                break;
            //
            // Wait...
            //
            case 8:

                break;
            //
            // Wait...
            //
            case 9:

                break;
            //
            // Wait...
            //
            case 10:

                break;
            //
            // Wait...
            //
            default:
                //
                // Stops the program.
                //
                stop();
                break;
        }
    }

    // Stop
    public void stop() {
        //
        // Stops -LOOP- and ends Program.
        //
    }
}
