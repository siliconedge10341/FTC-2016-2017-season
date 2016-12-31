// Package for -THIS- project
package org.firstinspires.ftc.teamcode.opmodes;

// Imports for variables and phone configuration
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
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
        private DcMotor MtrShootL;

    // Servos
        private Servo srvRelease;

    // Classes
        private Mecanum drive_train = new Mecanum();
        private Range range_sensor = new Range();
        private Light color_sensor = new Light();
        private ElapsedTime runtime = new ElapsedTime();

    //

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
