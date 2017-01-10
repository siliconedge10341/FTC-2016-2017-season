
package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.classes.Mecanum;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

import java.net.PortUnreachableException;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="40_points", group="Pushbot")

public class HitBall extends LinearOpMode {

    /* Declare OpMode members. */
    Mecanum Drive_Train = new Mecanum();

    private DcMotor fr;
    private DcMotor fl;
    private DcMotor bl;
    private DcMotor br;

    private DcMotor motorShootL;
    private DcMotor motorShootR;
    private DcMotor motorCollector;

    private Servo releaseServo;
    private Servo beaconServo;

    private ModernRoboticsI2cRangeSensor rangef;
    private ModernRoboticsI2cRangeSensor rangesl;
    private ModernRoboticsI2cRangeSensor rangesr;

    private OpticalDistanceSensor ods;
    private double initialC = 0;
    private static final Double ticks_per_inch = 510 / (3.1415 * 4);
    ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        fr = hardwareMap.dcMotor.get("fr_motor");
        fl = hardwareMap.dcMotor.get("fl_motor");
        br = hardwareMap.dcMotor.get("br_motor");
        bl = hardwareMap.dcMotor.get("bl_motor");

        rangef = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_front");
        rangesr = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_side_right");
        rangesl = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_side_left");

        motorShootL = hardwareMap.dcMotor.get("shooter_left");
        motorShootR = hardwareMap.dcMotor.get("shooter_right");
        motorCollector = hardwareMap.dcMotor.get("ball_collector");

        releaseServo = hardwareMap.servo.get("servo_ball");
        beaconServo = hardwareMap.servo.get("servo_beacon");
        beaconServo.setPosition(0.25);

        Drive_Train.reset_encoders(fr,fl,br,bl);

        releaseServo.setPosition(-0.2);
        motorShootL.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        motorShootR.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        //ods = hardwareMap.opticalDistanceSensor.get("ods_line");
        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        waitForStart();
        //
        // starting autonomous
        //
        encoderDrive(30.0, "left", 0.5);


        PauseAuto(0.2);

        //
        // Shoot
        //
        releaseServo.setPosition(-0.2);
        PauseAuto(.5);

       motorShootL.setPower(1.0);
        motorShootR.setPower(-1.0);
        motorCollector.setPower(1.0);
        runtime.reset();
        runtime.startTime();
        while (runtime.seconds() < 2){
            telemetry.addData("Seconds" , runtime.seconds());
            telemetry.update();
        }
        runtime.reset();
        runtime.startTime();
        while (runtime.seconds() < 2){
            telemetry.addData("Seconds" , runtime.seconds());
            telemetry.update();
        }
        runtime.reset();
        runtime.startTime();
        while (runtime.seconds() < 2){
            telemetry.addData("Seconds" , runtime.seconds());
            telemetry.update();
        }
        runtime.reset();
        runtime.startTime();
        while (runtime.seconds() < 2){
            telemetry.addData("Seconds" , runtime.seconds());
            telemetry.update();
        }
        motorShootL.setPower(0);
        motorShootR.setPower(0);
        motorCollector.setPower(0);

        //
        // Drives to the center of the field
        //
        encoderDrive(44.0, "left", 0.5);
        Drive_Train.turn_left(fr,fl,br,bl);
        runtime.reset();
        while (runtime.seconds() < 1){

        }
        Drive_Train.brake(fr,fl,br,bl);

        encoderDrive(5.0 , "forward" , 0.5);
        //
        // end of autonomous period
        //
        Drive_Train.brake(fr,fl,br,bl);
        telemetry.addData( "DONE" , "Done");
        telemetry.update();

    }

    // Functions
    public void encoderDrive(double inches, String direction , double power ) {
        int encoderval;
        //
        // Sets the encoders
        //
        Drive_Train.reset_encoders(fr,fl,br,bl);
        encoderval = ticks_per_inch.intValue() * (int) inches;
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
        } else if(direction == "backward"){
            Drive_Train.run_backward(fr,fl,br,bl);
        } else if (direction == "left"){
            Drive_Train.run_left(fr,fl,br,bl);
        } else if (direction == "right"){
            Drive_Train.run_right(fr,fl,br,bl);
        } else if (direction == "diagonal_left_up"){
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
                Drive_Train.run_left_using_alignment(fr,fl,br,bl,rangesl.getDistance(DistanceUnit.CM),rangesr.getDistance(DistanceUnit.CM));
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
    public void PauseAuto(double time) {
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
}
