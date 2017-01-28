package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.classes.Mecanum;

/**
 * Created by bandi on 1/27/2017.
 */



@TeleOp(name = "PID Driver", group = "Drive")
public class PIDTester extends OpMode {
    // instance variables
    // private variables
    // Motors
    private DcMotor motorC;
    private DcMotor motorFR;
    private DcMotor motorFL;
    private DcMotor motorBR;
    private DcMotor motorBL;
    private DcMotor motorShootB;
    private DcMotor motorShootT;
    private DcMotor motorLS;

    // Servos
    private Servo ballRelease;

    private boolean percision_flag = false;
    //
    // percision flag is to decrease power, if power is decreased, the robot will go slower.
    //
    // Variables
    private double LSRotations = 0;
    private double MaxR = -12500;
    private Mecanum yo = new Mecanum();
    private ElapsedTime runtime = new ElapsedTime();

    private double kP = 9000000.0;
    private double kI = 0.0;
    //private double kD = 380000;
    private double kD = 10000.0;

    private double integral = 0.0;
    private double derivative = 0.0;

    private double motorOut = 0.0;
    private double fTarget = 7.5e-7;
    private double fVelocity = 0.0;
    private double fError = 0.0;
    private double fLastError = 0.0;

    private int fEncoder = 0;
    private int fLastEncoder = 0;

    private long fVelocityTime = 0;
    private long fLastVelocityTime = 0;

    private double place = 0.1;

    private boolean firstCross;

    private double tolerance = 0.5e-7;

    private double targetVoltage = 12.5;
    private double voltage;
    //Right Motor
    private double kPRight = 9000000.0;
    private double kIRight = 0.0;
    //private double kD = 380000;
    private double kDRight = 10000.0;

    private double integralRight = 0.0;
    private double derivativeRight = 0.0;

    private double motorOutRight = 0.0;
    private double fTargetRight = -7.5e-7;
    private double fVelocityRight = 0.0;
    private double fErrorRight = 0.0;
    private double fLastErrorRight = 0.0;
    private double tbh = 0.0;

    private int fEncoderRight = 0;
    private int fLastEncoderRight = 0;

    private long fVelocityTimeRight = 0;
    private long fLastVelocityTimeRight = 0;

    // constructors
    public PIDTester() {
        // default constructor

    }
    public void calculateLeftPID(){
        fVelocityTime = System.nanoTime();
        fEncoder = motorShootB.getCurrentPosition();
        fVelocity = (double)(fEncoder - fLastEncoder) / (fVelocityTime - fLastVelocityTime);
        fError = fTarget - fVelocity;

        integral += fError;
        if(fError == 0)
        {
            integral = 0;
        }

        if(Math.abs(fError) > 50)
        {
            integral = 0;
        }

        derivative = fError - fLastError;

        fLastError = fError;
        fLastEncoder = fEncoder;
        fLastVelocityTime = fVelocityTime;

        motorOut = (kP * fError) + (kI * integral) + (kD * derivative);

        motorOut = Range.clip(motorOut, 0.0, 1.0);

        telemetry.addData("1", "LEFTkP " + (kP * fError));
        telemetry.addData("2", "LEFTError " + fError);
        telemetry.addData("3", "LEFTTime " + fVelocityTime);
        telemetry.addData("4", "LEFTEncoder " + fEncoder);
        telemetry.addData("5", "LEFTLast Encoder " + fLastEncoder);
        telemetry.addData("6", "LEFTEncoder Change " + (fEncoder - fLastEncoder));
        telemetry.addData("7", "LEFTTime Change " + (fVelocityTime - fLastVelocityTime));
        telemetry.addData("8", "LEFTVelocity " + fVelocity);
        telemetry.addData("9", "LEFTResult " + motorOut);
        telemetry.update();
        motorShootB.setPower(motorOut);
    }

    public void calculateRightPID(){
        fVelocityTimeRight = System.nanoTime();
        fEncoderRight = motorShootT.getCurrentPosition();
        fVelocityRight = (double)(fEncoder - fLastEncoder) / (fVelocityTime - fLastVelocityTime);
        fErrorRight = fTargetRight - fVelocityRight;

        integralRight += fErrorRight;
        if(fErrorRight == 0)
        {
            integralRight = 0;
        }

        if(Math.abs(fError) > 50)
        {
            integralRight = 0;
        }

        derivative = fError - fLastError;

        fLastError = fError;
        fLastEncoder = fEncoder;
        fLastVelocityTime = fVelocityTime;

        motorOut = (kP * fError) + (kI * integral) + (kD * derivative);

        motorOut = Range.clip(motorOut, 0.0, 1.0);

        telemetry.addData("1", "RIGHTkP " + (kP * fError));
        telemetry.addData("2", "RIGHTError " + fError);
        telemetry.addData("3", "RIGHTTime " + fVelocityTime);
        telemetry.addData("4", "RIGHTEncoder " + fEncoder);
        telemetry.addData("5", "RIGHTLast Encoder " + fLastEncoder);
        telemetry.addData("6", "RIGHTEncoder Change " + (fEncoder - fLastEncoder));
        telemetry.addData("7", "RIGHTTime Change " + (fVelocityTime - fLastVelocityTime));
        telemetry.addData("8", "RIGHTVelocity " + fVelocity);
        telemetry.addData("9", "RIGHTResult " + motorOut);
        telemetry.update();
        motorShootT.setPower(motorOut);
    }
    @Override
    public void init() {
        //
        // Initialize everything
        //
        // Motors
        motorFL = hardwareMap.dcMotor.get("fl_motor");
        motorFR = hardwareMap.dcMotor.get("fr_motor");
        motorBL = hardwareMap.dcMotor.get("bl_motor");
        motorBR = hardwareMap.dcMotor.get("br_motor");
        motorLS = hardwareMap.dcMotor.get("linear_slide_motor");
        motorC = hardwareMap.dcMotor.get("conveyor_motor");
        motorShootB = hardwareMap.dcMotor.get("shooter_right"); // Bottom
        motorShootT = hardwareMap.dcMotor.get("shooter_left");  // Top

        // Servos
        ballRelease = hardwareMap.servo.get("servo_ball");
        ballRelease.setPosition(Servo.MAX_POSITION);

        // Variables
        LSRotations = 0;
        motorLS.setMode(DcMotor.RunMode.RESET_ENCODERS);
        percision_flag = false;
    }

    // loop
    @Override
    public void loop() {
        //-------------KEY--------------//
        //   Start-B Runs the Program   //
        //----GAMEPAD 2 FUNCTION KEY----//
        //          -GAMEPAD-           //
        // UP: none;					//
        // RIGHT: Move Servo Right      //
        // LEFT: Move Servo Left        //
        // DOWN: None;                  //
        //          -BUTTONS-           //
        // A: Runs Conveyor 	        //
        // B: Runs Shooter              //
        // X: Raise Slide               //
        // Y: Lower Slide               //
        //         -TRIGGERED-          //
        // LEFT: None;                  //
        // RIGHT: None;                 //
        //------------------------------//
        //   Start-A Runs the Program   //
        //----GAMEPAD 1 FUNCTION KEY----//
        //         -GAMEPAD L-          //
        // UP: Forward					//
        // RIGHT: Strafe Right		    //
        // LEFT: Strafe Left	        //
        // DOWN: Backward				//
        //         -GAMEPAD R-          //
        // UP: none;					//
        // RIGHT: Turn right			//
        // LEFT: Turn left				//
        // DOWN: none;					//
        //          -BUTTONS-           //
        // A: Runs the Collector		//
        // B: none;             		//
        // X: none;      		        //
        // Y: none;		                //
        //         -TRIGGERED-          //
        // LEFT: Button goes down       //
        // RIGHT: Button goes up        //
        //------------------------------//
        //
        // Power Settings
        //
        if (gamepad1.right_trigger > 0.8) {
            percision_flag = true;
        } else if (gamepad1.left_trigger > 0.8){
            percision_flag = false;
        }
        //
        // runs the robot
        //
        yo.set_Power(gamepad1.right_stick_x, gamepad1.left_stick_y, gamepad1.left_stick_x,percision_flag);
        yo.run_motor(motorFR, motorFL, motorBR, motorBL);
        //
        //
        // Runs the collector
        //
        if (gamepad2.a)  {
            motorC.setPower(.7);
        } else {
            motorC.setPower(0);
        }
        //
        // bantu shooter function
        // This is a 3 second function that lets the motors accelerate and move
        // the servo to shoot one ball.
        //
        if (gamepad2.b) {
            //
            // Max distance
            //
            this.calculateLeftPID();
            this.calculateRightPID();
            runtime.reset();
            while (runtime.seconds() < 1){

            }
            ballRelease.setPosition(.75);
            runtime.reset();
            while (runtime.seconds() < 0.5){

            }
            ballRelease.setPosition(ballRelease.MAX_POSITION);
            motorShootB.setPower(0.0);
            motorShootT.setPower(0.0);
        }
        //
        // Raises CapBall
        //
        if (gamepad2.x){
            //
            // Raise
            //
            motorLS.setPower(-0.5);
            LSRotations = motorLS.getCurrentPosition();
        } else if (gamepad2.y) {
            //
            // Lower
            //
            motorLS.setPower(0.5);
            LSRotations = motorLS.getCurrentPosition();
        } else {
            motorLS.setPower(0.0);
        }
        telemetry.addData("LinearSlideMotor", LSRotations);
        telemetry.update();
    }

    // functions
    @Override
    public void stop() {
        yo.setPowerD(0.0);
        yo.brake(motorFR,motorFL, motorBR, motorBL);
        // set to zero so the power doesn't influence any motion or rotation in the robot

    }
}
