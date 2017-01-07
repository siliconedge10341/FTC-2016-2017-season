package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.classes.Mecanum;
import org.firstinspires.ftc.teamcode.classes.Range;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="MecanumDrive", group ="Drive")
public class MecanumDriver extends OpMode{
    // instance variables
    // private variables
		// Motors
		private DcMotor motorCollector;
		private DcMotor motorFR;
		private DcMotor motorFL;
		private DcMotor motorBR;
		private DcMotor motorBL;
		private DcMotor motorShootL;
		private DcMotor motorShootR;
		private DcMotor motorLS;

		// Servos
		private Servo ballRelease;
		private Servo beaconServo;

		private boolean percision_flag;
		// percision flag is to decrease power, if power is decreased, the robot will go slower.

	private double ballpos = .5;
    private double LSRotations = 0;
	private double initialR = 0;
	private Mecanum yo = new Mecanum();
    private double power = 0;
    //private Range dist = new Range();

    public MecanumDriver() {
		// default constructor

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
		motorCollector = hardwareMap.dcMotor.get("ball_collector");
		motorShootL = hardwareMap.dcMotor.get("shooter_left");
		motorShootR = hardwareMap.dcMotor.get("shooter_right");

		// Servos
		ballRelease = hardwareMap.servo.get("servo_ball");
		ballRelease.setPosition(0.7);
		beaconServo = hardwareMap.servo.get("servo_beacon");
		beaconServo.setPosition(0.5);

		// Classes
		//dist.setRange(hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "projectile_distance"));

		// Variables
		LSRotations = 0;
		initialR = motorLS.getCurrentPosition();
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
        // A: Turns on Collector        //
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
		// A: none;        				//
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
		// beacon servo
		//
		if (gamepad1.x) {
			beaconServo.setPosition(beaconServo.MIN_POSITION);
		} else if (gamepad1.y) {
			beaconServo.setPosition(0.5);
		}
		//
        // runs the robot
		//
		yo.set_Power(gamepad1.right_stick_x, gamepad1.left_stick_y, gamepad1.left_stick_x,percision_flag);
		yo.run_motor(motorFR, motorFL, motorBR, motorBL);
		//
        // Run Collector
		//
		if (gamepad2.a)  {
			motorCollector.setPower(1.0);
		} else {
			motorCollector.setPower(0);
		}
		//
		// bantu shooter
		//
		if (ballpos <= Servo.MIN_POSITION + .25 || ballpos >= Servo.MAX_POSITION - .25 || gamepad2.b){
            motorShootL.setPower(1.0);
			motorShootR.setPower(-1.0);
		} else {
			motorShootL.setPower(0);
			motorShootR.setPower(0);
		}
		//
        // Moves Servo
		//
		if (gamepad2.dpad_right){
            if (ballpos > Servo.MIN_POSITION) {
                ballpos = ballpos -.05;
            }
		}else if(gamepad2.dpad_left){
            if (ballpos < Servo.MAX_POSITION) {
                ballpos = ballpos +.05;
            }
		}
        ballRelease.setPosition(ballpos);
		//
        // Raises CatBall
        //
		if (gamepad2.x) {
            motorLS.setPower(-0.5);
            LSRotations = motorLS.getCurrentPosition();
        } else if (gamepad2.y && LSRotations > initialR) {
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