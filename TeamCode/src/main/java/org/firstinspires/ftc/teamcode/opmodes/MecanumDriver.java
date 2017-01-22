package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.classes.Mecanum;
import org.firstinspires.ftc.teamcode.classes.Range;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="MecanumDrive", group ="Drive")
public class MecanumDriver extends OpMode{
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
	private double initialR = 0;
	private Mecanum yo = new Mecanum();
	private ElapsedTime runtime = new ElapsedTime();

    // constructors
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
		motorC = hardwareMap.dcMotor.get("conveyor_motor");
		motorShootB = hardwareMap.dcMotor.get("shooter_left");
		motorShootT = hardwareMap.dcMotor.get("shooter_right");

		// Servos
		ballRelease = hardwareMap.servo.get("servo_ball");
		ballRelease.setPosition(Servo.MAX_POSITION);

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
			motorShootB.setPower(.9);
			motorShootT.setPower(-.8);
			runtime.reset();
			while (runtime.seconds() < 1){

			}
			ballRelease.setPosition(.85);
			ballRelease.setPosition(ballRelease.MAX_POSITION);
			motorShootB.setPower(0.0);
			motorShootT.setPower(0.0);
		}
		//
        // Raises CapBall
        //
		if (gamepad2.x) {
            motorLS.setPower(-0.5);
            LSRotations = motorLS.getCurrentPosition();
        } else if (gamepad2.y   ) {
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