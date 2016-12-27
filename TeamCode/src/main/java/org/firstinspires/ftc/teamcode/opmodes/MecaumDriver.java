package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.classes.Mecanum;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp(name="MecanumDrive", group ="Drive")

public class MecaumDriver extends OpMode{
	DcMotor motorCollector;
	DcMotor motorFrontRight;
	DcMotor motorFL;
	DcMotor motorBR;
	DcMotor motorBL;
	DcMotor motorShootL;
	DcMotor motorShootR;

	Servo ball;

	int percision_flag=0;
	double ballpos = .5;

	Mecanum yo = new Mecanum();
	public MecaumDriver(){
		percision_flag = 0;
	}

	@Override
	public void init() {

		motorFL = hardwareMap.dcMotor.get("fl_motor");
		motorFrontRight = hardwareMap.dcMotor.get("fr_motor");
		motorBL = hardwareMap.dcMotor.get("bl_motor");
		motorBR = hardwareMap.dcMotor.get("br_motor");

		motorCollector = hardwareMap.dcMotor.get("ball_collector");

		motorShootL = hardwareMap.dcMotor.get("shooter_left");
		motorShootR = hardwareMap.dcMotor.get("shooter_right");

		ball = hardwareMap.servo.get("servo_ball");
		ball.setPosition(ballpos);

	}

	//main function body
	@Override
	public void loop() {
		if (gamepad1.dpad_up) {
			percision_flag++;
		}
		if (percision_flag >= 2) {
			percision_flag = 0;
		}
		yo.set_Power(gamepad1.right_stick_x, gamepad1.left_stick_y, gamepad1.left_stick_x, percision_flag);
		yo.run_motor(motorFrontRight, motorFL, motorBR, motorBL);

		if (gamepad1.a) {
			//TODO: Re-attach to robot later
			motorCollector.setPower(0.5);

		}else{
			//TODO: Re-attach to robot later
			motorCollector.setPower(0);
		}
		//bantu shooter
		if (gamepad1.b){
			motorShootL.setPower(1.0);
			motorShootR.setPower(-1.0);
		}else{
			motorShootL.setPower(0);
			motorShootR.setPower(0);
		}
		if (gamepad2.dpad_right){

			ballpos = ballpos -.01;
		}else if(gamepad2.dpad_left){
			ballpos = ballpos +.01;
		}
		if (ballpos >= Servo.MAX_POSITION){
			ballpos = Servo.MAX_POSITION - .05;
		}else if(ballpos <= Servo.MIN_POSITION ){
			ballpos = Servo.MIN_POSITION +.05;
		}

		ball.setPosition(ballpos);

	}
	@Override
	public void stop() {
		yo.set_Power(0, 0, 0,0);
		// set to zero so the power doesn't influnce any motion or rotation in the robot
		yo.run_motor( motorFrontRight, motorFL, motorBR, motorBL);

	}


}


