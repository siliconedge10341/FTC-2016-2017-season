package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.ftcrobotcontroller.classes.Mecanum;

import com.qualcomm.robotcore.util.Range;

public class MecaumDriver extends OpMode{
	DcMotor motorCollector;
	DcMotor motorFrontRight;
	DcMotor motorFL;
	DcMotor motorBR;
	DcMotor motorBL;
	DcMotor motorShootL;
	DcMotor motorShootR;

	int percision_flag=0;

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

		if (gamepad2.a) {
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

	}
	@Override
	public void stop() {
		yo.set_Power(0, 0, 0,0);
		// set to zero so the power doesn't influnce any motion or rotation in the robot
		yo.run_motor( motorFrontRight, motorFL, motorBR, motorBL);

	}


}


