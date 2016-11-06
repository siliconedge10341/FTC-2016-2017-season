package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.ftcrobotcontroller.classes.Mecanum;

import com.qualcomm.robotcore.util.Range;

public class MecaumDriver extends OpMode{

	DcMotor motorFR;
	DcMotor motorFL;
	DcMotor motorBR;
	DcMotor motorBL;

	Mecanum yo = new Mecanum();
	public MecaumDriver(){

	}

	@Override
	public void init() {

		motorFL = hardwareMap.dcMotor.get("bmotor_l");
		motorFR = hardwareMap.dcMotor.get("bmotor_r");
		motorBL = hardwareMap.dcMotor.get("fmotor_l");
		motorBR = hardwareMap.dcMotor.get("fmotor_r");


	}

	//main function body
	@Override
	public void loop() {
		yo.set_Power(gamepad2.right_stick_x, gamepad1.left_stick_y, gamepad1.left_stick_x);
		yo.run_motor( motorFR, motorFL, motorBR, motorBL);

	}


	@Override
	public void stop() {
		yo.set_Power(0, 0, 0);
		// set to zero so the power doesn't influnce any motion or rotation in the robot
		yo.run_motor( motorFR, motorFL, motorBR, motorBL);

	}


}


